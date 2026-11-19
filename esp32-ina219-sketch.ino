
#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <DNSServer.h>
#include <Preferences.h>
#include <math.h>
#include <stdarg.h>
#include <Adafruit_INA219.h>
#include <ESPAsyncWebServer.h>

// ========================================================
//
// COMPILE TIME CONFIGURATION
//
// ========================================================

#define INA219_MAX_SENSORS 6
#define PIN_I2C_SDA 21
#define PIN_I2C_SCL 22
#define PIN_RELAY_UNIFIED_VOLTAGE 25
#define PIN_RELAY_CURRENT 26
#define PIN_RELAY_POWER 27
#define PIN_BOOT_BUTTON 0
#define PIN_STATUS_LED 2
#define CFG_MAGIC_VALUE 0x494E413231395636ULL
#define CFG_VERSION_VALUE 6ULL
#define CFG_NAMESPACE "esp32ina219"
#define CFG_SLOT_A "configA"
#define CFG_SLOT_B "configB"
#define AP_IP_ADDRESS IPAddress(192, 168, 4, 1)
#define AP_GATEWAY IPAddress(192, 168, 4, 1)
#define AP_SUBNET IPAddress(255, 255, 255, 0)
#define JSON_BUFFER_SIZE 16384
#define PAGE_BUFFER_SIZE 44000
#define FORM_BUFFER_SIZE 4096
#define STATUS_HISTORY 48
#define NVS_WRITE_DEBOUNCE_MS 3000ULL
#define NVS_WRITE_MIN_GAP_MS 8000ULL
#define NVS_BOOT_GUARD_MS 5000ULL
#define FACTORY_RESET_HOLD_MS 5000ULL
#define BUTTON_DEBOUNCE_MS 40ULL

// ========================================================
//
// DATA MODEL
//
// ========================================================

struct SensorConfig {
  uint64_t enabled;
  uint64_t address;
  uint64_t calibration;
  uint64_t shuntMilliOhm;
  char name[24];
  uint64_t voltageEnable;
  int64_t voltageMinMilliVolt;
  int64_t voltageMaxMilliVolt;
  uint64_t currentEnable;
  int64_t currentMinMilliAmp;
  int64_t currentMaxMilliAmp;
  uint64_t powerEnable;
  int64_t powerMinMilliWatt;
  int64_t powerMaxMilliWatt;
  uint64_t hysteresisPercent;
  uint64_t debounceSamples;
  uint64_t relayMask;
  uint64_t reserved0;
  uint64_t reserved1;
};

struct Config {
  char wifiSsid[32];
  char wifiPassword[64];
  uint64_t wifiChannel;
  uint64_t wifiHidden;
  uint64_t portalEnabled;
  uint64_t wifiMaxClients;
  uint64_t sampleIntervalMs;
  uint64_t publishIntervalMs;
  uint64_t relayPolarity;
  uint64_t relayLatch;
  uint64_t relayMinOnMs;
  uint64_t relayHoldOffMs;
  uint64_t bootHoldMs;
  uint64_t reserved0;
  uint64_t reserved1;
  uint64_t reserved2;
  SensorConfig sensor[INA219_MAX_SENSORS];
};

struct ConfigRecord {
  uint64_t magic;
  uint64_t version;
  uint64_t sequence;
  uint64_t length;
  uint64_t crc;
  uint64_t savedAtMs;
  Config cfg;
};

struct Reading {
  int64_t busMilliVolt;
  int64_t shuntMicroVolt;
  int64_t currentMicroAmp;
  int64_t powerMicroWatt;
};

struct AlarmState {
  uint64_t active;
  uint64_t latched;
  uint64_t onCount;
  uint64_t offCount;
  uint64_t snoozeUntil;
  uint64_t lastChangeMs;
};

struct SensorState {
  uint64_t begun;
  uint64_t online;
  uint64_t nextProbeMs;
  uint64_t nextReadMs;
  uint64_t lastOkMs;
  uint64_t sampleCount;
  uint64_t failureCount;
  uint64_t statusCode;
  Reading reading;
  AlarmState voltageAlarm;
  AlarmState currentAlarm;
  AlarmState powerAlarm;
};

struct ManualRelayState {
  uint64_t mode;
  uint64_t untilMs;
};

// ========================================================
//
// GLOBAL STATE
//
// ========================================================

Preferences g_preferences;
DNSServer g_dnsServer;
AsyncWebServer g_webServer(80);
Adafruit_INA219 g_ina219Drivers[16] = {
  Adafruit_INA219(0x40), Adafruit_INA219(0x41), Adafruit_INA219(0x42), Adafruit_INA219(0x43),
  Adafruit_INA219(0x44), Adafruit_INA219(0x45), Adafruit_INA219(0x46), Adafruit_INA219(0x47),
  Adafruit_INA219(0x48), Adafruit_INA219(0x49), Adafruit_INA219(0x4A), Adafruit_INA219(0x4B),
  Adafruit_INA219(0x4C), Adafruit_INA219(0x4D), Adafruit_INA219(0x4E), Adafruit_INA219(0x4F)
};

Config g_config;
ConfigRecord g_recordA;
ConfigRecord g_recordB;
SensorState g_sensorState[INA219_MAX_SENSORS];
ManualRelayState g_manualRelay[4];
uint64_t g_clockMs = 0;
uint64_t g_previousMillis = 0;
uint64_t g_bootMs = 0;
uint64_t g_bootCount = 0;
uint64_t g_configSequence = 0;
uint64_t g_configCrc = 0;
uint64_t g_configValid = 0;
uint64_t g_configDirty = 0;
uint64_t g_configDirtyAt = 0;
uint64_t g_activeSlot = 0;
uint64_t g_nvsWrites = 0;
uint64_t g_nvsSkips = 0;
uint64_t g_nvsFailures = 0;
uint64_t g_nvsLastWriteMs = 0;
uint64_t g_relayState[4] = {0, 0, 0, 0};
uint64_t g_relayOnSince[4] = {0, 0, 0, 0};
uint64_t g_relayOffSince[4] = {0, 0, 0, 0};
uint64_t g_relayDemand = 0;
uint64_t g_buttonLastRaw = 1;
uint64_t g_buttonStable = 1;
uint64_t g_buttonStableAt = 0;
uint64_t g_buttonHoldStart = 0;
uint64_t g_buttonProgress = 0;
uint64_t g_resetArmed = 0;
uint64_t g_resetAt = 0;
uint64_t g_rebootAt = 0;
uint64_t g_wifiRestartAt = 0;
uint64_t g_dnsRunning = 0;
uint64_t g_lastWebTick = 0;
uint64_t g_lastPersistCheck = 0;
uint64_t g_lastLedTick = 0;
uint64_t g_ledOn = 0;
uint64_t g_totalSamples = 0;
uint64_t g_totalReadFailures = 0;
uint64_t g_httpRequests = 0;
uint64_t g_httpRejected = 0;
uint64_t g_factoryResetCount = 0;
char g_jsonBuffer[JSON_BUFFER_SIZE];
char g_pageBuffer[PAGE_BUFFER_SIZE];
char g_formBuffer[FORM_BUFFER_SIZE];

// ========================================================
//
// 64 BIT CLOCK AND NUMERIC UTILITIES
//
// ========================================================

void advanceClock(void) {
  uint32_t current = millis();
  uint32_t delta = current - (uint32_t)g_previousMillis;
  g_previousMillis = (uint64_t)current;
  g_clockMs += (uint64_t)delta;
}

uint64_t nowMs(void) {
  return g_clockMs;
}

uint64_t clampU64(uint64_t value, uint64_t minimum, uint64_t maximum) {
  if (value < minimum) return minimum;
  if (value > maximum) return maximum;
  return value;
}

int64_t clampI64(int64_t value, int64_t minimum, int64_t maximum) {
  if (value < minimum) return minimum;
  if (value > maximum) return maximum;
  return value;
}

size_t appendFmt(char *buffer, size_t capacity, size_t &used, const char *format, ...) {
  if (used >= capacity) return 0;
  va_list arguments;
  va_start(arguments, format);
  int written = vsnprintf(buffer + used, capacity - used, format, arguments);
  va_end(arguments);
  if (written < 0) return 0;
  if ((size_t)written >= capacity - used) {
    used = capacity;
    return (size_t)written;
  }
  used += (size_t)written;
  return (size_t)written;
}

void fixedString(int64_t value, uint8_t decimals, char *output, size_t capacity) {
  uint64_t divisor = 1;
  for (uint8_t index = 0; index < decimals; index++) divisor *= 10ULL;
  bool negative = value < 0;
  uint64_t magnitude = negative ? (uint64_t)(-(value + 1)) + 1ULL : (uint64_t)value;
  uint64_t whole = magnitude / divisor;
  uint64_t fraction = magnitude % divisor;
  if (negative) {
    snprintf(output, capacity, "-%llu.%0*llu", (unsigned long long)whole, (int)decimals, (unsigned long long)fraction);
  } else {
    snprintf(output, capacity, "%llu.%0*llu", (unsigned long long)whole, (int)decimals, (unsigned long long)fraction);
  }
}

void sanitizeText(const char *source, char *destination, size_t capacity) {
  size_t position = 0;
  if (capacity == 0) return;
  for (size_t index = 0; source[index] != '\0' && position < capacity - 1; index++) {
    unsigned char character = (unsigned char)source[index];
    if (character >= 32 && character <= 126) destination[position++] = (char)character;
  }
  destination[position] = '\0';
}

void jsonEscape(const char *source, char *destination, size_t capacity) {
  size_t position = 0;
  if (capacity == 0) return;
  for (size_t index = 0; source[index] != '\0' && position < capacity - 3; index++) {
    unsigned char character = (unsigned char)source[index];
    if (character == '"' || character == '\\') {
      destination[position++] = '\\';
      destination[position++] = (char)character;
    } else if (character == '\n' || character == '\r' || character == '\t' || character < 32) {
      destination[position++] = ' ';
    } else {
      destination[position++] = (char)character;
    }
  }
  destination[position] = '\0';
}

uint64_t crc64Update(uint64_t crc, const uint8_t *data, size_t length) {
  for (size_t index = 0; index < length; index++) {
    crc ^= (uint64_t)data[index] << 56;
    for (int bit = 0; bit < 8; bit++) {
      crc = (crc & 0x8000000000000000ULL) ? ((crc << 1) ^ 0x42F0E1EBA9EA3693ULL) : (crc << 1);
    }
  }
  return crc;
}

uint64_t crc64Buffer(const void *data, size_t length) {
  return ~crc64Update(0xFFFFFFFFFFFFFFFFULL, (const uint8_t *)data, length);
}

uint64_t parseU64(const char *text, uint64_t minimum, uint64_t maximum, uint64_t fallback) {
  if (text == NULL || text[0] == '\0') return fallback;
  uint64_t value = 0;
  size_t index = 0;
  bool hex = false;
  if (text[0] == '0' && (text[1] == 'x' || text[1] == 'X')) {
    hex = true;
    index = 2;
    if (text[index] == '\0') return fallback;
  }
  for (; text[index] != '\0'; index++) {
    unsigned char character = (unsigned char)text[index];
    uint64_t digit;
    if (character >= '0' && character <= '9') digit = (uint64_t)(character - '0');
    else if (hex && character >= 'a' && character <= 'f') digit = (uint64_t)(character - 'a' + 10);
    else if (hex && character >= 'A' && character <= 'F') digit = (uint64_t)(character - 'A' + 10);
    else return fallback;
    uint64_t base = hex ? 16ULL : 10ULL;
    if (value > (UINT64_MAX - digit) / base) return fallback;
    value = value * base + digit;
  }
  return clampU64(value, minimum, maximum);
}

int64_t parseI64(const char *text, int64_t minimum, int64_t maximum, int64_t fallback) {
  if (text == NULL || text[0] == '\0') return fallback;
  bool negative = false;
  size_t index = 0;
  if (text[0] == '-') {
    negative = true;
    index = 1;
  } else if (text[0] == '+') {
    index = 1;
  }
  if (text[index] == '\0') return fallback;
  uint64_t value = 0;
  for (; text[index] != '\0'; index++) {
    if (text[index] < '0' || text[index] > '9') return fallback;
    uint64_t digit = (uint64_t)(text[index] - '0');
    if (value > (UINT64_MAX - digit) / 10ULL) return fallback;
    value = value * 10ULL + digit;
  }
  int64_t signedValue;
  if (negative) {
    if (value > (uint64_t)INT64_MAX + 1ULL) return fallback;
    signedValue = (value == (uint64_t)INT64_MAX + 1ULL) ? INT64_MIN : -(int64_t)value;
  } else {
    if (value > (uint64_t)INT64_MAX) return fallback;
    signedValue = (int64_t)value;
  }
  return clampI64(signedValue, minimum, maximum);
}

bool almostEqualFloat(float left, float right) {
  return fabsf(left - right) < 0.0001f;
}

// ========================================================
//
// CONFIGURATION DEFAULTS AND VALIDATION
//
// ========================================================

void setDefaultSensor(SensorConfig &sensor, uint64_t address, const char *name, uint64_t enabled) {
  memset(&sensor, 0, sizeof(SensorConfig));
  sensor.enabled = enabled & 1ULL;
  sensor.address = address;
  sensor.calibration = 0;
  sensor.shuntMilliOhm = 100;
  snprintf(sensor.name, sizeof(sensor.name), "%s", name);
  sensor.voltageEnable = 1;
  sensor.voltageMinMilliVolt = 10000;
  sensor.voltageMaxMilliVolt = 13500;
  sensor.currentEnable = 1;
  sensor.currentMinMilliAmp = -10000;
  sensor.currentMaxMilliAmp = 10000;
  sensor.powerEnable = 1;
  sensor.powerMinMilliWatt = -1000;
  sensor.powerMaxMilliWatt = 150000;
  sensor.hysteresisPercent = 2;
  sensor.debounceSamples = 3;
  sensor.relayMask = 7;
}

void setDefaults(Config &config) {
  memset(&config, 0, sizeof(Config));
  snprintf(config.wifiSsid, sizeof(config.wifiSsid), "ESP32_INA219");
  snprintf(config.wifiPassword, sizeof(config.wifiPassword), "12345678");
  config.wifiChannel = 1;
  config.wifiHidden = 0;
  config.portalEnabled = 1;
  config.wifiMaxClients = 8;
  config.sampleIntervalMs = 250;
  config.publishIntervalMs = 1000;
  config.relayPolarity = 0;
  config.relayLatch = 0;
  config.relayMinOnMs = 1000;
  config.relayHoldOffMs = 1000;
  config.bootHoldMs = FACTORY_RESET_HOLD_MS;
  setDefaultSensor(config.sensor[0], 0x40, "Main Bus", 1);
  setDefaultSensor(config.sensor[1], 0x41, "Sensor 2", 0);
  setDefaultSensor(config.sensor[2], 0x44, "Sensor 3", 0);
  setDefaultSensor(config.sensor[3], 0x45, "Sensor 4", 0);
  setDefaultSensor(config.sensor[4], 0x42, "Sensor 5", 0);
  setDefaultSensor(config.sensor[5], 0x43, "Sensor 6", 0);
}

void validateConfig(Config &config) {
  sanitizeText(config.wifiSsid, config.wifiSsid, sizeof(config.wifiSsid));
  sanitizeText(config.wifiPassword, config.wifiPassword, sizeof(config.wifiPassword));
  if (config.wifiSsid[0] == '\0') snprintf(config.wifiSsid, sizeof(config.wifiSsid), "ESP32_INA219");
  size_t passwordLength = strlen(config.wifiPassword);
  if (passwordLength != 0 && (passwordLength < 8 || passwordLength > 63)) {
    snprintf(config.wifiPassword, sizeof(config.wifiPassword), "12345678");
  }
  config.wifiChannel = clampU64(config.wifiChannel, 1, 13);
  config.wifiHidden = clampU64(config.wifiHidden, 0, 1);
  config.portalEnabled = clampU64(config.portalEnabled, 0, 1);
  config.wifiMaxClients = clampU64(config.wifiMaxClients, 1, 10);
  config.sampleIntervalMs = clampU64(config.sampleIntervalMs, 100, 60000);
  config.publishIntervalMs = clampU64(config.publishIntervalMs, 200, 60000);
  config.relayPolarity = clampU64(config.relayPolarity, 0, 1);
  config.relayLatch = clampU64(config.relayLatch, 0, 1);
  config.relayMinOnMs = clampU64(config.relayMinOnMs, 0, 600000);
  config.relayHoldOffMs = clampU64(config.relayHoldOffMs, 0, 600000);
  config.bootHoldMs = clampU64(config.bootHoldMs, 3000, 15000);
  for (uint64_t index = 0; index < INA219_MAX_SENSORS; index++) {
    SensorConfig &sensor = config.sensor[index];
    sensor.enabled = clampU64(sensor.enabled, 0, 1);
    sensor.address = clampU64(sensor.address, 0x40, 0x4F);
    sensor.calibration = clampU64(sensor.calibration, 0, 2);
    sensor.shuntMilliOhm = clampU64(sensor.shuntMilliOhm, 1, 100000);
    sanitizeText(sensor.name, sensor.name, sizeof(sensor.name));
    if (sensor.name[0] == '\0') snprintf(sensor.name, sizeof(sensor.name), "Sensor %llu", (unsigned long long)(index + 1ULL));
    sensor.voltageEnable = clampU64(sensor.voltageEnable, 0, 1);
    sensor.currentEnable = clampU64(sensor.currentEnable, 0, 1);
    sensor.powerEnable = clampU64(sensor.powerEnable, 0, 1);
    sensor.voltageMinMilliVolt = clampI64(sensor.voltageMinMilliVolt, 0, 32000);
    sensor.voltageMaxMilliVolt = clampI64(sensor.voltageMaxMilliVolt, 0, 32000);
    if (sensor.voltageMaxMilliVolt < sensor.voltageMinMilliVolt) sensor.voltageMaxMilliVolt = sensor.voltageMinMilliVolt;
    sensor.currentMinMilliAmp = clampI64(sensor.currentMinMilliAmp, -300000, 300000);
    sensor.currentMaxMilliAmp = clampI64(sensor.currentMaxMilliAmp, -300000, 300000);
    if (sensor.currentMaxMilliAmp < sensor.currentMinMilliAmp) sensor.currentMaxMilliAmp = sensor.currentMinMilliAmp;
    sensor.powerMinMilliWatt = clampI64(sensor.powerMinMilliWatt, -1000000, 1000000);
    sensor.powerMaxMilliWatt = clampI64(sensor.powerMaxMilliWatt, -1000000, 1000000);
    if (sensor.powerMaxMilliWatt < sensor.powerMinMilliWatt) sensor.powerMaxMilliWatt = sensor.powerMinMilliWatt;
    sensor.hysteresisPercent = clampU64(sensor.hysteresisPercent, 0, 25);
    sensor.debounceSamples = clampU64(sensor.debounceSamples, 1, 20);
    sensor.relayMask = clampU64(sensor.relayMask, 0, 7);
    for (uint64_t previous = 0; previous < index; previous++) {
      if (config.sensor[previous].enabled && config.sensor[previous].address == sensor.address) sensor.enabled = 0;
    }
  }
}

// ========================================================
//
// PERSISTENT STORAGE WITH DUAL SLOT AND WEAR CONTROL
//
// ========================================================

bool recordValid(const ConfigRecord &record) {
  if (record.magic != CFG_MAGIC_VALUE) return false;
  if (record.version != CFG_VERSION_VALUE) return false;
  if (record.length != sizeof(Config)) return false;
  if (record.cfg.wifiSsid[sizeof(record.cfg.wifiSsid) - 1] != '\0') return false;
  if (record.cfg.wifiPassword[sizeof(record.cfg.wifiPassword) - 1] != '\0') return false;
  return record.crc == crc64Buffer(&record.cfg, sizeof(Config));
}

bool readSlot(const char *key, ConfigRecord &record) {
  size_t bytes = g_preferences.getBytes(key, &record, sizeof(ConfigRecord));
  return bytes == sizeof(ConfigRecord);
}

bool writeSlot(const char *key, ConfigRecord &record) {
  size_t bytes = g_preferences.putBytes(key, &record, sizeof(ConfigRecord));
  if (bytes != sizeof(ConfigRecord)) return false;
  ConfigRecord verify;
  if (!readSlot(key, verify)) return false;
  return recordValid(verify) && verify.sequence == record.sequence;
}

void markConfigDirty(void) {
  g_configDirty = 1;
  g_configDirtyAt = nowMs();
}

void persistConfigNow(bool force) {
  uint64_t current = nowMs();
  if (!g_preferences.begin(CFG_NAMESPACE, false)) {
    g_nvsFailures++;
    return;
  }
  if (!force) {
    if (!g_configDirty) {
      g_preferences.end();
      return;
    }
    if (current - g_configDirtyAt < NVS_WRITE_DEBOUNCE_MS) {
      g_preferences.end();
      return;
    }
    if (current < NVS_BOOT_GUARD_MS) {
      g_nvsSkips++;
      g_preferences.end();
      return;
    }
    if (current - g_nvsLastWriteMs < NVS_WRITE_MIN_GAP_MS) {
      g_nvsSkips++;
      g_preferences.end();
      return;
    }
  }
  ConfigRecord candidate;
  memset(&candidate, 0, sizeof(ConfigRecord));
  candidate.magic = CFG_MAGIC_VALUE;
  candidate.version = CFG_VERSION_VALUE;
  candidate.sequence = g_configSequence + 1ULL;
  candidate.length = sizeof(Config);
  candidate.savedAtMs = current;
  candidate.cfg = g_config;
  candidate.crc = crc64Buffer(&candidate.cfg, sizeof(Config));
  const char *target = (g_activeSlot == 0) ? CFG_SLOT_B : CFG_SLOT_A;
  if (!writeSlot(target, candidate)) {
    target = (g_activeSlot == 0) ? CFG_SLOT_A : CFG_SLOT_B;
    if (!writeSlot(target, candidate)) {
      g_nvsFailures++;
      g_preferences.end();
      return;
    }
  }
  g_activeSlot = (target == CFG_SLOT_A) ? 0 : 1;
  g_configSequence = candidate.sequence;
  g_configCrc = candidate.crc;
  g_configDirty = 0;
  g_nvsWrites++;
  g_nvsLastWriteMs = current;
  g_preferences.end();
}

void loadConfig(void) {
  if (!g_preferences.begin(CFG_NAMESPACE, false)) {
    setDefaults(g_config);
    validateConfig(g_config);
    g_configValid = 0;
    return;
  }
  bool haveA = readSlot(CFG_SLOT_A, g_recordA) && recordValid(g_recordA);
  bool haveB = readSlot(CFG_SLOT_B, g_recordB) && recordValid(g_recordB);
  g_preferences.end();
  if (haveA && haveB) {
    if (g_recordB.sequence > g_recordA.sequence) {
      g_config = g_recordB.cfg;
      g_configSequence = g_recordB.sequence;
      g_configCrc = g_recordB.crc;
      g_activeSlot = 1;
    } else {
      g_config = g_recordA.cfg;
      g_configSequence = g_recordA.sequence;
      g_configCrc = g_recordA.crc;
      g_activeSlot = 0;
    }
    g_configValid = 1;
  } else if (haveA) {
    g_config = g_recordA.cfg;
    g_configSequence = g_recordA.sequence;
    g_configCrc = g_recordA.crc;
    g_activeSlot = 0;
    g_configValid = 1;
  } else if (haveB) {
    g_config = g_recordB.cfg;
    g_configSequence = g_recordB.sequence;
    g_configCrc = g_recordB.crc;
    g_activeSlot = 1;
    g_configValid = 1;
  } else {
    setDefaults(g_config);
    g_configValid = 0;
  }
  validateConfig(g_config);
  if (!g_configValid) {
    persistConfigNow(true);
  }
}

void factoryReset(void) {
  if (!g_preferences.begin(CFG_NAMESPACE, false)) return;
  g_preferences.remove(CFG_SLOT_A);
  g_preferences.remove(CFG_SLOT_B);
  g_preferences.end();
  setDefaults(g_config);
  validateConfig(g_config);
  g_configSequence = 0;
  g_configCrc = 0;
  g_activeSlot = 0;
  g_configDirty = 0;
  g_factoryResetCount++;
  g_configValid = 0;
  for (uint64_t index = 0; index < INA219_MAX_SENSORS; index++) {
    memset(&g_sensorState[index], 0, sizeof(SensorState));
    g_sensorState[index].nextProbeMs = nowMs();
    g_sensorState[index].nextReadMs = nowMs();
  }
  for (uint64_t index = 0; index < 4; index++) {
    g_manualRelay[index].mode = 0;
    g_manualRelay[index].untilMs = 0;
  }
}

void scheduleReboot(uint64_t delayMs) {
  g_resetArmed = 1;
  g_rebootAt = nowMs() + delayMs;
}

void scheduleWifiRestart(uint64_t delayMs) {
  g_wifiRestartAt = nowMs() + delayMs;
}

// ========================================================
//
// SENSOR ACQUISITION STATE MACHINE
//
// ========================================================

void resetAlarmState(AlarmState &alarm) {
  alarm.active = 0;
  alarm.latched = 0;
  alarm.onCount = 0;
  alarm.offCount = 0;
  alarm.snoozeUntil = 0;
  alarm.lastChangeMs = 0;
}

Adafruit_INA219 *driverForSensor(uint64_t index) {
  uint64_t offset = clampU64(g_config.sensor[index].address, 0x40, 0x4F) - 0x40ULL;
  return &g_ina219Drivers[offset];
}

void forceSensorReinit(uint64_t index) {
  if (index >= INA219_MAX_SENSORS) return;
  g_sensorState[index].begun = 0;
  g_sensorState[index].online = 0;
  g_sensorState[index].nextProbeMs = nowMs();
  g_sensorState[index].nextReadMs = nowMs();
}

void applySensorCalibration(uint64_t index) {
  if (index >= INA219_MAX_SENSORS) return;
  uint64_t mode = g_config.sensor[index].calibration;
  if (mode == 1ULL) driverForSensor(index)->setCalibration_32V_1A();
  else if (mode == 2ULL) driverForSensor(index)->setCalibration_16V_400mA();
  else driverForSensor(index)->setCalibration_32V_2A();
}

uint64_t evaluateAlarm(AlarmState &alarm, int64_t value, int64_t minimum, int64_t maximum, uint64_t enabled, uint64_t hysteresisPercent, uint64_t debounceSamples, uint64_t currentTime) {
  if (!enabled) {
    resetAlarmState(alarm);
    return 0;
  }
  uint64_t snoozeUntil = alarm.snoozeUntil;
  if (currentTime < snoozeUntil) {
    resetAlarmState(alarm);
    alarm.snoozeUntil = snoozeUntil;
    return 0;
  }
  alarm.snoozeUntil = 0;
  int64_t span = maximum - minimum;
  if (span < 0) span = 0;
  uint64_t spanUnsigned = (uint64_t)span;
  uint64_t hysteresis = (spanUnsigned * hysteresisPercent) / 100ULL;
  uint64_t maximumHysteresis = spanUnsigned / 4ULL;
  if (hysteresis > maximumHysteresis) hysteresis = maximumHysteresis;
  int64_t offset = (int64_t)hysteresis;
  bool breach;
  if (alarm.active) breach = (value < minimum + offset) || (value > maximum - offset);
  else breach = (value < minimum - offset) || (value > maximum + offset);
  if (breach) {
    alarm.onCount++;
    alarm.offCount = 0;
  } else {
    alarm.offCount++;
    alarm.onCount = 0;
  }
  if (!alarm.active && alarm.onCount >= debounceSamples) {
    alarm.active = 1;
    alarm.latched = 1;
    alarm.lastChangeMs = currentTime;
    alarm.onCount = 0;
    alarm.offCount = 0;
  } else if (alarm.active && alarm.offCount >= debounceSamples) {
    alarm.active = 0;
    alarm.lastChangeMs = currentTime;
    alarm.onCount = 0;
    alarm.offCount = 0;
  }
  if (alarm.active) return 1;
  if (g_config.relayLatch && alarm.latched) return 1;
  return 0;
}

void acknowledgeAlarms(uint64_t sensorIndex) {
  uint64_t currentTime = nowMs();
  if (sensorIndex == INA219_MAX_SENSORS) {
    for (uint64_t index = 0; index < INA219_MAX_SENSORS; index++) {
      g_sensorState[index].voltageAlarm.snoozeUntil = currentTime + 30000ULL;
      g_sensorState[index].currentAlarm.snoozeUntil = currentTime + 30000ULL;
      g_sensorState[index].powerAlarm.snoozeUntil = currentTime + 30000ULL;
      g_sensorState[index].voltageAlarm.latched = 0;
      g_sensorState[index].currentAlarm.latched = 0;
      g_sensorState[index].powerAlarm.latched = 0;
    }
    return;
  }
  if (sensorIndex >= INA219_MAX_SENSORS) return;
  g_sensorState[sensorIndex].voltageAlarm.snoozeUntil = currentTime + 30000ULL;
  g_sensorState[sensorIndex].currentAlarm.snoozeUntil = currentTime + 30000ULL;
  g_sensorState[sensorIndex].powerAlarm.snoozeUntil = currentTime + 30000ULL;
  g_sensorState[sensorIndex].voltageAlarm.latched = 0;
  g_sensorState[sensorIndex].currentAlarm.latched = 0;
  g_sensorState[sensorIndex].powerAlarm.latched = 0;
}

void serviceSensors(uint64_t currentTime) {
  g_relayDemand = 0;
  for (uint64_t index = 0; index < INA219_MAX_SENSORS; index++) {
    SensorConfig &sensorConfig = g_config.sensor[index];
    SensorState &state = g_sensorState[index];
    if (!sensorConfig.enabled) {
      state.begun = 0;
      state.online = 0;
      state.statusCode = 0;
      resetAlarmState(state.voltageAlarm);
      resetAlarmState(state.currentAlarm);
      resetAlarmState(state.powerAlarm);
      continue;
    }
    if (!state.begun && currentTime >= state.nextProbeMs) {
      if (driverForSensor(index)->begin()) {
        applySensorCalibration(index);
        state.begun = 1;
        state.online = 1;
        state.lastOkMs = currentTime;
      } else {
        state.begun = 0;
        state.online = 0;
        state.failureCount++;
        state.nextProbeMs = currentTime + 2000ULL;
      }
    }
    bool readFailed = false;
    if (state.begun && currentTime >= state.nextReadMs) {
      state.nextReadMs = currentTime + g_config.sampleIntervalMs;
      Wire.beginTransmission((uint8_t)sensorConfig.address);
      uint8_t busResult = Wire.endTransmission();
      if (busResult != 0) {
        readFailed = true;
      } else {
        float shuntVoltage = driverForSensor(index)->getShuntVoltage_mV();
        float busVoltage = driverForSensor(index)->getBusVoltage_V();
        if (!isfinite((double)shuntVoltage) || !isfinite((double)busVoltage) || busVoltage < 0.0f || busVoltage > 40.0f) {
          readFailed = true;
        } else {
          int64_t shuntMicroVolt = (int64_t)llround((double)shuntVoltage * 1000.0);
          int64_t busMilliVolt = (int64_t)llround((double)busVoltage * 1000.0);
          int64_t currentMicroAmp = (shuntMicroVolt * 1000LL) / (int64_t)sensorConfig.shuntMilliOhm;
          int64_t powerMicroWatt = (busMilliVolt * currentMicroAmp) / 1000LL;
          state.reading.busMilliVolt = busMilliVolt;
          state.reading.shuntMicroVolt = shuntMicroVolt;
          state.reading.currentMicroAmp = currentMicroAmp;
          state.reading.powerMicroWatt = powerMicroWatt;
          state.sampleCount++;
          g_totalSamples++;
          state.lastOkMs = currentTime;
        }
      }
    }
    if (readFailed) {
      state.failureCount++;
      g_totalReadFailures++;
      state.online = 0;
      state.begun = 0;
      state.nextProbeMs = currentTime + 2000ULL;
      state.statusCode = 1;
      resetAlarmState(state.voltageAlarm);
      resetAlarmState(state.currentAlarm);
      resetAlarmState(state.powerAlarm);
      continue;
    }
    if (!state.online || currentTime - state.lastOkMs > (g_config.sampleIntervalMs * 8ULL)) {
      state.online = 0;
      state.statusCode = 1;
      continue;
    }
    state.online = 1;
    uint64_t voltageAssert = evaluateAlarm(state.voltageAlarm, state.reading.busMilliVolt, sensorConfig.voltageMinMilliVolt, sensorConfig.voltageMaxMilliVolt, sensorConfig.voltageEnable, sensorConfig.hysteresisPercent, sensorConfig.debounceSamples, currentTime);
    uint64_t currentAssert = evaluateAlarm(state.currentAlarm, state.reading.currentMicroAmp, sensorConfig.currentMinMilliAmp * 1000LL, sensorConfig.currentMaxMilliAmp * 1000LL, sensorConfig.currentEnable, sensorConfig.hysteresisPercent, sensorConfig.debounceSamples, currentTime);
    uint64_t powerAssert = evaluateAlarm(state.powerAlarm, state.reading.powerMicroWatt, sensorConfig.powerMinMilliWatt * 1000LL, sensorConfig.powerMaxMilliWatt * 1000LL, sensorConfig.powerEnable, sensorConfig.hysteresisPercent, sensorConfig.debounceSamples, currentTime);
    if (voltageAssert || currentAssert || powerAssert) {
      state.statusCode = 3;
      g_relayDemand |= 1ULL;
      if (voltageAssert && (sensorConfig.relayMask & 1ULL)) g_relayDemand |= 2ULL;
      if (currentAssert && (sensorConfig.relayMask & 2ULL)) g_relayDemand |= 4ULL;
      if (powerAssert && (sensorConfig.relayMask & 4ULL)) g_relayDemand |= 8ULL;
    } else {
      state.statusCode = 2;
    }
  }
}

// ========================================================
//
// RELAY OUTPUT ENGINE
//
// ========================================================

void initializeRelays(void) {
  uint64_t pins[3] = {PIN_RELAY_UNIFIED_VOLTAGE, PIN_RELAY_CURRENT, PIN_RELAY_POWER};
  for (uint64_t index = 0; index < 3; index++) {
    pinMode((uint8_t)pins[index], OUTPUT);
    digitalWrite((uint8_t)pins[index], g_config.relayPolarity ? HIGH : LOW);
    g_relayState[index] = 0;
    g_relayOnSince[index] = 0;
    g_relayOffSince[index] = 0;
  }
  for (uint64_t index = 0; index < 4; index++) {
    g_manualRelay[index].mode = 0;
    g_manualRelay[index].untilMs = 0;
  }
}

void updateRelays(uint64_t currentTime) {
  uint64_t pins[3] = {PIN_RELAY_UNIFIED_VOLTAGE, PIN_RELAY_CURRENT, PIN_RELAY_POWER};
  uint64_t channelDemand[4];
  for (uint64_t index = 0; index < 4; index++) {
    ManualRelayState &manual = g_manualRelay[index];
    if (manual.mode == 3ULL && currentTime >= manual.untilMs) {
      manual.mode = 0;
      manual.untilMs = 0;
    }
    uint64_t desired = (g_relayDemand >> index) & 1ULL;
    if (manual.mode == 1ULL || manual.mode == 3ULL) desired = 1;
    if (manual.mode == 2ULL) desired = 0;
    channelDemand[index] = desired;
  }
  uint64_t pinDemand[3] = {channelDemand[0] | channelDemand[1], channelDemand[2], channelDemand[3]};
  for (uint64_t index = 0; index < 3; index++) {
    uint64_t desired = pinDemand[index];
    if (g_relayState[index]) {
      if (!desired && currentTime - g_relayOnSince[index] < g_config.relayMinOnMs) desired = 1;
    } else if (desired && currentTime - g_relayOffSince[index] < g_config.relayHoldOffMs) {
      desired = 0;
    }
    if (desired != g_relayState[index]) {
      g_relayState[index] = desired;
      if (desired) g_relayOnSince[index] = currentTime;
      else g_relayOffSince[index] = currentTime;
      uint64_t activeLevel = g_config.relayPolarity ? LOW : HIGH;
      uint64_t inactiveLevel = g_config.relayPolarity ? HIGH : LOW;
      digitalWrite((uint8_t)pins[index], desired ? (int)activeLevel : (int)inactiveLevel);
    }
  }
}

uint64_t relayPin(uint64_t index) {
  if (index == 0 || index == 1) return PIN_RELAY_UNIFIED_VOLTAGE;
  if (index == 2) return PIN_RELAY_CURRENT;
  return PIN_RELAY_POWER;
}

void setManualRelay(uint64_t index, uint64_t mode, uint64_t seconds) {
  if (index > 3) return;
  ManualRelayState &manual = g_manualRelay[index];
  manual.mode = clampU64(mode, 0, 3);
  manual.untilMs = (manual.mode == 3ULL) ? (nowMs() + clampU64(seconds, 1, 3600) * 1000ULL) : 0ULL;
}

// ========================================================
//
// BOOT BUTTON FACTORY RESET FSM
//
// ========================================================

void serviceFactoryButton(uint64_t currentTime) {
  bool rawPressed = digitalRead(PIN_BOOT_BUTTON) == LOW;
  uint64_t rawValue = rawPressed ? 1ULL : 0ULL;
  if (rawValue != g_buttonLastRaw) {
    g_buttonLastRaw = rawValue;
    g_buttonStableAt = currentTime;
  }
  if (g_buttonStableAt != 0 && currentTime - g_buttonStableAt >= BUTTON_DEBOUNCE_MS && rawValue != g_buttonStable) {
    g_buttonStable = rawValue;
    if (g_buttonStable) g_buttonHoldStart = currentTime;
    else {
      g_buttonHoldStart = 0;
      g_buttonProgress = 0;
    }
  }
  if (g_buttonStable && !g_resetArmed && g_buttonHoldStart != 0) {
    uint64_t heldMs = currentTime - g_buttonHoldStart;
    uint64_t requiredMs = g_config.bootHoldMs;
    g_buttonProgress = clampU64((heldMs * 100ULL) / requiredMs, 0ULL, 100ULL);
    if (heldMs >= requiredMs) {
      factoryReset();
      g_buttonProgress = 100;
      scheduleReboot(1200ULL);
    }
  }
}

void serviceStatusLed(uint64_t currentTime) {
  uint64_t interval = 2000ULL;
  if (g_resetArmed) interval = 120ULL;
  else if (g_buttonStable) interval = 120ULL + (100ULL - g_buttonProgress) * 8ULL;
  else if (g_relayDemand) interval = 400ULL;
  if (currentTime - g_lastLedTick >= interval) {
    g_lastLedTick = currentTime;
    g_ledOn = g_ledOn ? 0ULL : 1ULL;
    digitalWrite(PIN_STATUS_LED, g_ledOn ? HIGH : LOW);
  }
}

void serviceScheduledActions(uint64_t currentTime) {
  if (g_wifiRestartAt != 0 && currentTime >= g_wifiRestartAt) {
    g_wifiRestartAt = 0;
    if (g_dnsRunning) {
      g_dnsServer.stop();
      g_dnsRunning = 0;
    }
    WiFi.softAPdisconnect(true);
    yield();
    WiFi.mode(WIFI_AP);
    WiFi.softAPConfig(AP_IP_ADDRESS, AP_GATEWAY, AP_SUBNET);
    WiFi.softAP(g_config.wifiSsid, g_config.wifiPassword, (int)g_config.wifiChannel, g_config.wifiHidden ? true : false, (int)g_config.wifiMaxClients);
    if (g_config.portalEnabled) {
      g_dnsServer.setErrorReplyCode(DNSReplyCode::NoError);
      g_dnsServer.setTTL(120);
      g_dnsServer.start(53, "*", AP_IP_ADDRESS);
      g_dnsRunning = 1;
    }
  }
  if (g_resetArmed && g_rebootAt != 0 && currentTime >= g_rebootAt) {
    g_rebootAt = 0;
    ESP.restart();
  }
}

// ========================================================
//
// SHARED USER INTERFACE STYLESHEET
//
// ========================================================

const char CSS_TEXT[] PROGMEM = R"rawliteral(
:root{--bg:#0a0f1d;--panel:#121a2b;--panel2:#172136;--line:#26324b;--text:#e9eefb;--muted:#98a6c2;--accent:#5eead4;--accent2:#60a5fa;--ok:#34d399;--warn:#fbbf24;--bad:#fb7185;--shadow:0 10px 30px rgba(0,0,0,.35)}
*{box-sizing:border-box}
html,body{margin:0;padding:0;min-height:100%;background:radial-gradient(1200px 600px at 15% -10%,#1b2b4d 0%,transparent 60%),var(--bg);color:var(--text);font:15px/1.5 system-ui,-apple-system,"Segoe UI",Roboto,Arial,sans-serif;-webkit-font-smoothing:antialiased}
a{color:inherit;text-decoration:none}
button,input,select{font:inherit}
.app{display:grid;grid-template-columns:238px 1fr;min-height:100vh}
.topbar{grid-column:1/-1;position:sticky;top:0;z-index:20;display:flex;align-items:center;gap:14px;padding:12px 20px;background:rgba(10,15,29,.86);backdrop-filter:blur(14px);border-bottom:1px solid var(--line)}
.brand{display:flex;align-items:center;gap:10px;min-width:210px}
.logo{display:grid;place-items:center;width:38px;height:38px;border-radius:12px;background:linear-gradient(135deg,var(--accent),var(--accent2));color:#06131e;font-size:20px;font-weight:800}
.brand strong{display:block;letter-spacing:.06em;font-size:15px}
.brand small{display:block;color:var(--muted);font-size:11px;letter-spacing:.12em;text-transform:uppercase}
.top-status{margin-left:auto;display:flex;align-items:center;gap:10px;color:var(--muted);font-size:12px}
.dot{width:8px;height:8px;border-radius:50%;background:var(--ok);box-shadow:0 0 12px var(--ok)}
.dot.bad{background:var(--bad);box-shadow:0 0 12px var(--bad)}
.menu-btn{display:none;background:var(--panel2);border:1px solid var(--line);color:var(--text);border-radius:10px;padding:7px 11px;cursor:pointer}
.side{position:sticky;top:65px;height:calc(100vh - 65px);padding:18px 12px;border-right:1px solid var(--line);overflow:auto}
.side .group{color:var(--muted);font-size:10px;letter-spacing:.18em;text-transform:uppercase;margin:18px 10px 8px}
.nav-link{display:flex;align-items:center;gap:11px;padding:10px 12px;border-radius:11px;color:var(--muted);font-size:14px;transition:.18s}
.nav-link:hover{background:var(--panel2);color:var(--text)}
.nav-link.active{background:linear-gradient(90deg,rgba(94,234,212,.18),rgba(96,165,250,.08));color:var(--accent);box-shadow:inset 2px 0 0 var(--accent)}
.nav-link span{font-size:17px;width:20px;text-align:center}
.scrim{display:none}
.main{padding:22px 26px 90px;max-width:1600px;width:100%}
.page-head{display:flex;align-items:end;justify-content:space-between;gap:18px;margin-bottom:20px;flex-wrap:wrap}
.page-head h1{margin:0;font-size:25px;letter-spacing:-.02em}
.page-head p{margin:4px 0 0;color:var(--muted);font-size:13px}
.head-actions{display:flex;gap:8px;flex-wrap:wrap}
.banner{display:flex;align-items:center;gap:12px;padding:12px 16px;border-radius:13px;margin-bottom:18px;background:linear-gradient(90deg,rgba(251,113,133,.18),rgba(251,191,36,.06));border:1px solid rgba(251,113,133,.42)}
.banner[hidden]{display:none}
.grid{display:grid;gap:14px}
.cards{grid-template-columns:repeat(auto-fit,minmax(268px,1fr))}
.two{grid-template-columns:repeat(auto-fit,minmax(330px,1fr))}
.card{background:linear-gradient(160deg,var(--panel2),var(--panel));border:1px solid var(--line);border-radius:16px;padding:17px;box-shadow:var(--shadow)}
.card h2,.card h3{margin:0 0 4px;font-size:15px}
.card .sub{color:var(--muted);font-size:11px;letter-spacing:.08em;text-transform:uppercase}
.row{display:flex;align-items:center;justify-content:space-between;gap:10px}
.wrap{flex-wrap:wrap}
.mt{margin-top:14px}
.muted{color:var(--muted)}
.mono{font-family:ui-monospace,SFMono-Regular,Menlo,Consolas,monospace;font-variant-numeric:tabular-nums}
.metric{font-size:27px;font-weight:700;letter-spacing:-.02em;line-height:1.2}
.metric small{font-size:12px;color:var(--muted);font-weight:500;margin-left:4px}
.kpis{display:grid;grid-template-columns:repeat(3,1fr);gap:8px;margin-top:14px}
.kpi{background:rgba(255,255,255,.03);border:1px solid var(--line);border-radius:11px;padding:9px}
.kpi b{display:block;font-size:15px}
.kpi span{font-size:10px;color:var(--muted);text-transform:uppercase;letter-spacing:.08em}
.badge{display:inline-flex;align-items:center;gap:6px;padding:3px 9px;border-radius:999px;font-size:10px;font-weight:700;letter-spacing:.08em;text-transform:uppercase;background:rgba(52,211,153,.13);color:var(--ok)}
.badge.warn{background:rgba(251,191,36,.14);color:var(--warn)}
.badge.bad{background:rgba(251,113,133,.15);color:var(--bad)}
.badge.off{background:rgba(152,166,194,.13);color:var(--muted)}
.spark{width:100%;height:54px;display:block;margin-top:12px;overflow:visible}
.labels{display:flex;justify-content:space-between;color:var(--muted);font-size:10px;margin-top:2px}
form .fields{display:grid;gap:12px;grid-template-columns:repeat(auto-fit,minmax(180px,1fr))}
.field label{display:block;color:var(--muted);font-size:11px;margin-bottom:5px;letter-spacing:.04em}
.field input,.field select{width:100%;background:#0c1424;border:1px solid var(--line);color:var(--text);border-radius:10px;padding:9px 10px;outline:none}
.field input:focus,.field select:focus{border-color:var(--accent);box-shadow:0 0 0 3px rgba(94,234,212,.12)}
.field .hint{display:block;color:var(--muted);font-size:10px;margin-top:4px}
fieldset{border:1px solid var(--line);border-radius:14px;padding:15px;margin:0 0 14px}
legend{padding:0 7px;color:var(--accent);font-size:11px;letter-spacing:.14em;text-transform:uppercase}
.btn{display:inline-flex;align-items:center;justify-content:center;gap:7px;border:1px solid var(--line);background:var(--panel2);color:var(--text);border-radius:10px;padding:9px 14px;cursor:pointer;font-size:13px;font-weight:600;transition:.16s}
.btn:hover{border-color:var(--accent);transform:translateY(-1px)}
.btn.primary{background:linear-gradient(135deg,var(--accent),var(--accent2));color:#06131e;border-color:transparent}
.btn.danger{color:var(--bad);border-color:rgba(251,113,133,.45)}
.btn.small{padding:6px 10px;font-size:12px}
.btn.active{background:rgba(94,234,212,.18);border-color:var(--accent);color:var(--accent)}
.btn-row{display:flex;gap:8px;flex-wrap:wrap;margin-top:13px}
.switch{display:flex;align-items:center;justify-content:space-between;gap:10px;padding:9px 0;border-bottom:1px solid rgba(255,255,255,.04)}
.switch:last-child{border-bottom:0}
.switch input{appearance:none;width:38px;height:21px;border-radius:999px;background:#33405c;position:relative;cursor:pointer;flex:0 0 auto;transition:.2s}
.switch input:after{content:"";position:absolute;width:15px;height:15px;border-radius:50%;background:#dbe5fa;top:3px;left:3px;transition:.2s}
.switch input:checked{background:var(--accent)}
.switch input:checked:after{transform:translateX(17px);background:#06131e}
.switch span{font-size:13px}
.progress{height:8px;border-radius:999px;background:#0c1424;border:1px solid var(--line);overflow:hidden;margin-top:9px}
.progress i{display:block;height:100%;width:0;background:linear-gradient(90deg,var(--warn),var(--bad));transition:width .2s}
.table-wrap{overflow:auto;border:1px solid var(--line);border-radius:13px}
table{border-collapse:collapse;width:100%;min-width:640px;font-size:13px}
th,td{text-align:left;padding:10px 12px;border-bottom:1px solid var(--line)}
th{color:var(--muted);font-size:10px;letter-spacing:.1em;text-transform:uppercase;background:rgba(255,255,255,.02)}
tr:last-child td{border-bottom:0}
.toast{position:fixed;right:22px;bottom:24px;z-index:60;background:var(--panel2);border:1px solid var(--line);border-radius:12px;padding:12px 16px;box-shadow:var(--shadow);font-size:13px;opacity:0;transform:translateY(14px);pointer-events:none;transition:.22s;max-width:min(380px,calc(100vw - 44px))}
.toast.show{opacity:1;transform:translateY(0)}
.toast.ok{border-color:var(--ok)}
.toast.err{border-color:var(--bad)}
.bottom{display:none}
.foot{margin-top:26px;color:var(--muted);font-size:11px;text-align:center}
@media(max-width:880px){.app{display:block}.side{display:none}.main{padding:16px 14px 92px}.brand{min-width:0}.menu-btn{display:block}.top-status{font-size:11px}.cards{gap:11px}.bottom{display:flex;position:fixed;left:0;right:0;bottom:0;z-index:30;background:rgba(10,15,29,.94);backdrop-filter:blur(12px);border-top:1px solid var(--line);padding:7px 4px calc(7px + env(safe-area-inset-bottom))}.bottom a{flex:1;text-align:center;color:var(--muted);font-size:10px;padding:4px 0}.bottom a.active{color:var(--accent)}.bottom a span{display:block;font-size:18px}.scrim.show{display:block;position:fixed;inset:65px 0 0;background:rgba(0,0,0,.55);z-index:15}.page-head h1{font-size:21px}.metric{font-size:24px}}
@media(max-width:880px){body.nav-open .side{display:block;position:fixed;left:0;top:65px;bottom:0;width:238px;z-index:16;background:var(--bg);height:auto}}
@media(max-width:420px){.top-status .hide-sm{display:none}.kpis{grid-template-columns:repeat(2,1fr)}}
)rawliteral";

// ========================================================
//
// SHARED CLIENT SIDE APPLICATION LOGIC
//
// ========================================================

const char JS_TEXT[] PROGMEM = R"rawliteral(
const $=(s,r)=>(r||document).querySelector(s);
const $$=(s,r)=>Array.from((r||document).querySelectorAll(s));
const page=document.body.dataset.page||'dashboard';
let history={};
const esc=(v)=>{return String(v==null?'':v).replace(/[&<>"']/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]))}
const toast=(msg,kind)=>{const t=$('#toast');t.textContent=msg;t.className='toast show '+(kind||'');clearTimeout(t._timer);t._timer=setTimeout(()=>t.className='toast',3200)}
const api=async(path,options)=>{try{const r=await fetch(path,options||{});const data=await r.json().catch(()=>({ok:false,error:'Invalid response'}));if(!r.ok||!data.ok){throw new Error(data.error||('HTTP '+r.status))}return data}catch(e){toast(e.message,'err');throw e}}
const post=(path,payload)=>{return api(path,{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:new URLSearchParams(payload)})}
const action=async(name,extra)=>{const p=Object.assign({action:name},extra||{});try{await post('/api/action',p);toast(name.replace(/_/g,' ')+' accepted','ok');setTimeout(refreshPage,450)}catch(e){}}
const makeSpark=(values,color)=>{const w=240,h=54,n=values.length;if(!n)return '';let min=Math.min.apply(null,values),max=Math.max.apply(null,values);if(max-min<1){max=min+1}const pts=values.map((v,i)=>((i/(Math.max(1,n-1))*w).toFixed(1)+','+(h-4-((v-min)/(max-min))*(h-10)).toFixed(1))).join(' ');return '<svg class="spark" viewBox="0 0 '+w+' '+h+'" preserveAspectRatio="none"><polyline fill="none" stroke="'+color+'" stroke-width="2" stroke-linecap="round" stroke-linejoin="round" points="'+pts+'"/></svg>'}
const statusBadge=(code,latched)=>{if(code===0)return '<span class="badge off">Disabled</span>';if(code===1)return '<span class="badge off">Offline</span>';if(code===3||latched)return '<span class="badge bad">Alarm</span>';return '<span class="badge">Normal</span>'}
const refreshDashboard=()=>{return api('/api/status').then(s=>{$('#uptime').textContent=s.uptimeText;$('#heap').textContent=s.heapFree+' B';$('#clients').textContent=String(s.wifi.clients);$('#cfgstate').textContent=s.config.valid?'Valid':'Defaults';$('#alarmCount').textContent=String(s.alarmCount);$('#banner').hidden=s.alarmCount===0;$('#bannerText').textContent=s.alarmCount+' active threshold alarm'+(s.alarmCount===1?'':'s');$('#resetFill').style.width=s.reset.progress+'%';$('#resetWrap').hidden=s.reset.progress===0&&!s.reset.armed;const host=$('#sensorCards');host.innerHTML=s.sensors.map(x=>{const key='s'+x.idx;history[key]=history[key]||[];history[key].push(Number(x.curMa));if(history[key].length>48)history[key].shift();const latched=x.vLatch||x.iLatch||x.pLatch;return '<article class="card"><div class="row"><div><h2>'+esc(x.name)+'</h2><div class="sub">'+(x.enabled?'I2C '+esc(x.addr):'Disabled')+'</div></div>'+statusBadge(x.status,latched)+'</div><div class="metric mono mt">'+esc(x.voltText)+'<small>V</small></div>'+makeSpark(history[key],x.status===3?'#fb7185':'#5eead4')+'<div class="labels"><span>current history</span><span>'+x.samples+' samples</span></div><div class="kpis"><div class="kpi"><b class="mono">'+esc(x.curText)+'</b><span>Current A</span></div><div class="kpi"><b class="mono">'+esc(x.powerText)+'</b><span>Power W</span></div><div class="kpi"><b class="mono">'+x.ageMs+'ms</b><span>Age</span></div></div><div class="btn-row"><button class="btn small" data-ack="'+x.idx+'">Acknowledge</button><a class="btn small" href="/sensors">Thresholds</a></div></article>'}).join('');$$('[data-ack]').forEach(b=>b.onclick=()=>action('ack',{index:b.dataset.ack}));const rp=$('#relayPills');rp.innerHTML=s.relays.lines.map(r=>'<div class="row"><span class="muted">'+esc(r.label)+'</span><span class="badge '+(r.state?'':'off')+'">'+(r.state?'Energized':'Idle')+'</span></div>').join('')})}
const refreshWifi=()=>{return api('/api/status').then(s=>{$('#clientCount').textContent=String(s.wifi.clients);$('#ssidNow').textContent=s.wifi.ssid;$('#apIp').textContent=s.wifi.ip;$('#portalNow').textContent=s.wifi.portal?'Enabled':'Disabled'})}
const refreshPage=()=>{if(page==='dashboard')return refreshDashboard();if(page==='wifi')return refreshWifi();if(page==='system')return api('/api/status').then(s=>{$('#sysUptime').textContent=s.uptimeText;$('#sysHeap').textContent=s.heapFree+' B';$('#sysHeapMin').textContent=s.heapMin+' B';$('#sysWrites').textContent=String(s.storage.writes);$('#sysSkips').textContent=String(s.storage.skips);$('#sysCrc').textContent=s.config.crc;$('#sysSamples').textContent=String(s.samples);$('#sysFailures').textContent=String(s.failures);$('#resetFill').style.width=s.reset.progress+'%';$('#resetWrap').hidden=s.reset.progress===0&&!s.reset.armed})}
$$('form[data-api]').forEach(f=>f.addEventListener('submit',async e=>{e.preventDefault();const fd=new FormData(f);const payload={};fd.forEach((v,k)=>payload[k]=String(v));try{const r=await api(f.dataset.api,{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:new URLSearchParams(payload)});toast(r.message||'Settings saved','ok');if(f.dataset.reload==='true')setTimeout(()=>location.reload(),700);else refreshPage()}catch(err){}}));
$$('[data-action]').forEach(b=>b.addEventListener('click',()=>{if(b.dataset.confirm&&!confirm(b.dataset.confirm))return;const extra=b.dataset.extra?JSON.parse(b.dataset.extra):{};if(b.dataset.confirm)extra.confirm='yes';action(b.dataset.action,extra)}));
$$('[data-relay]').forEach(b=>b.addEventListener('click',()=>post('/api/action',{action:'relay',index:b.dataset.relay,mode:b.dataset.mode,duration:b.dataset.duration||'30'}).then(()=>toast('Relay command applied','ok')).then(refreshPage).catch(()=>{})));
$$('[data-toggle-pass]').forEach(b=>b.addEventListener('click',()=>{const i=$(b.dataset.togglePass);i.type=i.type==='password'?'text':'password'}));
$('#menuBtn')&&$('#menuBtn').addEventListener('click',()=>document.body.classList.toggle('nav-open'));
$('.scrim')&&$('.scrim').addEventListener('click',()=>document.body.classList.remove('nav-open'));
refreshPage().catch(()=>{});
setInterval(()=>refreshPage().catch(()=>{}),2000);
)rawliteral";

// ========================================================
//
// PAGE COMPOSITION AND NAVIGATION
//
// ========================================================

const char *checkedAttribute(uint64_t value) {
  return value ? "checked" : "";
}

const char *activeAttribute(const char *active, const char *target) {
  return strcmp(active, target) == 0 ? " active" : "";
}

void replaceToken(char *buffer, const char *token, const char *value) {
  size_t tokenLength = strlen(token);
  size_t valueLength = strlen(value);
  if (tokenLength == 0) return;
  char *cursor = buffer;
  while ((cursor = strstr(cursor, token)) != NULL) {
    size_t used = strlen(buffer);
    size_t tailLength = strlen(cursor + tokenLength) + 1;
    if (valueLength > tokenLength && used + valueLength - tokenLength >= PAGE_BUFFER_SIZE) return;
    if (valueLength < tokenLength) memmove(cursor + valueLength, cursor + tokenLength, tailLength);
    else if (valueLength > tokenLength) memmove(cursor + valueLength, cursor + tokenLength, tailLength);
    memcpy(cursor, value, valueLength);
    cursor += valueLength;
  }
}

void addNavigationLink(size_t &used, const char *href, const char *icon, const char *label, const char *active, const char *target) {
  appendFmt(g_pageBuffer, sizeof(g_pageBuffer), used, "<a class=\"nav-link%s\" href=\"%s\"><span>%s</span>%s</a>", activeAttribute(active, target), href, icon, label);
}

void sendPage(AsyncWebServerRequest *request, const char *pageId, const char *title, const char *subtitle, const char *active, void (*bodyBuilder)(size_t &used), const char *extraScript) {
  size_t used = 0;
  appendFmt(g_pageBuffer, sizeof(g_pageBuffer), used,
    "<!doctype html><html lang=\"en\"><head><meta charset=\"utf-8\"><meta name=\"viewport\" content=\"width=device-width,initial-scale=1,viewport-fit=cover\"><meta name=\"theme-color\" content=\"#0a0f1d\"><title>%s - INA219 Monitor</title><style>%s</style></head><body data-page=\"%s\">"
    "<div class=\"app\"><header class=\"topbar\"><button class=\"menu-btn\" id=\"menuBtn\" aria-label=\"Toggle navigation\">☰</button><div class=\"brand\"><div class=\"logo\">⚡</div><div><strong>INA219 CONTROL</strong><small>Power Monitor</small></div></div>"
    "<div class=\"top-status\"><span class=\"dot\" id=\"liveDot\"></span><span class=\"hide-sm\">AP</span><span class=\"mono\">192.168.4.1</span></div></header>"
    "<aside class=\"side\"><div class=\"group\">Monitor</div>", title, CSS_TEXT, pageId);
  addNavigationLink(used, "/", "◉", "Dashboard", active, "dashboard");
  addNavigationLink(used, "/sensors", "◈", "Sensor Settings", active, "sensors");
  addNavigationLink(used, "/relays", "⏻", "Relay Control", active, "relays");
  appendFmt(g_pageBuffer, sizeof(g_pageBuffer), used, "<div class=\"group\">Device</div>");
  addNavigationLink(used, "/wifi", "⌁", "Wi-Fi Access Point", active, "wifi");
  addNavigationLink(used, "/system", "⚙", "System & Diagnostics", active, "system");
  appendFmt(g_pageBuffer, sizeof(g_pageBuffer), used,
    "<div class=\"group\">Network</div><div class=\"card\"><div class=\"sub\">Access point</div><div class=\"mono mt\">ESP32_INA219</div><div class=\"muted mt\" style=\"font-size:11px\">Captive portal redirects all clients to this dashboard.</div></div></aside>"
    "<div class=\"scrim\"></div><main class=\"main\"><div class=\"page-head\"><div><h1>%s</h1><p>%s</p></div><div class=\"head-actions\"><a class=\"btn\" href=\"/\">Dashboard</a><button class=\"btn\" data-action=\"refresh_page\">Refresh</button></div></div>", title, subtitle);
  bodyBuilder(used);
  appendFmt(g_pageBuffer, sizeof(g_pageBuffer), used,
    "<div class=\"foot\">ESP32 INA219 Production Firmware · 64-bit core · non-blocking scheduler · local-only control plane</div></main>"
    "<nav class=\"bottom\"><a class=\"%s\" href=\"/\"><span>◉</span>Home</a><a class=\"%s\" href=\"/sensors\"><span>◈</span>Sensors</a><a class=\"%s\" href=\"/relays\"><span>⏻</span>Relays</a><a class=\"%s\" href=\"/wifi\"><span>⌁</span>Wi-Fi</a><a class=\"%s\" href=\"/system\"><span>⚙</span>System</a></nav>"
    "<div class=\"toast\" id=\"toast\" role=\"status\" aria-live=\"polite\"></div></div><script>%s</script><script>%s</script></body></html>",
    activeAttribute(active, "dashboard"), activeAttribute(active, "sensors"), activeAttribute(active, "relays"), activeAttribute(active, "wifi"), activeAttribute(active, "system"),
    JS_TEXT, (extraScript == NULL) ? "" : extraScript);
  request->send(200, "text/html; charset=utf-8", g_pageBuffer);
}

// ========================================================
//
// DASHBOARD STATUS PAGE
//
// ========================================================

void buildDashboardPage(size_t &used) {
  appendFmt(g_pageBuffer, sizeof(g_pageBuffer), used, R"HTML(
<div class="banner" id="banner" hidden><b id="bannerText"></b><button class="btn small" data-action="ack" data-extra='{"index":"6"}' style="margin-left:auto">Acknowledge all</button></div>
<div class="grid cards" id="sensorCards"></div>
<section class="card mt"><div class="row"><div><h2>Relay Outputs</h2><div class="sub">Live demand and physical state</div></div><span class="badge off" id="alarmBadge">Alarms <b id="alarmCount">0</b></span></div>
<div class="grid two mt" id="relayPills"></div>
<div class="row wrap mt"><button class="btn small" data-action="ack" data-extra='{"index":"6"}'>Acknowledge all alarms</button><button class="btn small" data-relay="0" data-mode="0">Unified auto</button><button class="btn small" data-relay="0" data-mode="2">Unified force off</button></div></section>
<section class="grid two mt">
<div class="card"><h2>Runtime</h2><div class="sub">Controller health</div><div class="kpis"><div class="kpi"><b class="mono" id="uptime">-</b><span>Uptime</span></div><div class="kpi"><b class="mono" id="heap">-</b><span>Free heap</span></div><div class="kpi"><b class="mono" id="clients">-</b><span>AP clients</span></div></div><div class="row mt"><span class="muted">Configuration</span><span class="badge" id="cfgstate">-</span></div></div>
<div class="card"><h2>Boot Button Reset</h2><div class="sub">GPIO0 hold for 5 seconds</div><div class="muted mt" style="font-size:12px">Keep the BOOT button pressed to erase configuration and restore factory defaults. The controller reboots automatically.</div><div class="progress" id="resetWrap" hidden><i id="resetFill"></i></div></div>
</section>
)HTML");
}

// ========================================================
//
// SENSOR SETTINGS PAGE
//
// ========================================================

void buildSensorSettingsPage(size_t &used) {
  appendFmt(g_pageBuffer, sizeof(g_pageBuffer), used, R"HTML(
<form data-api="/api/config/sensors" data-reload="false" autocomplete="off">
<div class="card"><div class="row wrap"><div><h2>Sensor Fleet</h2><div class="sub">Up to six INA219 devices on GPIO21 SDA / GPIO22 SCL</div></div><button class="btn primary" type="submit">Save sensor settings</button></div>
<div class="muted mt" style="font-size:12px">Thresholds are integer values in millivolts, milliamps and milliwatts. Hysteresis and sample debounce prevent relay chatter. The unified relay reacts to any enabled threshold.</div></div>
)HTML");
  for (uint64_t index = 0; index < INA219_MAX_SENSORS; index++) {
    SensorConfig &sensor = g_config.sensor[index];
    appendFmt(g_pageBuffer, sizeof(g_pageBuffer), used,
      "<fieldset class=\"mt\"><legend>Sensor %llu</legend>"
      "<div class=\"switch\"><span><b>Enable this sensor</b></span><input type=\"checkbox\" name=\"s%llu_en\" value=\"1\" %s></div>"
      "<div class=\"fields mt\">"
      "<div class=\"field\"><label>Label</label><input name=\"s%llu_name\" maxlength=\"23\" value=\"%s\"></div>"
      "<div class=\"field\"><label>I2C address</label><select name=\"s%llu_addr\">",
      (unsigned long long)(index + 1ULL), (unsigned long long)index, checkedAttribute(sensor.enabled),
      (unsigned long long)index, sensor.name, (unsigned long long)index);
    for (uint64_t address = 0x40; address <= 0x4F; address++) {
      appendFmt(g_pageBuffer, sizeof(g_pageBuffer), used, "<option value=\"0x%02llX\"%s>0x%02llX</option>",
        (unsigned long long)address, sensor.address == address ? " selected" : "", (unsigned long long)address);
    }
    appendFmt(g_pageBuffer, sizeof(g_pageBuffer), used,
      "</select><span class=\"hint\">Adafruit breakouts: 0x40, 0x41, 0x44, 0x45</span></div>"
      "<div class=\"field\"><label>Measurement range</label><select name=\"s%llu_cal\">"
      "<option value=\"0\"%s>±320 mV shunt / 32 V bus</option>"
      "<option value=\"1\"%s>±160 mV shunt / 32 V bus</option>"
      "<option value=\"2\"%s>±40 mV shunt / 16 V bus</option></select></div>"
      "<div class=\"field\"><label>Shunt resistance (mΩ)</label><input type=\"number\" min=\"1\" max=\"100000\" step=\"1\" name=\"s%llu_shunt\" value=\"%llu\"><span class=\"hint\">0.1 Ω = 100 mΩ</span></div>"
      "</div><div class=\"fields mt\">"
      "<div class=\"switch\"><span>Voltage threshold</span><input type=\"checkbox\" name=\"s%llu_ven\" value=\"1\" %s></div>"
      "<div class=\"field\"><label>Minimum voltage (mV)</label><input type=\"number\" min=\"0\" max=\"32000\" name=\"s%llu_vmin\" value=\"%lld\"></div>"
      "<div class=\"field\"><label>Maximum voltage (mV)</label><input type=\"number\" min=\"0\" max=\"32000\" name=\"s%llu_vmax\" value=\"%lld\"></div>"
      "</div><div class=\"fields mt\">"
      "<div class=\"switch\"><span>Current threshold</span><input type=\"checkbox\" name=\"s%llu_ien\" value=\"1\" %s></div>"
      "<div class=\"field\"><label>Minimum current (mA)</label><input type=\"number\" min=\"-300000\" max=\"300000\" name=\"s%llu_imin\" value=\"%lld\"></div>"
      "<div class=\"field\"><label>Maximum current (mA)</label><input type=\"number\" min=\"-300000\" max=\"300000\" name=\"s%llu_imax\" value=\"%lld\"></div>"
      "</div><div class=\"fields mt\">"
      "<div class=\"switch\"><span>Power threshold</span><input type=\"checkbox\" name=\"s%llu_pen\" value=\"1\" %s></div>"
      "<div class=\"field\"><label>Minimum power (mW)</label><input type=\"number\" min=\"-1000000\" max=\"1000000\" name=\"s%llu_pmin\" value=\"%lld\"></div>"
      "<div class=\"field\"><label>Maximum power (mW)</label><input type=\"number\" min=\"-1000000\" max=\"1000000\" name=\"s%llu_pmax\" value=\"%lld\"></div>"
      "</div><div class=\"fields mt\">"
      "<div class=\"field\"><label>Hysteresis (%)</label><input type=\"number\" min=\"0\" max=\"25\" name=\"s%llu_hyst\" value=\"%llu\"></div>"
      "<div class=\"field\"><label>Debounce samples</label><input type=\"number\" min=\"1\" max=\"20\" name=\"s%llu_deb\" value=\"%llu\"></div>"
      "<div class=\"field\"><label>Relay routing</label><div class=\"muted\" style=\"font-size:11px;margin-top:7px\">",
      (unsigned long long)index,
      sensor.calibration == 0 ? " selected" : "", sensor.calibration == 1 ? " selected" : "", sensor.calibration == 2 ? " selected" : "",
      (unsigned long long)index, (unsigned long long)sensor.shuntMilliOhm,
      (unsigned long long)index, checkedAttribute(sensor.voltageEnable), (unsigned long long)index, (long long)sensor.voltageMinMilliVolt, (unsigned long long)index, (long long)sensor.voltageMaxMilliVolt,
      (unsigned long long)index, checkedAttribute(sensor.currentEnable), (unsigned long long)index, (long long)sensor.currentMinMilliAmp, (unsigned long long)index, (long long)sensor.currentMaxMilliAmp,
      (unsigned long long)index, checkedAttribute(sensor.powerEnable), (unsigned long long)index, (long long)sensor.powerMinMilliWatt, (unsigned long long)index, (long long)sensor.powerMaxMilliWatt,
      (unsigned long long)index, (unsigned long long)sensor.hysteresisPercent,
      (unsigned long long)index, (unsigned long long)sensor.debounceSamples);
    appendFmt(g_pageBuffer, sizeof(g_pageBuffer), used,
      "<label class=\"switch\"><span>Voltage → GPIO25</span><input type=\"checkbox\" name=\"s%llu_r1\" value=\"1\" %s></label>"
      "<label class=\"switch\"><span>Current → GPIO26</span><input type=\"checkbox\" name=\"s%llu_r2\" value=\"1\" %s></label>"
      "<label class=\"switch\"><span>Power → GPIO27</span><input type=\"checkbox\" name=\"s%llu_r3\" value=\"1\" %s></label>"
      "</div></div></div></fieldset>",
      (unsigned long long)index, checkedAttribute((sensor.relayMask & 1ULL) != 0),
      (unsigned long long)index, checkedAttribute((sensor.relayMask & 2ULL) != 0),
      (unsigned long long)index, checkedAttribute((sensor.relayMask & 4ULL) != 0));
  }
  appendFmt(g_pageBuffer, sizeof(g_pageBuffer), used, "<div class=\"btn-row\"><button class=\"btn primary\" type=\"submit\">Save sensor settings</button><a class=\"btn\" href=\"/\">Cancel</a></div></form>");
}

// ========================================================
//
// RELAY CONTROL PAGE
//
// ========================================================

void buildRelayPage(size_t &used) {
  appendFmt(g_pageBuffer, sizeof(g_pageBuffer), used, R"HTML(
<div class="grid two">
<form class="card" data-api="/api/config/relays"><h2>Relay Behaviour</h2><div class="sub">Safety timing and output polarity</div>
<div class="switch mt"><span><b>Active-low outputs</b><br><span class="muted">Enabled for modules that energise on LOW</span></span><input type="checkbox" name="polarity" value="1" POLARITY_CHECKED></div>
<div class="switch"><span><b>Latch alarms until acknowledged</b></span><input type="checkbox" name="latch" value="1" LATCH_CHECKED></div>
<div class="fields mt">
<div class="field"><label>Minimum ON time (ms)</label><input type="number" min="0" max="600000" name="minon" value="MIN_ON"></div>
<div class="field"><label>Hold-off OFF time (ms)</label><input type="number" min="0" max="600000" name="holdoff" value="HOLD_OFF"></div>
<div class="field"><label>Sample interval (ms)</label><input type="number" min="100" max="60000" name="sample" value="SAMPLE_MS"></div>
</div><div class="btn-row"><button class="btn primary" type="submit">Save relay settings</button></div></form>
<div class="card"><h2>Pin Assignment</h2><div class="sub">ESP32 GPIO mapping</div>
<div class="table-wrap mt"><table><thead><tr><th>Output</th><th>GPIO</th><th>Behaviour</th></tr></thead><tbody>
<tr><td>Unified</td><td class="mono">25</td><td>Any enabled sensor threshold</td></tr>
<tr><td>Voltage</td><td class="mono">25</td><td>Voltage thresholds, shared with unified</td></tr>
<tr><td>Current</td><td class="mono">26</td><td>Current thresholds</td></tr>
<tr><td>Power</td><td class="mono">27</td><td>Power thresholds</td></tr>
</tbody></table></div>
<div class="muted mt" style="font-size:12px">GPIO25 is physically shared by the unified and voltage functions. The pin energises when either demand is present. Relays start in the de-energised state after every boot.</div>
</div></div>
<h2 class="mt">Manual Override</h2><div class="sub">Commands are temporary and return to automatic control</div>
<div class="grid cards mt">
MANUAL_CARDS
</div>
)HTML");
  const char *polarityChecked = g_config.relayPolarity ? "checked" : "";
  const char *latchChecked = g_config.relayLatch ? "checked" : "";
  replaceToken(g_pageBuffer, "POLARITY_CHECKED", polarityChecked);
  replaceToken(g_pageBuffer, "LATCH_CHECKED", latchChecked);
  char numberBuffer[32];
  snprintf(numberBuffer, sizeof(numberBuffer), "%llu", (unsigned long long)g_config.relayMinOnMs);
  replaceToken(g_pageBuffer, "MIN_ON", numberBuffer);
  snprintf(numberBuffer, sizeof(numberBuffer), "%llu", (unsigned long long)g_config.relayHoldOffMs);
  replaceToken(g_pageBuffer, "HOLD_OFF", numberBuffer);
  snprintf(numberBuffer, sizeof(numberBuffer), "%llu", (unsigned long long)g_config.sampleIntervalMs);
  replaceToken(g_pageBuffer, "SAMPLE_MS", numberBuffer);
  size_t manualUsed = 0;
  const char *names[4] = {"Unified", "Voltage", "Current", "Power"};
  const char *pins[4] = {"25", "25", "26", "27"};
  for (uint64_t index = 0; index < 4; index++) {
    appendFmt(g_formBuffer, sizeof(g_formBuffer), manualUsed,
      "<div class=\"card\"><div class=\"row\"><h3>%s relay</h3><span class=\"badge off\">GPIO %s</span></div><div class=\"muted mt\" style=\"font-size:11px\">Mode %s</div>"
      "<div class=\"btn-row\"><button class=\"btn small\" data-relay=\"%llu\" data-mode=\"0\">Auto</button><button class=\"btn small\" data-relay=\"%llu\" data-mode=\"1\">Force ON</button><button class=\"btn small\" data-relay=\"%llu\" data-mode=\"2\">Force OFF</button><button class=\"btn small\" data-relay=\"%llu\" data-mode=\"3\" data-duration=\"30\">30 s ON</button></div></div>",
      names[index], pins[index], g_manualRelay[index].mode == 0 ? "automatic" : (g_manualRelay[index].mode == 1 ? "forced on" : (g_manualRelay[index].mode == 2 ? "forced off" : "timed on")),
      (unsigned long long)index, (unsigned long long)index, (unsigned long long)index, (unsigned long long)index);
  }
  g_formBuffer[manualUsed < sizeof(g_formBuffer) ? manualUsed : sizeof(g_formBuffer) - 1] = '\0';
  replaceToken(g_pageBuffer, "MANUAL_CARDS", g_formBuffer);
  used = strlen(g_pageBuffer);
}

// ========================================================
//
// WIFI ACCESS POINT SETTINGS PAGE
//
// ========================================================

void buildWifiPage(size_t &used) {
  appendFmt(g_pageBuffer, sizeof(g_pageBuffer), used, R"HTML(
<div class="grid two">
<form class="card" data-api="/api/config/wifi"><h2>Access Point</h2><div class="sub">Credentials and broadcast options</div>
<div class="fields mt">
<div class="field"><label>Network name (SSID)</label><input name="ssid" maxlength="31" value="WIFI_SSID" required><span class="hint">Default: ESP32_INA219</span></div>
<div class="field"><label>Password</label><input name="password" id="wifiPass" type="password" maxlength="63" value="WIFI_PASS"><span class="hint">8 to 63 characters, or empty for an open AP</span></div>
</div><div class="btn-row"><button class="btn" type="button" data-toggle-pass="#wifiPass">Show password</button></div>
<div class="switch mt"><span><b>Hide network name</b></span><input type="checkbox" name="hidden" value="1" HIDDEN_CHECKED></div>
<div class="switch"><span><b>Captive portal</b><br><span class="muted">Redirect every unknown request to 192.168.4.1</span></span><input type="checkbox" name="portal" value="1" PORTAL_CHECKED></div>
<div class="fields mt">
<div class="field"><label>Wi-Fi channel</label><select name="channel">CHANNEL_OPTIONS</select></div>
<div class="field"><label>Maximum clients</label><select name="maxclients"><option value="4">4</option><option value="8">8</option><option value="10">10</option></select></div>
</div><div class="btn-row"><button class="btn primary" type="submit">Apply and restart AP</button></div>
<div class="muted mt" style="font-size:11px">Applying changes briefly restarts the access point. Your browser may need to reconnect.</div></form>
<div class="card"><h2>Network Status</h2><div class="sub">Current access point</div>
<div class="table-wrap mt"><table><tbody>
<tr><th>SSID</th><td class="mono" id="ssidNow">-</td></tr>
<tr><th>Address</th><td class="mono" id="apIp">192.168.4.1</td></tr>
<tr><th>Clients</th><td class="mono" id="clientCount">-</td></tr>
<tr><th>Captive portal</th><td id="portalNow">-</td></tr>
</tbody></table></div>
<div class="btn-row mt"><button class="btn" data-action="reboot" data-confirm="Reboot the controller now?">Reboot device</button><button class="btn danger" data-action="factory_reset" data-confirm="Erase all settings and restore factory defaults?">Factory reset</button></div>
</div></div>
)HTML");
  replaceToken(g_pageBuffer, "WIFI_SSID", g_config.wifiSsid);
  replaceToken(g_pageBuffer, "WIFI_PASS", g_config.wifiPassword);
  replaceToken(g_pageBuffer, "HIDDEN_CHECKED", g_config.wifiHidden ? "checked" : "");
  replaceToken(g_pageBuffer, "PORTAL_CHECKED", g_config.portalEnabled ? "checked" : "");
  char channelBuffer[512];
  size_t channelUsed = 0;
  for (uint64_t channel = 1; channel <= 13; channel++) {
    appendFmt(channelBuffer, sizeof(channelBuffer), channelUsed, "<option value=\"%llu\"%s>%llu</option>", (unsigned long long)channel, g_config.wifiChannel == channel ? " selected" : "", (unsigned long long)channel);
  }
  replaceToken(g_pageBuffer, "CHANNEL_OPTIONS", channelBuffer);
  size_t maxClientUsed = strlen(g_pageBuffer);
  const char *maxSelected[3] = {"", "", ""};
  if (g_config.wifiMaxClients == 4) maxSelected[0] = " selected";
  else if (g_config.wifiMaxClients == 10) maxSelected[2] = " selected";
  else maxSelected[1] = " selected";
  const char *maxPatterns[3] = {"<option value=\"4\">4</option>", "<option value=\"8\">8</option>", "<option value=\"10\">10</option>"};
  char maxReplacement[64];
  for (uint64_t index = 0; index < 3; index++) {
    snprintf(maxReplacement, sizeof(maxReplacement), "<option value=\"%llu\"%s>%llu</option>", (unsigned long long)(index == 0 ? 4 : (index == 1 ? 8 : 10)), maxSelected[index], (unsigned long long)(index == 0 ? 4 : (index == 1 ? 8 : 10)));
    replaceToken(g_pageBuffer, maxPatterns[index], maxReplacement);
  }
  (void)maxClientUsed;
  used = strlen(g_pageBuffer);
}

// ========================================================
//
// SYSTEM DIAGNOSTICS PAGE
//
// ========================================================

void buildSystemPage(size_t &used) {
  appendFmt(g_pageBuffer, sizeof(g_pageBuffer), used, R"HTML(
<div class="grid two">
<div class="card"><h2>Controller Health</h2><div class="sub">Live runtime counters</div><div class="table-wrap mt"><table><tbody>
<tr><th>Uptime</th><td class="mono" id="sysUptime">-</td></tr>
<tr><th>Free heap</th><td class="mono" id="sysHeap">-</td></tr>
<tr><th>Minimum heap</th><td class="mono" id="sysHeapMin">-</td></tr>
<tr><th>Sensor samples</th><td class="mono" id="sysSamples">-</td></tr>
<tr><th>Read failures</th><td class="mono" id="sysFailures">-</td></tr>
<tr><th>Config writes</th><td class="mono" id="sysWrites">-</td></tr>
<tr><th>Deferred writes</th><td class="mono" id="sysSkips">-</td></tr>
<tr><th>Config CRC64</th><td class="mono" id="sysCrc">-</td></tr>
</tbody></table></div></div>
<div class="card"><h2>Reliability Design</h2><div class="sub">Continuous operation features</div>
<div class="muted mt" style="font-size:12px">
Dual alternating configuration slots with 64-bit CRC and sequence numbers.<br>
Writes are debounced, rate limited and skipped during boot.<br>
Every numeric field is validated and clamped before use.<br>
A 64-bit software clock removes millisecond rollover at 49.7 days.<br>
Fixed buffers and no dynamic allocation in the sampling path.<br>
No blocking delays in the main scheduling loop.
</div><div class="progress" id="resetWrap" hidden><i id="resetFill"></i></div></div></div>
<div class="card mt"><h2>Maintenance</h2><div class="sub">Destructive actions require confirmation</div>
<div class="btn-row"><button class="btn" data-action="reboot" data-confirm="Reboot the controller now?">Reboot</button><button class="btn danger" data-action="factory_reset" data-confirm="Erase configuration and restore all factory defaults?">Factory reset</button></div>
<div class="muted mt" style="font-size:12px">Physical reset: hold GPIO0 BOOT for 5 seconds. The status LED accelerates while the hold is detected.</div>
</div>
)HTML");
}

// ========================================================
//
// REQUEST PARAMETER HELPERS
//
// ========================================================

bool hasRequestParameter(AsyncWebServerRequest *request, const char *name) {
  return request->hasParam(name, true) || request->hasParam(name, false);
}

String requestParameter(AsyncWebServerRequest *request, const char *name) {
  if (request->hasParam(name, true)) return request->getParam(name, true)->value();
  if (request->hasParam(name, false)) return request->getParam(name, false)->value();
  return String();
}

uint64_t paramU64(AsyncWebServerRequest *request, const char *name, uint64_t fallback, uint64_t minimum, uint64_t maximum) {
  if (!hasRequestParameter(request, name)) return fallback;
  return parseU64(requestParameter(request, name).c_str(), minimum, maximum, fallback);
}

int64_t paramI64(AsyncWebServerRequest *request, const char *name, int64_t fallback, int64_t minimum, int64_t maximum) {
  if (!hasRequestParameter(request, name)) return fallback;
  return parseI64(requestParameter(request, name).c_str(), minimum, maximum, fallback);
}

void paramText(AsyncWebServerRequest *request, const char *name, char *destination, size_t capacity) {
  if (!hasRequestParameter(request, name)) return;
  sanitizeText(requestParameter(request, name).c_str(), destination, capacity);
}

void sendJson(AsyncWebServerRequest *request, int code) {
  request->send(code, "application/json; charset=utf-8", g_jsonBuffer);
}

void sendJsonOk(AsyncWebServerRequest *request, const char *message) {
  size_t used = 0;
  appendFmt(g_jsonBuffer, sizeof(g_jsonBuffer), used, "{\"ok\":true,\"message\":\"%s\"}", message);
  sendJson(request, 200);
}

void sendJsonError(AsyncWebServerRequest *request, int code, const char *message) {
  size_t used = 0;
  appendFmt(g_jsonBuffer, sizeof(g_jsonBuffer), used, "{\"ok\":false,\"error\":\"%s\"}", message);
  sendJson(request, code);
}

// ========================================================
//
// STATUS AND CONFIGURATION API
//
// ========================================================

void durationText(uint64_t milliseconds, char *output, size_t capacity) {
  uint64_t seconds = milliseconds / 1000ULL;
  uint64_t days = seconds / 86400ULL;
  uint64_t hours = (seconds % 86400ULL) / 3600ULL;
  uint64_t minutes = (seconds % 3600ULL) / 60ULL;
  uint64_t remainder = seconds % 60ULL;
  snprintf(output, capacity, "%llud %02llu:%02llu:%02llu", (unsigned long long)days, (unsigned long long)hours, (unsigned long long)minutes, (unsigned long long)remainder);
}

const char *relayLabel(uint64_t index) {
  if (index == 0) return "Unified · GPIO25";
  if (index == 1) return "Voltage · GPIO25";
  if (index == 2) return "Current · GPIO26";
  return "Power · GPIO27";
}

void handleStatusApi(AsyncWebServerRequest *request) {
  size_t used = 0;
  char uptimeText[40];
  char escapedName[64];
  char escapedSsid[96];
  char valueBuffer[32];
  durationText(nowMs(), uptimeText, sizeof(uptimeText));
  jsonEscape(g_config.wifiSsid, escapedSsid, sizeof(escapedSsid));
  uint64_t alarmCount = 0;
  for (uint64_t index = 0; index < INA219_MAX_SENSORS; index++) {
    if (g_sensorState[index].statusCode == 3) alarmCount++;
  }
  appendFmt(g_jsonBuffer, sizeof(g_jsonBuffer), used,
    "{\"ok\":true,\"uptimeMs\":%llu,\"uptimeText\":\"%s\",\"heapFree\":%u,\"heapMin\":%u,\"samples\":%llu,\"failures\":%llu,\"alarmCount\":%llu,"
    "\"config\":{\"valid\":%llu,\"sequence\":%llu,\"crc\":\"%016llX\"},"
    "\"storage\":{\"writes\":%llu,\"skips\":%llu,\"failures\":%llu},"
    "\"reset\":{\"progress\":%llu,\"armed\":%llu},"
    "\"wifi\":{\"ssid\":\"%s\",\"clients\":%u,\"ip\":\"192.168.4.1\",\"portal\":%llu,\"channel\":%llu},"
    "\"relays\":{\"lines\":[",
    (unsigned long long)nowMs(), uptimeText, (unsigned int)ESP.getFreeHeap(), (unsigned int)ESP.getMinFreeHeap(),
    (unsigned long long)g_totalSamples, (unsigned long long)g_totalReadFailures, (unsigned long long)alarmCount,
    (unsigned long long)g_configValid, (unsigned long long)g_configSequence, (unsigned long long)g_configCrc,
    (unsigned long long)g_nvsWrites, (unsigned long long)g_nvsSkips, (unsigned long long)g_nvsFailures,
    (unsigned long long)g_buttonProgress, (unsigned long long)g_resetArmed, escapedSsid,
    (unsigned int)WiFi.softAPgetStationNum(), (unsigned long long)g_config.portalEnabled, (unsigned long long)g_config.wifiChannel);
  for (uint64_t index = 0; index < 4; index++) {
    uint64_t pinState = index < 3 ? g_relayState[index] : ((g_relayDemand & 1ULL) ? 1ULL : 0ULL);
    appendFmt(g_jsonBuffer, sizeof(g_jsonBuffer), used, "{\"label\":\"%s\",\"state\":%llu,\"mode\":%llu,\"pin\":%llu}%s",
      relayLabel(index), (unsigned long long)pinState, (unsigned long long)g_manualRelay[index].mode,
      (unsigned long long)relayPin(index), index == 3 ? "" : ",");
  }
  appendFmt(g_jsonBuffer, sizeof(g_jsonBuffer), used, "],\"demand\":%llu},\"sensors\":[", (unsigned long long)g_relayDemand);
  for (uint64_t index = 0; index < INA219_MAX_SENSORS; index++) {
    SensorState &state = g_sensorState[index];
    SensorConfig &sensor = g_config.sensor[index];
    jsonEscape(sensor.name, escapedName, sizeof(escapedName));
    fixedString(state.reading.busMilliVolt, 3, valueBuffer, sizeof(valueBuffer));
    char voltageText[32];
    snprintf(voltageText, sizeof(voltageText), "%s", valueBuffer);
    fixedString(state.reading.currentMicroAmp / 1000LL, 3, valueBuffer, sizeof(valueBuffer));
    char currentText[32];
    snprintf(currentText, sizeof(currentText), "%s", valueBuffer);
    fixedString(state.reading.powerMicroWatt / 1000LL, 3, valueBuffer, sizeof(valueBuffer));
    char powerText[32];
    snprintf(powerText, sizeof(powerText), "%s", valueBuffer);
    fixedString(state.reading.shuntMicroVolt, 3, valueBuffer, sizeof(valueBuffer));
    char shuntText[32];
    snprintf(shuntText, sizeof(shuntText), "%s", valueBuffer);
    snprintf(valueBuffer, sizeof(valueBuffer), "0x%02llX", (unsigned long long)sensor.address);
    appendFmt(g_jsonBuffer, sizeof(g_jsonBuffer), used,
      "{\"idx\":%llu,\"name\":\"%s\",\"addr\":\"%s\",\"enabled\":%llu,\"online\":%llu,\"status\":%llu,"
      "\"volt\":%lld,\"voltText\":\"%s\",\"curMa\":%lld,\"curText\":\"%s\",\"powerMw\":%lld,\"powerText\":\"%s\",\"shuntMv\":\"%s\","
      "\"ageMs\":%llu,\"samples\":%llu,\"vLatch\":%llu,\"iLatch\":%llu,\"pLatch\":%llu}%s",
      (unsigned long long)index, escapedName, valueBuffer, (unsigned long long)sensor.enabled, (unsigned long long)state.online, (unsigned long long)state.statusCode,
      (long long)state.reading.busMilliVolt, voltageText,
      (long long)(state.reading.currentMicroAmp / 1000LL), currentText,
      (long long)(state.reading.powerMicroWatt / 1000LL), powerText, shuntText,
      (unsigned long long)(state.online ? nowMs() - state.lastOkMs : 0ULL),
      (unsigned long long)state.sampleCount,
      (unsigned long long)state.voltageAlarm.latched, (unsigned long long)state.currentAlarm.latched, (unsigned long long)state.powerAlarm.latched,
      index == INA219_MAX_SENSORS - 1 ? "" : ",");
  }
  appendFmt(g_jsonBuffer, sizeof(g_jsonBuffer), used, "]}");
  sendJson(request, 200);
}

void handleConfigApi(AsyncWebServerRequest *request) {
  size_t used = 0;
  char escapedSsid[96];
  jsonEscape(g_config.wifiSsid, escapedSsid, sizeof(escapedSsid));
  appendFmt(g_jsonBuffer, sizeof(g_jsonBuffer), used,
    "{\"ok\":true,\"config\":{\"ssid\":\"%s\",\"channel\":%llu,\"hidden\":%llu,\"portal\":%llu,\"maxclients\":%llu,"
    "\"sample\":%llu,\"polarity\":%llu,\"latch\":%llu,\"minon\":%llu,\"holdoff\":%llu},\"sensors\":[",
    escapedSsid, (unsigned long long)g_config.wifiChannel, (unsigned long long)g_config.wifiHidden,
    (unsigned long long)g_config.portalEnabled, (unsigned long long)g_config.wifiMaxClients,
    (unsigned long long)g_config.sampleIntervalMs, (unsigned long long)g_config.relayPolarity,
    (unsigned long long)g_config.relayLatch, (unsigned long long)g_config.relayMinOnMs, (unsigned long long)g_config.relayHoldOffMs);
  for (uint64_t index = 0; index < INA219_MAX_SENSORS; index++) {
    SensorConfig &sensor = g_config.sensor[index];
    char escapedName[64];
    jsonEscape(sensor.name, escapedName, sizeof(escapedName));
    appendFmt(g_jsonBuffer, sizeof(g_jsonBuffer), used,
      "{\"idx\":%llu,\"name\":\"%s\",\"address\":\"0x%02llX\",\"enabled\":%llu,\"vmin\":%lld,\"vmax\":%lld,\"imin\":%lld,\"imax\":%lld,\"pmin\":%lld,\"pmax\":%lld}%s",
      (unsigned long long)index, escapedName, (unsigned long long)sensor.address, (unsigned long long)sensor.enabled,
      (long long)sensor.voltageMinMilliVolt, (long long)sensor.voltageMaxMilliVolt,
      (long long)sensor.currentMinMilliAmp, (long long)sensor.currentMaxMilliAmp,
      (long long)sensor.powerMinMilliWatt, (long long)sensor.powerMaxMilliWatt,
      index == INA219_MAX_SENSORS - 1 ? "" : ",");
  }
  appendFmt(g_jsonBuffer, sizeof(g_jsonBuffer), used, "]}");
  sendJson(request, 200);
}

// ========================================================
//
// SETTINGS WRITE HANDLERS
//
// ========================================================

void handleSensorSettingsPost(AsyncWebServerRequest *request) {
  Config previous;
  memcpy(&previous, &g_config, sizeof(Config));
  char parameterName[32];
  for (uint64_t index = 0; index < INA219_MAX_SENSORS; index++) {
    SensorConfig &sensor = g_config.sensor[index];
    snprintf(parameterName, sizeof(parameterName), "s%llu_en", (unsigned long long)index);
    sensor.enabled = hasRequestParameter(request, parameterName) ? 1ULL : 0ULL;
    snprintf(parameterName, sizeof(parameterName), "s%llu_name", (unsigned long long)index);
    paramText(request, parameterName, sensor.name, sizeof(sensor.name));
    snprintf(parameterName, sizeof(parameterName), "s%llu_addr", (unsigned long long)index);
    sensor.address = paramU64(request, parameterName, sensor.address, 0x40, 0x4F);
    snprintf(parameterName, sizeof(parameterName), "s%llu_cal", (unsigned long long)index);
    sensor.calibration = paramU64(request, parameterName, sensor.calibration, 0, 2);
    snprintf(parameterName, sizeof(parameterName), "s%llu_shunt", (unsigned long long)index);
    sensor.shuntMilliOhm = paramU64(request, parameterName, sensor.shuntMilliOhm, 1, 100000);
    snprintf(parameterName, sizeof(parameterName), "s%llu_ven", (unsigned long long)index);
    sensor.voltageEnable = hasRequestParameter(request, parameterName) ? 1ULL : 0ULL;
    snprintf(parameterName, sizeof(parameterName), "s%llu_ien", (unsigned long long)index);
    sensor.currentEnable = hasRequestParameter(request, parameterName) ? 1ULL : 0ULL;
    snprintf(parameterName, sizeof(parameterName), "s%llu_pen", (unsigned long long)index);
    sensor.powerEnable = hasRequestParameter(request, parameterName) ? 1ULL : 0ULL;
    snprintf(parameterName, sizeof(parameterName), "s%llu_vmin", (unsigned long long)index);
    sensor.voltageMinMilliVolt = paramI64(request, parameterName, sensor.voltageMinMilliVolt, 0, 32000);
    snprintf(parameterName, sizeof(parameterName), "s%llu_vmax", (unsigned long long)index);
    sensor.voltageMaxMilliVolt = paramI64(request, parameterName, sensor.voltageMaxMilliVolt, 0, 32000);
    snprintf(parameterName, sizeof(parameterName), "s%llu_imin", (unsigned long long)index);
    sensor.currentMinMilliAmp = paramI64(request, parameterName, sensor.currentMinMilliAmp, -300000, 300000);
    snprintf(parameterName, sizeof(parameterName), "s%llu_imax", (unsigned long long)index);
    sensor.currentMaxMilliAmp = paramI64(request, parameterName, sensor.currentMaxMilliAmp, -300000, 300000);
    snprintf(parameterName, sizeof(parameterName), "s%llu_pmin", (unsigned long long)index);
    sensor.powerMinMilliWatt = paramI64(request, parameterName, sensor.powerMinMilliWatt, -1000000, 1000000);
    snprintf(parameterName, sizeof(parameterName), "s%llu_pmax", (unsigned long long)index);
    sensor.powerMaxMilliWatt = paramI64(request, parameterName, sensor.powerMaxMilliWatt, -1000000, 1000000);
    snprintf(parameterName, sizeof(parameterName), "s%llu_hyst", (unsigned long long)index);
    sensor.hysteresisPercent = paramU64(request, parameterName, sensor.hysteresisPercent, 0, 25);
    snprintf(parameterName, sizeof(parameterName), "s%llu_deb", (unsigned long long)index);
    sensor.debounceSamples = paramU64(request, parameterName, sensor.debounceSamples, 1, 20);
    uint64_t mask = 0;
    snprintf(parameterName, sizeof(parameterName), "s%llu_r1", (unsigned long long)index);
    if (hasRequestParameter(request, parameterName)) mask |= 1ULL;
    snprintf(parameterName, sizeof(parameterName), "s%llu_r2", (unsigned long long)index);
    if (hasRequestParameter(request, parameterName)) mask |= 2ULL;
    snprintf(parameterName, sizeof(parameterName), "s%llu_r3", (unsigned long long)index);
    if (hasRequestParameter(request, parameterName)) mask |= 4ULL;
    sensor.relayMask = mask;
  }
  validateConfig(g_config);
  for (uint64_t index = 0; index < INA219_MAX_SENSORS; index++) {
    if (previous.sensor[index].address != g_config.sensor[index].address ||
        previous.sensor[index].calibration != g_config.sensor[index].calibration ||
        previous.sensor[index].shuntMilliOhm != g_config.sensor[index].shuntMilliOhm ||
        previous.sensor[index].enabled != g_config.sensor[index].enabled) {
      forceSensorReinit(index);
    }
  }
  if (memcmp(&previous, &g_config, sizeof(Config)) != 0) markConfigDirty();
  sendJsonOk(request, "Sensor settings saved");
}

void handleRelaySettingsPost(AsyncWebServerRequest *request) {
  Config previous;
  memcpy(&previous, &g_config, sizeof(Config));
  g_config.relayPolarity = hasRequestParameter(request, "polarity") ? 1ULL : 0ULL;
  g_config.relayLatch = hasRequestParameter(request, "latch") ? 1ULL : 0ULL;
  g_config.relayMinOnMs = paramU64(request, "minon", g_config.relayMinOnMs, 0, 600000);
  g_config.relayHoldOffMs = paramU64(request, "holdoff", g_config.relayHoldOffMs, 0, 600000);
  g_config.sampleIntervalMs = paramU64(request, "sample", g_config.sampleIntervalMs, 100, 60000);
  validateConfig(g_config);
  if (previous.relayPolarity != g_config.relayPolarity) initializeRelays();
  if (memcmp(&previous, &g_config, sizeof(Config)) != 0) markConfigDirty();
  sendJsonOk(request, "Relay settings saved");
}

void handleWifiSettingsPost(AsyncWebServerRequest *request) {
  Config previous;
  memcpy(&previous, &g_config, sizeof(Config));
  paramText(request, "ssid", g_config.wifiSsid, sizeof(g_config.wifiSsid));
  paramText(request, "password", g_config.wifiPassword, sizeof(g_config.wifiPassword));
  g_config.wifiChannel = paramU64(request, "channel", g_config.wifiChannel, 1, 13);
  g_config.wifiMaxClients = paramU64(request, "maxclients", g_config.wifiMaxClients, 1, 10);
  g_config.wifiHidden = hasRequestParameter(request, "hidden") ? 1ULL : 0ULL;
  g_config.portalEnabled = hasRequestParameter(request, "portal") ? 1ULL : 0ULL;
  validateConfig(g_config);
  if (memcmp(&previous, &g_config, sizeof(Config)) != 0) markConfigDirty();
  persistConfigNow(true);
  scheduleWifiRestart(1500ULL);
  sendJsonOk(request, "Access point settings saved and restarting");
}

void handleActionPost(AsyncWebServerRequest *request) {
  String action = requestParameter(request, "action");
  if (action == "ack") {
    acknowledgeAlarms(paramU64(request, "index", INA219_MAX_SENSORS, 0, INA219_MAX_SENSORS));
    sendJsonOk(request, "Alarms acknowledged");
    return;
  }
  if (action == "relay") {
    uint64_t index = paramU64(request, "index", 0, 0, 3);
    uint64_t mode = paramU64(request, "mode", 0, 0, 3);
    uint64_t seconds = paramU64(request, "duration", 30, 1, 3600);
    setManualRelay(index, mode, seconds);
    sendJsonOk(request, "Relay command applied");
    return;
  }
  if (action == "reboot") {
    sendJsonOk(request, "Reboot scheduled");
    scheduleReboot(900ULL);
    return;
  }
  if (action == "factory_reset") {
    if (requestParameter(request, "confirm") != "yes") {
      sendJsonError(request, 400, "Confirmation required");
      return;
    }
    factoryReset();
    persistConfigNow(true);
    sendJsonOk(request, "Factory defaults restored");
    scheduleReboot(1200ULL);
    return;
  }
  if (action == "refresh_page") {
    sendJsonOk(request, "Refresh requested");
    return;
  }
  sendJsonError(request, 400, "Unknown action");
}

// ========================================================
//
// WEB ROUTES AND CAPTIVE PORTAL
//
// ========================================================

void setupWebRoutes(void) {
  g_webServer.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    g_httpRequests++;
    sendPage(request, "dashboard", "Live Dashboard", "Real-time voltage, current and power from every enabled INA219 sensor", "dashboard", buildDashboardPage, "");
  });
  g_webServer.on("/sensors", HTTP_GET, [](AsyncWebServerRequest *request) {
    g_httpRequests++;
    sendPage(request, "sensors", "Sensor Settings", "Addresses, calibration, thresholds, hysteresis and relay routing", "sensors", buildSensorSettingsPage, "");
  });
  g_webServer.on("/relays", HTTP_GET, [](AsyncWebServerRequest *request) {
    g_httpRequests++;
    sendPage(request, "relays", "Relay Control", "Threshold routing, safety timing and temporary manual override", "relays", buildRelayPage, "");
  });
  g_webServer.on("/wifi", HTTP_GET, [](AsyncWebServerRequest *request) {
    g_httpRequests++;
    sendPage(request, "wifi", "Wi-Fi Access Point", "Default network ESP32_INA219 with captive portal at 192.168.4.1", "wifi", buildWifiPage, "");
  });
  g_webServer.on("/system", HTTP_GET, [](AsyncWebServerRequest *request) {
    g_httpRequests++;
    sendPage(request, "system", "System & Diagnostics", "Runtime counters, persistence health and maintenance actions", "system", buildSystemPage, "");
  });
  g_webServer.on("/api/status", HTTP_GET, [](AsyncWebServerRequest *request) {
    g_httpRequests++;
    handleStatusApi(request);
  });
  g_webServer.on("/api/config", HTTP_GET, [](AsyncWebServerRequest *request) {
    g_httpRequests++;
    handleConfigApi(request);
  });
  g_webServer.on("/api/config/sensors", HTTP_POST, [](AsyncWebServerRequest *request) {
    g_httpRequests++;
    handleSensorSettingsPost(request);
  });
  g_webServer.on("/api/config/relays", HTTP_POST, [](AsyncWebServerRequest *request) {
    g_httpRequests++;
    handleRelaySettingsPost(request);
  });
  g_webServer.on("/api/config/wifi", HTTP_POST, [](AsyncWebServerRequest *request) {
    g_httpRequests++;
    handleWifiSettingsPost(request);
  });
  g_webServer.on("/api/action", HTTP_POST, [](AsyncWebServerRequest *request) {
    g_httpRequests++;
    handleActionPost(request);
  });
  g_webServer.on("/favicon.ico", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(204);
  });
  const char *probePaths[] = {"/generate_204", "/redirect", "/hotspot-detect.html", "/connecttest.txt", "/ncsi.txt", "/fwlink", "/success.html", "/portal", "/wifidog"};
  for (uint64_t index = 0; index < (sizeof(probePaths) / sizeof(probePaths[0])); index++) {
    g_webServer.on(probePaths[index], HTTP_GET, [](AsyncWebServerRequest *request) {
      g_httpRequests++;
      request->redirect("http://192.168.4.1/");
    });
  }
  g_webServer.onNotFound([](AsyncWebServerRequest *request) {
    g_httpRequests++;
    g_httpRejected++;
    String url = request->url();
    if (url.startsWith("/api/")) {
      sendJsonError(request, 404, "Not found");
      return;
    }
    if (g_config.portalEnabled) request->redirect("http://192.168.4.1/");
    else request->send(404, "text/plain", "Not found");
  });
  g_webServer.begin();
}

void startAccessPoint(void) {
  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(AP_IP_ADDRESS, AP_GATEWAY, AP_SUBNET);
  WiFi.softAP(g_config.wifiSsid, g_config.wifiPassword, (int)g_config.wifiChannel, g_config.wifiHidden ? true : false, (int)g_config.wifiMaxClients);
  if (g_config.portalEnabled) {
    g_dnsServer.setErrorReplyCode(DNSReplyCode::NoError);
    g_dnsServer.setTTL(120);
    g_dnsServer.start(53, "*", AP_IP_ADDRESS);
    g_dnsRunning = 1;
  }
}

// ========================================================
//
// SETUP
//
// ========================================================

void setup(void) {
  g_previousMillis = (uint64_t)millis();
  advanceClock();
  g_bootMs = nowMs();
  loadConfig();
  validateConfig(g_config);
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  Wire.setClock(400000);
  Wire.setTimeout(5);
  pinMode(PIN_BOOT_BUTTON, INPUT_PULLUP);
  pinMode(PIN_STATUS_LED, OUTPUT);
  digitalWrite(PIN_STATUS_LED, LOW);
  initializeRelays();
  startAccessPoint();
  setupWebRoutes();
  for (uint64_t index = 0; index < INA219_MAX_SENSORS; index++) {
    g_sensorState[index].nextProbeMs = nowMs() + index * 40ULL;
    g_sensorState[index].nextReadMs = nowMs() + index * 40ULL;
  }
}

// ========================================================
//
// NON-BLOCKING MAIN SCHEDULER
//
// ========================================================

void loop(void) {
  advanceClock();
  uint64_t currentTime = nowMs();
  if (g_dnsRunning) g_dnsServer.processNextRequest();
  serviceFactoryButton(currentTime);
  serviceSensors(currentTime);
  updateRelays(currentTime);
  serviceStatusLed(currentTime);
  serviceScheduledActions(currentTime);
  if (currentTime - g_lastPersistCheck >= 1000ULL) {
    g_lastPersistCheck = currentTime;
    if (g_configDirty && currentTime - g_configDirtyAt >= NVS_WRITE_DEBOUNCE_MS) persistConfigNow(false);
  }
}
