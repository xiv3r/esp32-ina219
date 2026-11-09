/*
 * ESP32 INA219 Relay + WiFi AP Dashboard
 */

#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <Preferences.h>
#include <Wire.h>
#include <Adafruit_INA219.h>
#include <esp_system.h>
#include <esp_timer.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

// ---------------------------------------------------------------------------
#define RELAY_PIN        25
#define BOOT_BUTTON_PIN   0
#define I2C_SDA          21
#define I2C_SCL          22

static const char* AP_SSID = "ESP32_INA219";
static const char* AP_PASS = "12345678";

static const uint32_t SAMPLE_INTERVAL_MS    = 1000;
static const uint32_t BUTTON_POLL_MS        = 20;
static const uint32_t RESET_HOLD_MS         = 5000;
static const uint32_t NVS_FLUSH_INTERVAL_MS = 60000;
static const uint32_t WIFI_RECHECK_MS       = 5000;

// ---------------------------------------------------------------------------
WebServer   server(80);
DNSServer   dnsServer;
Preferences prefs;
Adafruit_INA219 ina219(0x40);

volatile float g_voltage = 0.0f;
volatile float g_current = 0.0f;
volatile float g_power   = 0.0f;

#pragma pack(push, 1)
struct Config {
  float vThresh;
  float iThresh;
  float pThresh;
  bool  relayActiveHigh;
  bool  relayEnabled;
  bool  useVoltage;
  bool  useCurrent;
  bool  usePower;
};
#pragma pack(pop)

static Config cfg = {
  0.0f, 0.0f, 0.0f,
  true, false, false, false, false
};

static bool     cfgDirty        = false;
static uint32_t cfgLastFlushMs  = 0;
static bool     g_relayOn       = false;

static uint32_t tLastSample     = 0;
static uint32_t tLastButtonPoll = 0;
static uint32_t tLastWifiCheck  = 0;

// Button state
static uint8_t  btnLowCount     = 0;
static uint32_t btnPressStartMs = 0;
static bool     btnHeld         = false;
static bool     resetTriggered  = false;

// Forward declarations
static void doFactoryReset();
static void startAP();
static void startCaptivePortal();
static void flushConfigIfDue(uint32_t now, bool force);

// ---------------------------------------------------------------------------
// PROGMEM pages
// ---------------------------------------------------------------------------
static const char INDEX_HTML[] PROGMEM = R"HTML(<!DOCTYPE html>
<html lang="en"><head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,maximum-scale=1,user-scalable=no">
<meta name="theme-color" content="#0b1220"><title>ESP32 INA219</title>
<style>
*{box-sizing:border-box;-webkit-tap-highlight-color:transparent}
html,body{margin:0;padding:0;font-family:-apple-system,BlinkMacSystemFont,"Segoe UI",Roboto,sans-serif;background:#0b1220;color:#e6eefc}
.wrap{max-width:820px;margin:0 auto;padding:14px}
h1{font-size:18px;margin:0 0 6px;letter-spacing:.4px}
.sub{font-size:12px;color:#8aa0c0;margin-bottom:14px}
.tabs{display:flex;gap:6px;margin-bottom:14px;flex-wrap:wrap}
.tab{flex:1;min-width:110px;padding:10px 8px;background:#14213a;border:1px solid #1f3358;border-radius:10px;text-align:center;font-size:13px;cursor:pointer;user-select:none;transition:background .15s}
.tab.active{background:#1f6feb;border-color:#1f6feb;color:#fff}
.card{background:#0f1a2e;border:1px solid #1f3358;border-radius:14px;padding:14px;margin-bottom:14px}
.card h2{font-size:14px;margin:0 0 10px;color:#9ec1ff;font-weight:600;letter-spacing:.5px;text-transform:uppercase}
.row{display:flex;justify-content:space-between;align-items:center;padding:8px 0;border-bottom:1px dashed #1a2b48;font-size:14px}
.row:last-child{border-bottom:0}
.val{font-variant-numeric:tabular-nums;color:#63e6a8;font-weight:600}
label{display:block;font-size:12px;color:#8aa0c0;margin:8px 0 4px}
input[type=number]{width:100%;padding:10px;background:#0b1220;border:1px solid #24406d;border-radius:8px;color:#e6eefc;font-size:16px;outline:none}
input:focus{border-color:#1f6feb}
.btn{display:inline-block;padding:11px 16px;background:#1f6feb;color:#fff;border:0;border-radius:10px;font-size:14px;cursor:pointer;min-height:42px}
.btn.secondary{background:#24406d}
.btn.danger{background:#e5484d}
.btn:active{opacity:.85}
.toggle{position:relative;display:inline-block;width:48px;height:28px;flex:none}
.toggle input{opacity:0;width:0;height:0}
.slider{position:absolute;inset:0;background:#24406d;border-radius:28px;transition:.2s;cursor:pointer}
.slider:before{content:"";position:absolute;height:22px;width:22px;left:3px;top:3px;background:#fff;border-radius:50%;transition:.2s}
.toggle input:checked + .slider{background:#2ea043}
.toggle input:checked + .slider:before{transform:translateX(20px)}
.grid3{display:grid;grid-template-columns:1fr 1fr 1fr;gap:10px}
.pill{display:inline-block;padding:4px 10px;border-radius:999px;font-size:12px;background:#24406d;color:#c7d8f4}
.pill.on{background:#2ea043;color:#fff}
.pill.off{background:#e5484d;color:#fff}
.hide{display:none}
.msg{margin-top:10px;font-size:12px;color:#63e6a8;min-height:16px}
.stat{background:#0b1220;border:1px solid #1a2b48;border-radius:10px;padding:10px;text-align:center}
.stat .lbl{font-size:11px;color:#8aa0c0;text-transform:uppercase;letter-spacing:.6px}
.stat .num{font-size:20px;font-weight:700;margin-top:4px;font-variant-numeric:tabular-nums}
@media(max-width:480px){.grid3{grid-template-columns:1fr 1fr}}
</style></head><body>
<div class="wrap">
<h1>ESP32 INA219 Dashboard</h1>
<div class="sub" id="statusLine">connecting…</div>
<div class="tabs">
<div class="tab active" data-tab="dash">Dashboard</div>
<div class="tab" data-tab="relay">Relay</div>
<div class="tab" data-tab="wifi">WiFi AP</div>
<div class="tab" data-tab="sys">System</div>
</div>

<div id="tab-dash">
<div class="card"><h2>INA219 Live</h2>
<div class="grid3">
<div class="stat"><div class="lbl">Voltage</div><div class="num"><span id="v">--</span> V</div></div>
<div class="stat"><div class="lbl">Current</div><div class="num"><span id="i">--</span> A</div></div>
<div class="stat"><div class="lbl">Power</div><div class="num"><span id="p">--</span> W</div></div>
</div>
<div class="row" style="margin-top:10px"><span>Relay output</span><span><span id="relayPill" class="pill off">OFF</span></span></div>
</div></div>

<div id="tab-relay" class="hide">
<div class="card"><h2>Threshold Control</h2>
<div class="row"><span>Relay output</span><span><span id="relayPill2" class="pill off">OFF</span></span></div>
<div class="row"><span>Enable auto thresholds</span>
<label class="toggle"><input type="checkbox" id="enabled"><span class="slider"></span></label></div>
<div class="row"><span>Active HIGH relay</span>
<label class="toggle"><input type="checkbox" id="activeHigh"><span class="slider"></span></label></div>
<div class="grid3" style="margin-top:6px">
<div><label>Voltage (V)</label><input type="number" step="0.1" id="vT">
<div class="row" style="border:0;padding-top:6px"><span style="font-size:12px">Use</span>
<label class="toggle"><input type="checkbox" id="vU"><span class="slider"></span></label></div></div>
<div><label>Current (A)</label><input type="number" step="0.01" id="iT">
<div class="row" style="border:0;padding-top:6px"><span style="font-size:12px">Use</span>
<label class="toggle"><input type="checkbox" id="iU"><span class="slider"></span></label></div></div>
<div><label>Power (W)</label><input type="number" step="0.1" id="pT">
<div class="row" style="border:0;padding-top:6px"><span style="font-size:12px">Use</span>
<label class="toggle"><input type="checkbox" id="pU"><span class="slider"></span></label></div></div>
</div>
<div style="margin-top:12px"><button class="btn" id="saveRelay">Save Settings</button></div>
<div class="msg" id="relayMsg"></div>
</div></div>

<div id="tab-wifi" class="hide">
<div class="card"><h2>WiFi Access Point</h2>
<div class="row"><span>SSID</span><span class="val">ESP32_INA219</span></div>
<div class="row"><span>Password</span><span class="val">••••••••</span></div>
<div class="row"><span>Status</span><span><span class="pill on">ALWAYS ACTIVE</span></span></div>
<div class="row"><span>IP</span><span class="val" id="apIp">192.168.4.1</span></div>
<div class="row"><span>Clients</span><span class="val" id="apClients">0</span></div>
<div class="row"><span>Captive portal</span><span><span class="pill on">ENABLED</span></span></div>
</div></div>

<div id="tab-sys" class="hide">
<div class="card"><h2>System</h2>
<div class="row"><span>Uptime</span><span class="val" id="uptime">--</span></div>
<div class="row"><span>Free heap</span><span class="val" id="heap">--</span></div>
<div class="row"><span>Min free heap</span><span class="val" id="minheap">--</span></div>
<div class="row"><span>Reset reason</span><span class="val" id="rst">--</span></div>
<div style="margin-top:12px">
<button class="btn secondary" id="btnRestart">Restart</button>
<button class="btn danger" id="btnFactory">Factory Reset</button>
</div><div class="msg" id="sysMsg"></div>
</div></div>

</div>
<script>
(function(){
 var tabs=document.querySelectorAll('.tab');
 function show(n){
  document.querySelectorAll('[id^="tab-"]').forEach(function(el){el.classList.add('hide')});
  document.getElementById('tab-'+n).classList.remove('hide');
  tabs.forEach(function(t){t.classList.toggle('active',t.dataset.tab===n)});
 }
 tabs.forEach(function(t){t.addEventListener('click',function(){show(t.dataset.tab)})});
 function $(id){return document.getElementById(id)}
 function setPill(el,on){if(!el)return;el.textContent=on?'ON':'OFF';el.className='pill '+(on?'on':'off')}
 function fmt(n,d){if(n===undefined||n===null)return '--';return Number(n).toFixed(d)}
 function applyState(s){
  $('v').textContent=fmt(s.v,2);$('i').textContent=fmt(s.i,3);$('p').textContent=fmt(s.p,2);
  setPill($('relayPill'),s.relay);setPill($('relayPill2'),s.relay);
  $('enabled').checked=!!s.en;$('activeHigh').checked=!!s.ah;
  $('vT').value=s.vT;$('iT').value=s.iT;$('pT').value=s.pT;
  $('vU').checked=!!s.vU;$('iU').checked=!!s.iU;$('pU').checked=!!s.pU;
  $('apIp').textContent=s.ip||'192.168.4.1';
  $('apClients').textContent=s.cli||0;
  $('uptime').textContent=s.up||'--';
  $('heap').textContent=s.heap||'--';
  $('minheap').textContent=s.minheap||'--';
  $('rst').textContent=s.rst||'--';
  $('statusLine').textContent='online · '+(s.ip||'');
 }
 function poll(){
  fetch('/api/state',{cache:'no-store'}).then(function(r){return r.json()})
   .then(applyState).catch(function(){$('statusLine').textContent='offline — retrying…'})
   .finally(function(){setTimeout(poll,1500)});
 }
 poll();
 function post(u,b){return fetch(u,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(b||{})}).then(function(r){return r.json()})}
 $('saveRelay').addEventListener('click',function(){
  post('/api/config',{
   en:$('enabled').checked,ah:$('activeHigh').checked,
   vT:parseFloat($('vT').value)||0,iT:parseFloat($('iT').value)||0,pT:parseFloat($('pT').value)||0,
   vU:$('vU').checked,iU:$('iU').checked,pU:$('pU').checked
  }).then(function(r){
   $('relayMsg').textContent=r.ok?'Saved.':'Save failed.';
   if(r.ok) setTimeout(poll,150);
   setTimeout(function(){$('relayMsg').textContent=''},2500);
  }).catch(function(){$('relayMsg').textContent='Network error.'});
 });
 $('btnRestart').addEventListener('click',function(){
  if(!confirm('Restart ESP32?'))return;
  post('/api/restart').then(function(){$('sysMsg').textContent='Restarting…'});
 });
 $('btnFactory').addEventListener('click',function(){
  if(!confirm('Factory reset will erase all settings. Continue?'))return;
  post('/api/factory').then(function(){$('sysMsg').textContent='Resetting…'});
 });
})();
</script></body></html>)HTML";

static const char PORTAL_HTML[] PROGMEM = R"HTML(<!DOCTYPE html><html><head>
<meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<meta http-equiv="refresh" content="0;url=http://192.168.4.1/">
<title>ESP32 INA219</title></head>
<body style="font-family:sans-serif;background:#0b1220;color:#e6eefc;text-align:center;padding:40px">
<h2>ESP32 INA219</h2><p>Redirecting…</p>
<a style="color:#63e6a8" href="http://192.168.4.1/">Open dashboard</a>
</body></html>)HTML";

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------
static inline bool timeReached(uint32_t now, uint32_t last, uint32_t interval){
  return (uint32_t)(now - last) >= interval;
}

static void applyRelayHardware(){
  bool level = cfg.relayActiveHigh ? g_relayOn : !g_relayOn;
  digitalWrite(RELAY_PIN, level ? HIGH : LOW);
}
static void setRelay(bool on){
  if(g_relayOn==on) return;
  g_relayOn=on;
  applyRelayHardware();
}

static uint32_t cfgCrc(const Config& c){
  const uint8_t* p = reinterpret_cast<const uint8_t*>(&c);
  uint32_t crc = 0xFFFFFFFFu;
  for(size_t i=0;i<sizeof(Config);++i){
    crc ^= p[i];
    for(int b=0;b<8;++b) crc = (crc>>1) ^ (0xEDB88320u & (0u-(crc&1u)));
  }
  return ~crc;
}

// ---------------------------------------------------------------------------
// Persistent config
// ---------------------------------------------------------------------------
static void loadConfig(){
  prefs.begin("ina219", true);
  size_t got = prefs.getBytesLength("cfg");
  uint32_t storedCrc = prefs.getUInt("crc", 0);
  Config tmp;
  bool ok=false;
  if(got==sizeof(Config)){
    if(prefs.getBytes("cfg",&tmp,sizeof(tmp))==sizeof(tmp) && cfgCrc(tmp)==storedCrc){
      cfg=tmp; ok=true;
    }
  }
  prefs.end();
  if(!ok){
    cfg.vThresh=0; cfg.iThresh=0; cfg.pThresh=0;
    cfg.relayActiveHigh=true; cfg.relayEnabled=false;
    cfg.useVoltage=cfg.useCurrent=cfg.usePower=false;
    cfgDirty=true;
  }
}

static void flushConfigIfDue(uint32_t now, bool force){
  if(!cfgDirty) return;
  if(!force && !timeReached(now, cfgLastFlushMs, NVS_FLUSH_INTERVAL_MS)) return;
  prefs.begin("ina219", false);
  size_t w = prefs.putBytes("cfg", &cfg, sizeof(cfg));
  if(w==sizeof(cfg)){
    prefs.putUInt("crc", cfgCrc(cfg));
    Config rb;
    bool vOk=false;
    if(prefs.getBytes("cfg",&rb,sizeof(rb))==sizeof(rb) &&
       prefs.getUInt("crc",0)==cfgCrc(rb)) vOk=true;
    if(vOk){
      cfgDirty = false;
      cfgLastFlushMs = now;
    }
  }
  prefs.end();
}

// ---------------------------------------------------------------------------
// Sensors
// ---------------------------------------------------------------------------
static void sampleSensors(){
  float v = ina219.getBusVoltage_V();
  float i = ina219.getCurrent_mA()/1000.0f;
  float p = ina219.getPower_mW()/1000.0f;
  if(!isfinite(v)) v=0.0f;
  if(!isfinite(i)) i=0.0f;
  if(!isfinite(p)) p=0.0f;
  g_voltage=v; g_current=i; g_power=p;

  bool shouldOn=false;
  if(cfg.relayEnabled){
    if(cfg.useVoltage && cfg.vThresh>0 && v>=cfg.vThresh) shouldOn=true;
    if(cfg.useCurrent && cfg.iThresh>0 && i>=cfg.iThresh) shouldOn=true;
    if(cfg.usePower   && cfg.pThresh>0 && p>=cfg.pThresh) shouldOn=true;
  }
  setRelay(shouldOn);
}

// ---------------------------------------------------------------------------
// Boot button
// ---------------------------------------------------------------------------
static void pollBootButton(uint32_t now){
  bool raw = digitalRead(BOOT_BUTTON_PIN);
  if(raw==LOW){
    if(btnLowCount<3) btnLowCount++;
    if(btnLowCount>=3){
      if(!btnHeld){
        btnHeld=true;
        btnPressStartMs=now;
        resetTriggered=false;
      } else if(!resetTriggered && timeReached(now,btnPressStartMs,RESET_HOLD_MS)){
        resetTriggered=true;
        doFactoryReset();
      }
    }
  } else {
    btnLowCount=0;
    btnHeld=false;
    resetTriggered=false;
  }
}

// ---------------------------------------------------------------------------
// Factory reset
// ---------------------------------------------------------------------------
static void doFactoryReset(){
  prefs.begin("ina219", false);
  prefs.clear();
  prefs.end();
  for(int k=0;k<6;++k){
    digitalWrite(RELAY_PIN,(k&1)?LOW:HIGH);
    delay(120);
  }
  delay(150);
  ESP.restart();
}

// ---------------------------------------------------------------------------
// HTTP handlers
// ---------------------------------------------------------------------------
static void handleRoot(){ server.send_P(200,"text/html",INDEX_HTML); }
static void handlePortal(){ server.send_P(200,"text/html",PORTAL_HTML); }

static void handleAndroid204(){
  server.send(204,"text/plain","");
}

static void handleState(){
  static char buf[512];
  uint64_t upUs = (uint64_t)esp_timer_get_time();
  unsigned long upSec = (unsigned long)(upUs/1000000ULL);
  int n = snprintf(buf, sizeof(buf),
    "{\"v\":%.3f,\"i\":%.4f,\"p\":%.3f,"
    "\"relay\":%s,\"en\":%s,\"ah\":%s,"
    "\"vT\":%.3f,\"iT\":%.4f,\"pT\":%.3f,"
    "\"vU\":%s,\"iU\":%s,\"pU\":%s,"
    "\"ip\":\"192.168.4.1\",\"cli\":%u,"
    "\"heap\":%u,\"minheap\":%u,"
    "\"up\":\"%lus\",\"rst\":\"%d\"}",
    (double)g_voltage,(double)g_current,(double)g_power,
    g_relayOn?"true":"false",
    cfg.relayEnabled?"true":"false",
    cfg.relayActiveHigh?"true":"false",
    (double)cfg.vThresh,(double)cfg.iThresh,(double)cfg.pThresh,
    cfg.useVoltage?"true":"false",
    cfg.useCurrent?"true":"false",
    cfg.usePower?"true":"false",
    (unsigned)WiFi.softAPgetStationNum(),
    (unsigned)ESP.getFreeHeap(),
    (unsigned)ESP.getMinFreeHeap(),
    upSec,
    (int)esp_reset_reason());
  if(n<0) n=0;
  if(n>=(int)sizeof(buf)) n=(int)sizeof(buf)-1;
  server.sendHeader("Cache-Control","no-store");
  server.setContentLength((size_t)n);
  server.send(200,"application/json","");
  server.sendContent(buf,(size_t)n);
}

static const char* findKey(const char* b, const char* k){
  char needle[24];
  snprintf(needle,sizeof(needle),"\"%s\":",k);
  return strstr(b,needle);
}
static bool jNum(const char* b,const char* k,float& o){
  const char* p=findKey(b,k);
  if(!p) return false;
  p += strlen(k)+3;
  while(*p==' ') p++;
  char* end=nullptr;
  float v=strtof(p,&end);
  if(end==p) return false;
  if(!isfinite(v)) return false;
  o=v; return true;
}
static bool jBool(const char* b,const char* k,bool& o){
  const char* p=findKey(b,k);
  if(!p) return false;
  p += strlen(k)+3;
  while(*p==' ') p++;
  if(!strncmp(p,"true",4)){ o=true; return true; }
  if(!strncmp(p,"false",5)){ o=false; return true; }
  return false;
}

static void handleConfig(){
  if(!server.hasArg("plain")){ server.send(400,"application/json","{\"ok\":false}"); return; }
  const char* body = server.arg("plain").c_str();

  bool en=false,ah=false,vU=false,iU=false,pU=false;
  float vT=cfg.vThresh,iT=cfg.iThresh,pT=cfg.pThresh;
  jBool(body,"en",en); jBool(body,"ah",ah);
  jBool(body,"vU",vU); jBool(body,"iU",iU); jBool(body,"pU",pU);
  jNum(body,"vT",vT);  jNum(body,"iT",iT);  jNum(body,"pT",pT);

  if(!isfinite(vT)||vT<0||vT>32)   vT=0;
  if(!isfinite(iT)||iT<0||iT>3.2f) iT=0;
  if(!isfinite(pT)||pT<0||pT>100)  pT=0;

  cfg.relayEnabled=en; cfg.relayActiveHigh=ah;
  cfg.useVoltage=vU;   cfg.useCurrent=iU;    cfg.usePower=pU;
  cfg.vThresh=vT;      cfg.iThresh=iT;       cfg.pThresh=pT;
  cfgDirty=true;

  applyRelayHardware();
  flushConfigIfDue(millis(), true);

  server.sendHeader("Cache-Control","no-store");
  server.send(200,"application/json","{\"ok\":true}");
}

static void handleRestart(){
  server.sendHeader("Cache-Control","no-store");
  server.send(200,"application/json","{\"ok\":true}");
  server.client().flush();
  delay(150);
  flushConfigIfDue(millis(), true);
  ESP.restart();
}

static void handleFactory(){
  server.sendHeader("Cache-Control","no-store");
  server.send(200,"application/json","{\"ok\":true}");
  server.client().flush();
  delay(200);
  doFactoryReset();
}

static void handleNotFound(){
  server.sendHeader("Location","http://192.168.4.1/",true);
  server.send_P(302,"text/html",PORTAL_HTML);
}

// ---------------------------------------------------------------------------
// WiFi
// ---------------------------------------------------------------------------
static void startAP(){
  WiFi.persistent(false);
  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(IPAddress(192,168,4,1),IPAddress(192,168,4,1),IPAddress(255,255,255,0));
  WiFi.softAP(AP_SSID, AP_PASS, 1, 0, 4);
}

static void startCaptivePortal(){
  dnsServer.setErrorReplyCode(DNSReplyCode::NoError);
  dnsServer.start(53,"*",IPAddress(192,168,4,1));
}

static void maintainAP(uint32_t now){
  if(!timeReached(now,tLastWifiCheck,WIFI_RECHECK_MS)) return;
  tLastWifiCheck=now;
  if(WiFi.getMode()!=WIFI_AP){
    startAP();
    startCaptivePortal();
  }
}

// ---------------------------------------------------------------------------
// Setup / Loop
// ---------------------------------------------------------------------------
void setup(){
  pinMode(RELAY_PIN,OUTPUT);
  digitalWrite(RELAY_PIN,LOW);
  pinMode(BOOT_BUTTON_PIN,INPUT_PULLUP);

  Wire.begin(I2C_SDA,I2C_SCL);
  if(!ina219.begin(&Wire)){ }
  ina219.setCalibration_32V_2A();

  loadConfig();
  applyRelayHardware();
  cfgLastFlushMs = millis();

  startAP();
  startCaptivePortal();

  server.on("/",HTTP_GET,handleRoot);
  server.on("/api/state",HTTP_GET,handleState);
  server.on("/api/config",HTTP_POST,handleConfig);
  server.on("/api/restart",HTTP_POST,handleRestart);
  server.on("/api/factory",HTTP_POST,handleFactory);

  server.on("/generate_204",HTTP_GET,handleAndroid204);
  server.on("/gen_204",HTTP_GET,handleAndroid204);
  server.on("/hotspot-detect.html",HTTP_GET,handlePortal);
  server.on("/canonical.html",HTTP_GET,handlePortal);
  server.on("/success.txt",HTTP_GET,handlePortal);
  server.on("/ncsi.txt",HTTP_GET,handlePortal);
  server.on("/connecttest.txt",HTTP_GET,handlePortal);
  server.onNotFound(handleNotFound);
  server.begin();

  tLastSample=tLastButtonPoll=tLastWifiCheck=millis();
}

void loop(){
  uint32_t now = millis();
  dnsServer.processNextRequest();
  server.handleClient();

  if(timeReached(now,tLastSample,SAMPLE_INTERVAL_MS)){ tLastSample=now; sampleSensors(); }
  if(timeReached(now,tLastButtonPoll,BUTTON_POLL_MS)){ tLastButtonPoll=now; pollBootButton(now); }
  maintainAP(now);
  flushConfigIfDue(now,false);

  delay(1);
}
