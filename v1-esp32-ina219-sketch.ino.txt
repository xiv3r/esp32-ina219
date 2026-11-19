/*
 * ESP32 INA219 Relay + WiFi AP Dashboard
 */

#include <Arduino.h>
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

#define CONFIG_VERSION 2

#pragma pack(push, 1)
struct Config {
  uint8_t version;
  float vThresh;
  float iThresh;
  float pThresh;
  bool  relayActiveHigh;
  bool  relayEnabled;
  bool  useVoltage;
  bool  useCurrent;
  bool  usePower;
  char    apSsid[33];
  char    apPass[64];
  uint8_t apChannel;
  bool    apHidden;
  uint8_t apMaxClients;
};
#pragma pack(pop)

static Config cfg = {
  CONFIG_VERSION,
  0.0f, 0.0f, 0.0f,
  true, false, false, false, false,
  "ESP32_INA219", "12345678",
  1, false, 4
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
static String portalUrl();

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
input[type=number],input[type=text]{width:100%;padding:10px;background:#0b1220;border:1px solid #24406d;border-radius:8px;color:#e6eefc;font-size:16px;outline:none}
input:focus{border-color:#1f6feb}
input:disabled{opacity:.5;cursor:not-allowed}
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
<div class="row"><span>Status</span><span><span class="pill on">ALWAYS ACTIVE</span></span></div>
<div class="row"><span>IP</span><span class="val" id="apIp">192.168.4.1</span></div>
<div class="row"><span>Clients</span><span class="val" id="apClients">0</span></div>
<div class="row"><span>Captive portal</span><span><span class="pill on">ENABLED</span></span></div>
<label>SSID (1–32 chars)</label>
<input type="text" id="ssid" maxlength="32">
<div class="row"><span>Change password</span>
<label class="toggle"><input type="checkbox" id="chgPass"><span class="slider"></span></label></div>
<label>New password (empty = open network, or 8–63 chars)</label>
<input type="text" id="pass" maxlength="63" disabled placeholder="unchanged">
<div class="grid3" style="margin-top:6px">
<div><label>Channel (1–13)</label><input type="number" min="1" max="13" id="ch"></div>
<div><label>Max clients (1–8)</label><input type="number" min="1" max="8" id="maxCli"></div>
<div><label>Hidden SSID</label>
<label class="toggle"><input type="checkbox" id="hidden"><span class="slider"></span></label></div>
</div>
<div style="margin-top:12px"><button class="btn" id="saveAp">Save &amp; Reboot</button></div>
<div class="msg" id="apMsg"></div>
<div style="margin-top:14px;font-size:11px;color:#8aa0c0;line-height:1.5">
Forgot the AP password? Hold the BOOT button for 5 seconds to factory reset.
</div>
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
  if(!$('enabled').dataset.dirty) $('enabled').checked=!!s.en;
  if(!$('activeHigh').dataset.dirty) $('activeHigh').checked=!!s.ah;
  if(!$('vT').dataset.dirty) $('vT').value=s.vT;
  if(!$('iT').dataset.dirty) $('iT').value=s.iT;
  if(!$('pT').dataset.dirty) $('pT').value=s.pT;
  if(!$('vU').dataset.dirty) $('vU').checked=!!s.vU;
  if(!$('iU').dataset.dirty) $('iU').checked=!!s.iU;
  if(!$('pU').dataset.dirty) $('pU').checked=!!s.pU;
  $('apIp').textContent=s.ip||'192.168.4.1';
  $('apClients').textContent=s.cli||0;
  if(!$('ssid').dataset.dirty){
    $('ssid').value=s.ssid||'';
    $('ch').value=s.ch||1;
    $('maxCli').value=s.maxCli||4;
    $('hidden').checked=!!s.hidden;
  }
  if(!$('chgPass').checked){
    $('pass').value='';
  }
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
 var RELAY_FIELDS=['enabled','activeHigh','vT','iT','pT','vU','iU','pU'];
 RELAY_FIELDS.forEach(function(id){
   var el=$(id); if(!el) return;
   var ev=(el.type==='checkbox')?'change':'input';
   el.addEventListener(ev,function(){el.dataset.dirty='1'});
 });
 function clearRelayDirty(){
   RELAY_FIELDS.forEach(function(id){var el=$(id);if(el)delete el.dataset.dirty;});
 }
 $('saveRelay').addEventListener('click',function(){
  var en=$('enabled').checked;
  var vT=parseFloat($('vT').value)||0, iT=parseFloat($('iT').value)||0, pT=parseFloat($('pT').value)||0;
  var vU=$('vU').checked, iU=$('iU').checked, pU=$('pU').checked;
  if(en && !((vU&&vT>0)||(iU&&iT>0)||(pU&&pT>0))){
    $('relayMsg').textContent='Enable at least one channel with a threshold > 0, or turn auto off.';
    return;
  }
  post('/api/config',{
   en:en,ah:$('activeHigh').checked,
   vT:vT,iT:iT,pT:pT,
   vU:vU,iU:iU,pU:pU
  }).then(function(r){
   if(r.ok){ $('relayMsg').textContent='Saved.'; clearRelayDirty(); setTimeout(poll,150); }
   else if(r && r.err==='no-threshold'){ $('relayMsg').textContent='Enable at least one channel with a threshold > 0.'; }
   else if(r && r.err==='vT-range'){ $('relayMsg').textContent='Voltage threshold must be 0–32 V.'; }
   else if(r && r.err==='iT-range'){ $('relayMsg').textContent='Current threshold must be 0–3.2 A.'; }
   else if(r && r.err==='pT-range'){ $('relayMsg').textContent='Power threshold must be 0–100 W.'; }
   else { $('relayMsg').textContent='Save failed.'; }
   setTimeout(function(){$('relayMsg').textContent=''},3000);
  }).catch(function(){$('relayMsg').textContent='Network error.'});
 });
 ['ssid','ch','maxCli','hidden'].forEach(function(id){
   var el=$(id); if(!el) return;
   el.addEventListener('input',function(){el.dataset.dirty='1'});
 });
 $('chgPass').addEventListener('change',function(){
   $('pass').disabled = !this.checked;
   if(this.checked){ $('pass').focus(); } else { $('pass').value=''; }
 });
 $('saveAp').addEventListener('click',function(){
  var ssid=$('ssid').value.trim();
  var changePass=$('chgPass').checked;
  var pass=$('pass').value;
  function utf8Len(s){
    var n=0;
    for(var i=0;i<s.length;i++){
      var c=s.charCodeAt(i);
      if(c<0x80) n+=1;
      else if(c<0x800) n+=2;
      else if(c>=0xD800&&c<=0xDBFF){
        if(i+1<s.length){
          var d=s.charCodeAt(i+1);
          if(d>=0xDC00&&d<=0xDFFF){ n+=4; i++; }
          else { n+=3; }
        } else { n+=3; }
      }
      else n+=3;
    }
    return n;
  }
  var ssidBytes=utf8Len(ssid);
  if(ssidBytes<1||ssidBytes>32){$('apMsg').textContent='SSID must be 1–32 bytes (UTF-8). Current: '+ssidBytes+' bytes.';return}
  if(changePass && pass.length!==0 && (pass.length<8||pass.length>63)){$('apMsg').textContent='Password must be empty or 8–63 chars.';return}
  if(!confirm('Changing WiFi AP settings will disconnect you and reboot the ESP32. Continue?'))return;
  var body={
   ssid:ssid,
   ch:parseFloat($('ch').value)||1,
   maxCli:parseFloat($('maxCli').value)||4,
   hidden:$('hidden').checked
  };
  if(changePass){ body.pass=pass; }
  $('apMsg').textContent='Saving and rebooting… you will be disconnected.';
  post('/api/ap',body).then(function(r){
   if(r && r.ok){ $('apMsg').textContent='Rebooting… reconnect to your AP.'; }
   else {
     $('apMsg').textContent='Save failed.';
     ['ssid','ch','maxCli','hidden'].forEach(function(id){
       var el=$(id); if(el) delete el.dataset.dirty;
     });
   }
  }).catch(function(){ $('apMsg').textContent='Rebooting… reconnect to your AP.'; });
 });
 $('btnRestart').addEventListener('click',function(){
  if(!confirm('Restart ESP32?'))return;
  $('sysMsg').textContent='Restarting…';
  post('/api/restart').then(function(){$('sysMsg').textContent='Restarting…'})
    .catch(function(){$('sysMsg').textContent='Restart sent — reconnect if page drops.'});
 });
 $('btnFactory').addEventListener('click',function(){
  if(!confirm('Factory reset will erase all settings. Continue?'))return;
  $('sysMsg').textContent='Resetting…';
  post('/api/factory').then(function(){$('sysMsg').textContent='Resetting…'})
    .catch(function(){$('sysMsg').textContent='Reset sent — reconnect to default AP.'});
 });
})();
</script></body></html>)HTML";

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
    if(prefs.getBytes("cfg",&tmp,sizeof(tmp))==sizeof(tmp)
       && tmp.version==CONFIG_VERSION
       && cfgCrc(tmp)==storedCrc){
      cfg=tmp; ok=true;
    }
  }
  prefs.end();
  if(!ok){
    cfg.version          = CONFIG_VERSION;
    cfg.vThresh=0; cfg.iThresh=0; cfg.pThresh=0;
    cfg.relayActiveHigh=true; cfg.relayEnabled=false;
    cfg.useVoltage=cfg.useCurrent=cfg.usePower=false;
    strncpy(cfg.apSsid, AP_SSID, sizeof(cfg.apSsid)-1); cfg.apSsid[sizeof(cfg.apSsid)-1]=0;
    strncpy(cfg.apPass, AP_PASS, sizeof(cfg.apPass)-1); cfg.apPass[sizeof(cfg.apPass)-1]=0;
    cfg.apChannel=1; cfg.apHidden=false; cfg.apMaxClients=4;
    cfgDirty=true;
  }
  if(!cfg.apSsid[0]){ strncpy(cfg.apSsid, AP_SSID, sizeof(cfg.apSsid)-1); cfg.apSsid[sizeof(cfg.apSsid)-1]=0; cfgDirty=true; }
  if(cfg.apChannel<1||cfg.apChannel>13){ cfg.apChannel=1; cfgDirty=true; }
  if(cfg.apMaxClients<1||cfg.apMaxClients>8){ cfg.apMaxClients=4; cfgDirty=true; }
}

static void flushConfigIfDue(uint32_t now, bool force){
  if(!cfgDirty) return;
  if(!force && !timeReached(now, cfgLastFlushMs, NVS_FLUSH_INTERVAL_MS)) return;
  prefs.begin("ina219", false);
  size_t w = prefs.putBytes("cfg", &cfg, sizeof(cfg));
  if(w==sizeof(cfg)){
    size_t cw = prefs.putUInt("crc", cfgCrc(cfg));
    if(cw != sizeof(uint32_t)){ prefs.end(); return; }
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

  const float H = 0.02f;
  bool shouldOn=false;
  if(cfg.relayEnabled){
    if(cfg.useVoltage && cfg.vThresh>0){
      float trip = g_relayOn ? cfg.vThresh*(1.0f-H) : cfg.vThresh;
      if(v>=trip) shouldOn=true;
    }
    if(cfg.useCurrent && cfg.iThresh>0){
      float trip = g_relayOn ? cfg.iThresh*(1.0f-H) : cfg.iThresh;
      if(i>=trip) shouldOn=true;
    }
    if(cfg.usePower && cfg.pThresh>0){
      float trip = g_relayOn ? cfg.pThresh*(1.0f-H) : cfg.pThresh;
      if(p>=trip) shouldOn=true;
    }
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
  cfg = Config{
    CONFIG_VERSION,
    0.0f, 0.0f, 0.0f,
    true, false, false, false, false,
    "ESP32_INA219", "12345678",
    1, false, 4
  };
  g_relayOn = false;
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
static void handleRoot(){
  server.sendHeader("Cache-Control","no-store, no-cache, must-revalidate");
  server.sendHeader("Pragma","no-cache");
  server.sendHeader("Expires","0");
  server.send_P(200,"text/html",INDEX_HTML);
}

static void handlePortal(){
  server.sendHeader("Cache-Control","no-store, no-cache, must-revalidate");
  server.sendHeader("Pragma","no-cache");
  server.sendHeader("Expires","0");
  server.send_P(200,"text/html",INDEX_HTML);
}

static void handleAndroid204(){
  String url = portalUrl();
  server.sendHeader("Location", url, true);
  server.sendHeader("Cache-Control","no-store, no-cache, must-revalidate");
  server.sendHeader("Pragma","no-cache");
  server.sendHeader("Expires","0");
  server.send(302,"text/plain","");
}

static void jsonEscapeInto(const char* in, char* out, size_t outLen){
  if(!out || outLen==0) return;
  size_t o=0;
  for(const char* p=in; *p && o+7<outLen; ++p){
    unsigned char c=(unsigned char)*p;
    if(c=='"' || c=='\\'){
      out[o++]='\\'; out[o++]=(char)c;
    } else if(c=='\n'){ out[o++]='\\'; out[o++]='n'; }
    else if(c=='\r'){ out[o++]='\\'; out[o++]='r'; }
    else if(c=='\t'){ out[o++]='\\'; out[o++]='t'; }
    else if(c<0x20){
      int n=snprintf(out+o,outLen-o,"\\u%04x",c);
      if(n>0) o+=(size_t)n;
    } else {
      out[o++]=(char)c;
    }
  }
  out[o]='\0';
}

static void handleState(){
  static char buf[1400];
  uint64_t upUs = (uint64_t)esp_timer_get_time();
  unsigned long upSec = (unsigned long)(upUs/1000000ULL);
  String ip = WiFi.softAPIP().toString();
  char jsonEscSsid[sizeof(cfg.apSsid)*6 + 1];
  jsonEscapeInto(cfg.apSsid, jsonEscSsid, sizeof(jsonEscSsid));
  int n = snprintf(buf, sizeof(buf),
    "{\"v\":%.3f,\"i\":%.4f,\"p\":%.3f,"
    "\"relay\":%s,\"en\":%s,\"ah\":%s,"
    "\"vT\":%.3f,\"iT\":%.4f,\"pT\":%.3f,"
    "\"vU\":%s,\"iU\":%s,\"pU\":%s,"
    "\"ip\":\"%s\",\"cli\":%u,"
    "\"ssid\":\"%s\",\"ch\":%u,\"hidden\":%s,\"maxCli\":%u,"
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
    ip.c_str(),
    (unsigned)WiFi.softAPgetStationNum(),
    jsonEscSsid,
    (unsigned)cfg.apChannel,
    cfg.apHidden?"true":"false",
    (unsigned)cfg.apMaxClients,
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
  int nl=snprintf(needle,sizeof(needle),"\"%s\":",k);
  if(nl<=0||nl>=(int)sizeof(needle)) return nullptr;
  const char* p=b;
  while((p=strstr(p,needle))){
    bool inString=false;
    for(const char* q=b; q<p; ++q){
      if(*q=='\\'){ if(q[1]) ++q; continue; }
      if(*q=='"') inString=!inString;
    }
    if(!inString) return p;
    p += nl;
  }
  return nullptr;
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
static bool jStr(const char* b,const char* k,char* out,size_t outLen){
  if(!out||outLen==0) return false;
  const char* p=findKey(b,k);
  if(!p) return false;
  p += strlen(k)+3;
  while(*p==' ') p++;
  if(*p!='"') return false;
  p++;
  size_t i=0;
  while(*p && *p!='"' && i<outLen-1){
    if(*p=='\\' && p[1]){
      p++;
      char e=*p++;
      char c=e;
      if(e=='n') c='\n';
      else if(e=='t') c='\t';
      else if(e=='r') c='\r';
      else if(e=='b') c='\b';
      else if(e=='f') c='\f';
      else if(e=='/') c='/';
      else if(e=='"') c='"';
      else if(e=='\\') c='\\';
      else if(e=='u'){
        if(p[0]&&p[1]&&p[2]&&p[3]){
          char hex[5]={p[0],p[1],p[2],p[3],0};
          char* endp=nullptr;
          unsigned long cp=strtoul(hex,&endp,16);
          if(endp!=hex+4){ c='u'; }
          else {
            p+=4;
            if(cp>=0xD800UL && cp<=0xDBFFUL && p[0]=='\\' && p[1]=='u'
               && p[2]&&p[3]&&p[4]&&p[5]){
              char hex2[5]={p[2],p[3],p[4],p[5],0};
              char* endp2=nullptr;
              unsigned long lo=strtoul(hex2,&endp2,16);
              if(endp2==hex2+4 && lo>=0xDC00UL && lo<=0xDFFFUL){
                cp = 0x10000UL + ((cp-0xD800UL)<<10) + (lo-0xDC00UL);
                p += 6;
              }
            }
            if(cp==0){ continue; }
            if(cp<=0x7F) c=(char)cp;
            else if(cp<=0x7FF){
              if(i+1>=outLen-1) break;
              out[i++]=(char)(0xC0|(cp>>6));
              c=(char)(0x80|(cp&0x3F));
            } else if(cp<=0xFFFF){
              if(i+2>=outLen-1) break;
              out[i++]=(char)(0xE0|(cp>>12));
              out[i++]=(char)(0x80|((cp>>6)&0x3F));
              c=(char)(0x80|(cp&0x3F));
            } else {
              if(i+3>=outLen-1) break;
              out[i++]=(char)(0xF0|(cp>>18));
              out[i++]=(char)(0x80|((cp>>12)&0x3F));
              out[i++]=(char)(0x80|((cp>>6)&0x3F));
              c=(char)(0x80|(cp&0x3F));
            }
          }
        } else {
          c='u';
        }
      }
      out[i++]=c;
    } else {
      out[i++]=*p++;
    }
  }
  out[i]='\0';
  return (*p=='"');
}

static void handleConfig(){
  if(!server.hasArg("plain")){ server.send(400,"application/json","{\"ok\":false}"); return; }
  String bodyStr = server.arg("plain");
  const char* body = bodyStr.c_str();

  bool en=cfg.relayEnabled,ah=cfg.relayActiveHigh;
  bool vU=cfg.useVoltage,iU=cfg.useCurrent,pU=cfg.usePower;
  float vT=cfg.vThresh,iT=cfg.iThresh,pT=cfg.pThresh;
  jBool(body,"en",en); jBool(body,"ah",ah);
  jBool(body,"vU",vU); jBool(body,"iU",iU); jBool(body,"pU",pU);
  jNum(body,"vT",vT);  jNum(body,"iT",iT);  jNum(body,"pT",pT);

  if(!isfinite(vT)||vT<0||vT>32){
    server.sendHeader("Cache-Control","no-store");
    server.send(400,"application/json","{\"ok\":false,\"err\":\"vT-range\"}");
    return;
  }
  if(!isfinite(iT)||iT<0||iT>3.2f){
    server.sendHeader("Cache-Control","no-store");
    server.send(400,"application/json","{\"ok\":false,\"err\":\"iT-range\"}");
    return;
  }
  if(!isfinite(pT)||pT<0||pT>100){
    server.sendHeader("Cache-Control","no-store");
    server.send(400,"application/json","{\"ok\":false,\"err\":\"pT-range\"}");
    return;
  }
  if(en){
    bool anyArmed = (vU && vT>0.0f) || (iU && iT>0.0f) || (pU && pT>0.0f);
    if(!anyArmed){
      server.sendHeader("Cache-Control","no-store");
      server.send(400,"application/json","{\"ok\":false,\"err\":\"no-threshold\"}");
      return;
    }
  }

  cfg.version = CONFIG_VERSION;
  cfg.relayEnabled=en; cfg.relayActiveHigh=ah;
  cfg.useVoltage=vU;   cfg.useCurrent=iU;    cfg.usePower=pU;
  cfg.vThresh=vT;      cfg.iThresh=iT;       cfg.pThresh=pT;
  cfgDirty=true;

  if(!cfg.relayEnabled) g_relayOn = false;
  applyRelayHardware();
  flushConfigIfDue(millis(), true);
  if(cfgDirty){
    server.sendHeader("Cache-Control","no-store");
    server.send(500,"application/json","{\"ok\":false,\"err\":\"nvs\"}");
    return;
  }

  server.sendHeader("Cache-Control","no-store");
  server.send(200,"application/json","{\"ok\":true}");
}

static bool validSsid(const char* s){ size_t n=strlen(s); return n>=1 && n<=32; }
static bool validPass(const char* s){ size_t n=strlen(s); return n==0 || (n>=8 && n<=63); }

static void handleApConfig(){
  if(!server.hasArg("plain")){ server.send(400,"application/json","{\"ok\":false,\"err\":\"body\"}"); return; }
  String bodyStr = server.arg("plain");
  const char* body = bodyStr.c_str();

  char ssid[33] = {0};
  char pass[64] = {0};
  float ch = (float)cfg.apChannel;
  float maxCli = (float)cfg.apMaxClients;
  bool  hidden = cfg.apHidden;

  if(!jStr(body,"ssid",ssid,sizeof(ssid)) || !validSsid(ssid)){
    server.send(400,"application/json","{\"ok\":false,\"err\":\"ssid\"}"); return;
  }
  if(jStr(body,"pass",pass,sizeof(pass))){
    if(!validPass(pass)){
      server.send(400,"application/json","{\"ok\":false,\"err\":\"pass\"}"); return;
    }
  } else {
    strncpy(pass, cfg.apPass, sizeof(pass)-1);
    pass[sizeof(pass)-1]=0;
  }
  jNum(body,"ch",ch);
  jBool(body,"hidden",hidden);
  jNum(body,"maxCli",maxCli);

  strncpy(cfg.apSsid, ssid, sizeof(cfg.apSsid)-1); cfg.apSsid[sizeof(cfg.apSsid)-1]=0;
  strncpy(cfg.apPass, pass, sizeof(cfg.apPass)-1); cfg.apPass[sizeof(cfg.apPass)-1]=0;
  cfg.apChannel    = (uint8_t)constrain((int)ch, 1, 13);
  cfg.apHidden     = hidden;
  cfg.apMaxClients = (uint8_t)constrain((int)maxCli, 1, 8);
  cfgDirty = true;

  flushConfigIfDue(millis(), true);
  if(cfgDirty){
    server.sendHeader("Cache-Control","no-store");
    server.send(500,"application/json","{\"ok\":false,\"err\":\"nvs\"}");
    server.client().stop();
    return;
  }

  server.sendHeader("Cache-Control","no-store");
  server.send(200,"application/json","{\"ok\":true,\"reboot\":true}");
  delay(600);
  ESP.restart();
}

static void handleRestart(){
  server.sendHeader("Cache-Control","no-store");
  server.send(200,"application/json","{\"ok\":true}");
  delay(400);
  flushConfigIfDue(millis(), true);
  ESP.restart();
}

static void handleFactory(){
  server.sendHeader("Cache-Control","no-store");
  server.send(200,"application/json","{\"ok\":true}");
  delay(400);
  doFactoryReset();
}

static void handleNotFound(){
  if(server.method() != HTTP_GET){
    server.send(405,"text/plain","");
    return;
  }
  String url = portalUrl();
  server.sendHeader("Location", url, true);
  server.sendHeader("Cache-Control","no-store, no-cache, must-revalidate");
  server.sendHeader("Pragma","no-cache");
  server.sendHeader("Expires","0");
  server.send(302,"text/plain","");
}

// ---------------------------------------------------------------------------
// WiFi
// ---------------------------------------------------------------------------
static String portalUrl(){
  return String("http://") + WiFi.softAPIP().toString() + "/";
}

static void startAP(){
  WiFi.persistent(false);
  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(IPAddress(192,168,4,1),IPAddress(192,168,4,1),IPAddress(255,255,255,0));
  const char* ssid = cfg.apSsid;
  const char* pass = cfg.apPass;
  uint8_t ch  = (cfg.apChannel    ? cfg.apChannel    : 1);
  uint8_t maxc= (cfg.apMaxClients ? cfg.apMaxClients : 4);
  bool up = WiFi.softAP(ssid, pass, ch, cfg.apHidden, maxc);
  if(!up && pass[0] && strlen(pass) < 8){
    up = WiFi.softAP(ssid, nullptr, ch, cfg.apHidden, maxc);
    if(up){ cfg.apPass[0]=0; cfgDirty=true; }
  }
  if(!up && ch > 11){
    ch = 11;
    up = WiFi.softAP(ssid, pass, ch, cfg.apHidden, maxc);
    if(up){
      cfg.apChannel = ch;
      cfgDirty = true;
    }
  }
  if(!up && ch > 1){
    ch = 1;
    up = WiFi.softAP(ssid, pass, ch, cfg.apHidden, maxc);
    if(up){
      cfg.apChannel = ch;
      cfgDirty = true;
    }
  }
}

static void startCaptivePortal(){
  dnsServer.stop();
  dnsServer.setErrorReplyCode(DNSReplyCode::NoError);
  dnsServer.start(53,"*",WiFi.softAPIP());
}

static void maintainAP(uint32_t now){
  if(!timeReached(now,tLastWifiCheck,WIFI_RECHECK_MS)) return;
  tLastWifiCheck=now;
  wifi_mode_t m = WiFi.getMode();
  bool modeOk = (m == WIFI_AP || m == WIFI_AP_STA);
  IPAddress apIp = WiFi.softAPIP();
  bool ipOk = (apIp != IPAddress(0,0,0,0));
  if(!modeOk || !ipOk){
    startAP();
    startCaptivePortal();
  }
}

// ---------------------------------------------------------------------------
// Setup / Loop
// ---------------------------------------------------------------------------
void setup(){
  pinMode(BOOT_BUTTON_PIN,INPUT_PULLUP);

  Wire.begin(I2C_SDA,I2C_SCL);
  if(!ina219.begin(&Wire)){ }
  ina219.setCalibration_32V_2A();

  loadConfig();
  digitalWrite(RELAY_PIN, cfg.relayActiveHigh ? LOW : HIGH);
  pinMode(RELAY_PIN, OUTPUT);
  applyRelayHardware();
  cfgLastFlushMs = millis();

  startAP();
  startCaptivePortal();

  server.on("/",HTTP_GET,handleRoot);
  server.on("/index.html",HTTP_GET,handleRoot);
  server.on("/api/state",HTTP_GET,handleState);
  server.on("/api/config",HTTP_POST,handleConfig);
  server.on("/api/ap",HTTP_POST,handleApConfig);
  server.on("/api/restart",HTTP_POST,handleRestart);
  server.on("/api/factory",HTTP_POST,handleFactory);

  server.on("/generate_204",HTTP_GET,handleAndroid204);
  server.on("/gen_204",HTTP_GET,handleAndroid204);
  server.on("/hotspot-detect.html",HTTP_GET,handlePortal);
  server.on("/library/test/success.html",HTTP_GET,handlePortal);
  server.on("/canonical.html",HTTP_GET,handlePortal);
  server.on("/connecttest.txt",HTTP_GET,handlePortal);
  server.on("/ncsi.txt",HTTP_GET,handlePortal);
  server.on("/redirect",HTTP_GET,handlePortal);
  server.on("/success.txt",HTTP_GET,handlePortal);
  server.on("/kindle-wifi/wifistub.html",HTTP_GET,handlePortal);
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
