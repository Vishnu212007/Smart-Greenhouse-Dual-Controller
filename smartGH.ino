// =====================================================================
//  PLANT MONITOR v2 + TELEGRAM & STM32 ACTUATOR BRIDGE (ESP32)
//  - Offline web dashboard & LittleFS logging
//  - Telegram Bot: /status, /fan_on, /fan_off, /vent_open, /vent_close
//  - Web Dashboard Controls: Water Pump, Auto-water, 12V Fan, Servo Vent
//  - UART2 output to STM32: Sensor telemetry + actuator commands
// =====================================================================
#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <Preferences.h>
#include <LittleFS.h>
#include <time.h>
#include <sys/time.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <BH1750.h>
#include <DHT.h>
#include <esp_task_wdt.h>   
#include <ArduinoOTA.h>     
#include <WiFiClientSecure.h>
#include <HTTPClient.h>

#define WDT_TIMEOUT 15      // seconds - board auto-resets if loop hangs

// ===================== USER SETTINGS =====================
// WARNING: Do not put your real Wi-Fi or Telegram keys here when uploading to GitHub!
const char* WIFI_SSID     = "YOUR_WIFI_SSID_HERE";
const char* WIFI_PASSWORD = "YOUR_WIFI_PASSWORD_HERE";

const char* HOSTNAME      = "plantmonitor";     // http://plantmonitor.local
const char* AP_SSID       = "PlantMonitor";     // hotspot if Wi-Fi fails
const char* AP_PASSWORD   = "12345678";

const long  TZ_OFFSET_SEC = 19800;              // India (IST, UTC+5:30)
const char* OTA_PASSWORD  = "plant1234";        

#define BTN_PIN  13                             // push button to GND = phone alerts ON/OFF

#define ENABLE_TELEGRAM    1                    
const char* TG_TOKEN = "YOUR_TELEGRAM_BOT_TOKEN_HERE";
const char* TG_CHAT  = "YOUR_TELEGRAM_CHAT_ID_HERE";
const unsigned long TG_POLL_MS = 3500UL;        
const unsigned long ALERT_COOLDOWN_MS = 3600000UL; 

#define LIGHT_BRIGHT_LX 1000                    

#define ENABLE_PUMP        1                    
#define PUMP_PIN           26
#define PUMP_ACTIVE_LEVEL  HIGH                 
const unsigned long PUMP_RUN_MS        = 5000UL;     
const unsigned long PUMP_COOLDOWN_MS   = 600000UL;   
const unsigned long PUMP_MANUAL_MAX_MS = 60000UL;    
const int           AUTO_MAX_PER_DAY   = 6;          

#define SCREEN_WIDTH   128
#define SCREEN_HEIGHT  64
#define OLED_RESET     -1
#define SCREEN_ADDRESS 0x3C

#define MOISTURE_PIN 34
#define LED_PIN      2
#define DHTPIN       4
#define DHTTYPE      DHT22

#define RXD2 16
#define TXD2 17

const unsigned long SENSOR_INTERVAL_MS = 2000;
const unsigned long PAGE_INTERVAL_MS   = 4000;

// ===================== LOG TIERS =====================
#define LIVE_N 180      
#define M5_N   288      
#define H1_N   744      
#define D1_N   366      

struct __attribute__((packed)) Rec {
  uint32_t t;   
  uint16_t l;   
  int16_t  c;   
  uint8_t  m;   
  uint8_t  h;   
};

struct Tier {
  const char* file; uint32_t iv; uint16_t cap; Rec* buf;
  bool useMillis; bool local; uint8_t flushEvery;
  uint16_t count; uint16_t head; uint32_t bucket; uint16_t n;
  float sM, sL, sC, sH; uint8_t unsaved;
};

Rec bufLive[LIVE_N], buf5m[M5_N], buf1h[H1_N], buf1d[D1_N];
Tier tLive = { nullptr,     10,    LIVE_N, bufLive, true,  false, 0 };
Tier t5m   = { "/m5.bin",   300,   M5_N,   buf5m,   false, false, 6 };   
Tier t1h   = { "/h1.bin",   3600,  H1_N,   buf1h,   false, false, 1 };
Tier t1d   = { "/d1.bin",   86400, D1_N,   buf1d,   false, true,  1 };

// ===================== GLOBALS =====================
struct Profile { const char* name; uint8_t dry; int8_t tMin, tMax; uint8_t hMin, hMax; uint16_t lMin; };
const Profile PROFILES[] = {
  {"General houseplant",   30, 18, 30, 40, 70, 1000},
  {"Succulent / Cactus",   15, 15, 35, 20, 50, 3000},
  {"Tomato / Vegetables",  40, 18, 32, 50, 75, 8000},
  {"Fern / Tropical",      45, 18, 28, 60, 85,  500},
  {"Herbs (Basil, Mint)",  35, 18, 30, 40, 70, 5000},
  {"Orchid",               35, 18, 28, 50, 80, 1500},
};
#define PROFILE_COUNT ((int)(sizeof(PROFILES) / sizeof(PROFILES[0])))
uint8_t  profileIdx = 0;
uint8_t  dhtFails = 0, lightFails = 0, faults = 0;   
uint32_t brightSec = 0, brightDay = 0;

WebServer server(80);
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
BH1750 lightMeter;
DHT dht(DHTPIN, DHTTYPE);
Preferences prefs;

int  airVal   = 3500;
int  waterVal = 1500;
int  dryThr   = 30;
bool autoPump = false;
bool logOn    = true;
bool oledOn   = true;
bool alertsOn = true;       

// Actuator State Globals
bool autoFanState = false;
bool servoVentOpen = false;
bool manualOverride = false;
unsigned long manualOverrideUntil = 0;
const unsigned long OVERRIDE_DURATION_MS = 600000UL; // 10 min override lock

float smoothRaw   = 0;
float smoothTemp  = 0;      
float smoothHum   = 0;      
bool  rawPrimed   = false;
bool  dhtPrimed   = false;
int   moisturePct = 0;
float lux         = 0;
float temperature = 0;
float humidity    = 0;
int   minMoist    = 100, maxMoist = 0;
float minLux      = 1e9, maxLux   = 0;
bool  apMode      = false;

uint32_t monSec = 0;       
uint32_t sinceEpoch = 0;   

bool          pumpOn = false;
bool          pumpEverRun = false;
uint8_t       pumpSrc = 0;                 
unsigned long pumpStartMs = 0, pumpEndMs = 0, pumpLimitMs = 0;
uint32_t      pumpTotal = 0;
int           autoRunsToday = 0;
uint32_t      autoDay = 0;

struct Evt { uint32_t t; uint8_t src; uint16_t sec; };
Evt evts[20];
int evCount = 0, evHead = 0;

long tgLastUpdateId = 0;   

// ===================== WEB PAGE =====================
const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>Plant Monitor</title>
<style>
:root{--bd:#27325a;--mut:#8b9ac8}
*{box-sizing:border-box;margin:0;padding:0}
body{font-family:system-ui,-apple-system,Segoe UI,Roboto,sans-serif;color:#f1f5ff;min-height:100vh;padding:16px;background-color:#070b1a;
 background-image:radial-gradient(800px 480px at 8% -8%,#6d28d955,transparent),radial-gradient(800px 480px at 100% 0%,#0891b255,transparent),radial-gradient(700px 500px at 50% 110%,#16a34a33,transparent);background-attachment:fixed}
.wrap{max-width:720px;margin:0 auto}
h1{font-size:1.7rem;text-align:center;background:linear-gradient(90deg,#4ade80,#38bdf8,#a78bfa,#f472b6);-webkit-background-clip:text;background-clip:text;color:transparent}
.sub{text-align:center;color:var(--mut);font-size:.8rem;margin:4px 0 10px}
.chips{display:flex;flex-wrap:wrap;gap:6px;justify-content:center;margin-bottom:14px}
.chip{font-size:.7rem;padding:4px 10px;border-radius:99px;background:#121a33;border:1px solid var(--bd);color:#cbd5ff}
.cards{display:grid;grid-template-columns:1fr 1fr;gap:12px;margin-bottom:12px}
.card,.box,.tile{position:relative;background:linear-gradient(160deg,#17214b,#0f1731);border:1px solid var(--bd);border-radius:16px;padding:14px;overflow:hidden}
.card:before,.tile:before{content:"";position:absolute;left:0;top:0;right:0;height:4px;background:var(--c)}
.box{margin-bottom:12px}
.t{font-size:.7rem;text-transform:uppercase;letter-spacing:.06em;color:var(--mut)}
.v{font-size:1.9rem;font-weight:800;margin-top:4px}
.s{font-size:.78rem;margin-top:3px;color:var(--mut)}
.m{color:#38bdf8}.l{color:#facc15}.temp{color:#fb7185}.hum{color:#c084fc}
.badge{display:inline-block;padding:3px 11px;border-radius:99px;font-size:.72rem;font-weight:700;color:#0b1120}
.b-dry{background:#f87171}.b-good{background:#4ade80}.b-wet{background:#38bdf8}
.flex{display:flex;align-items:center;justify-content:space-between;gap:8px}
.ring{width:84px;height:84px;flex:none}
.full{grid-column:1/-1}
.bar{height:10px;border-radius:9px;background:#1e2a52;margin-top:8px;overflow:hidden}
.bar i{display:block;height:100%;width:0;border-radius:9px;transition:width .6s,background .6s}
.tgs{display:grid;grid-template-columns:1fr 1fr;gap:10px;margin-top:10px}
.tg{display:flex;align-items:center;justify-content:space-between;gap:10px;padding:11px 12px;border-radius:14px;background:#0c142c;border:1px solid var(--bd)}
.tg b{font-size:.85rem;display:block}.tg span{font-size:.7rem;color:var(--mut)}
.sw{position:relative;width:52px;height:30px;flex:none}
.sw input{opacity:0;width:0;height:0}
.sl{position:absolute;inset:0;background:#334155;border-radius:30px;transition:.25s;cursor:pointer}
.sl:before{content:"";position:absolute;width:24px;height:24px;left:3px;top:3px;background:#fff;border-radius:50%;transition:.25s}
.sw input:checked+.sl{background:linear-gradient(90deg,#22c55e,#4ade80)}
.sw.pm input:checked+.sl{background:linear-gradient(90deg,#0ea5e9,#38bdf8)}
.sw input:checked+.sl:before{transform:translateX(22px)}
.sw input:disabled+.sl{opacity:.35;cursor:not-allowed}
.pulse{animation:pl 1s infinite}@keyframes pl{50%{opacity:.4}}
.tabs{display:flex;gap:6px;flex-wrap:wrap;margin:10px 0}
.tab{background:#121a33;border:1px solid var(--bd);color:#cbd5ff;border-radius:99px;padding:7px 14px;font-size:.78rem;cursor:pointer;margin:0}
.tab.on{background:linear-gradient(90deg,#8b5cf6,#38bdf8);border-color:transparent;color:#fff;font-weight:700}
.tiles{display:grid;grid-template-columns:repeat(auto-fit,minmax(125px,1fr));gap:10px;margin-bottom:12px}
.tile{padding:12px}.tile .tv{font-size:1.25rem;font-weight:800;margin-top:3px}
.ch{width:100%;height:190px;background:#0a1128;border-radius:10px;margin-top:8px;touch-action:pan-y}
.dot{display:inline-block;width:9px;height:9px;border-radius:50%;margin-right:6px}
.row{display:flex;justify-content:space-between;font-size:.8rem;color:#cbd5ff;padding:4px 0;border-bottom:1px dashed #1e2a52}
button{background:#26335f;color:#f8fafc;border:0;border-radius:10px;padding:10px 14px;font-size:.8rem;cursor:pointer;margin:6px 6px 0 0}
button:active{background:#3b4a80}
button.pri{background:linear-gradient(90deg,#8b5cf6,#38bdf8);font-weight:700}
button.grn{background:linear-gradient(90deg,#16a34a,#4ade80);color:#052e16;font-weight:700}
input[type=range]{width:100%;margin-top:8px;accent-color:#38bdf8}
.warn{font-size:.75rem;color:#fbbf24;margin-top:8px;display:none}
.ev{font-size:.8rem;padding:6px 0;border-bottom:1px dashed #1e2a52;color:#cbd5ff}
#tip{position:fixed;display:none;z-index:9;background:#000c;border:1px solid var(--bd);padding:6px 9px;border-radius:8px;font-size:.72rem;pointer-events:none}
#toast{position:fixed;left:50%;bottom:20px;transform:translate(-50%,40px);opacity:0;background:#16a34a;color:#fff;padding:9px 18px;border-radius:99px;font-size:.8rem;transition:.3s;z-index:10}
#toast.show{opacity:1;transform:translate(-50%,0)}
.al{margin-bottom:8px;padding:10px 14px;border-radius:12px;font-size:.82rem;background:#78350f66;border:1px solid #f59e0b;color:#fde68a}
.al.bad{background:#7f1d1d77;border-color:#f87171;color:#fecaca}
select{width:100%;margin-top:6px;background:#0c142c;color:#f1f5ff;border:1px solid var(--bd);border-radius:10px;padding:9px;font-size:.85rem}
@media(max-width:480px){.tgs{grid-template-columns:1fr}.v{font-size:1.6rem}}
</style>
</head>
<body>
<div class="wrap">
  <h1>&#127793; Plant Monitor</h1>
  <div class="sub" id="sub">connecting...</div>
  <div class="chips">
    <span class="chip" id="cWifi">Wi-Fi --</span>
    <span class="chip" id="cTime">Clock --</span>
    <span class="chip" id="cLog">Logging --</span>
    <span class="chip" id="cMon">Monitored --</span>
  </div>

  <div id="alerts"></div>

  <div class="cards">
    <div class="card" style="--c:linear-gradient(90deg,#0ea5e9,#22d3ee)">
      <div class="flex">
        <div>
          <div class="t">&#128167; Soil Moisture</div>
          <div class="v m" id="vm">-- %</div>
          <div class="s"><span id="badge" class="badge b-good">--</span></div>
        </div>
        <svg class="ring" viewBox="0 0 80 80"><circle cx="40" cy="40" r="34" fill="none" stroke="#1e2a52" stroke-width="9"/><circle id="rg" cx="40" cy="40" r="34" fill="none" stroke="#38bdf8" stroke-width="9" stroke-linecap="round" stroke-dasharray="213.6" stroke-dashoffset="213.6" transform="rotate(-90 40 40)" style="transition:stroke-dashoffset .6s,stroke .6s"/></svg>
      </div>
    </div>
    <div class="card" style="--c:linear-gradient(90deg,#facc15,#fb923c)">
      <div class="t">&#9728;&#65039; Ambient Light</div>
      <div class="v l" id="vl">-- lx</div>
      <div class="s" id="ll">--</div>
    </div>
    <div class="card" style="--c:linear-gradient(90deg,#fb7185,#f97316)">
      <div class="t">&#127777;&#65039; Temperature</div>
      <div class="v temp" id="vt">-- &deg;C</div>
    </div>
    <div class="card" style="--c:linear-gradient(90deg,#c084fc,#f472b6)">
      <div class="t">&#128168; Air Humidity</div>
      <div class="v hum" id="vh">-- %</div>
    </div>
    <div class="card" style="--c:linear-gradient(90deg,#2dd4bf,#38bdf8)">
      <div class="t">&#127811; VPD (plant stress)</div>
      <div class="v" style="color:#2dd4bf" id="vv">-- kPa</div>
      <div class="s" id="vvl">--</div>
    </div>
    <div class="card" style="--c:linear-gradient(90deg,#fde047,#fb923c)">
      <div class="t">&#9728;&#65039; Bright light today</div>
      <div class="v l" id="vb">--</div>
      <div class="s">time above 1000 lx</div>
    </div>
    <div class="card full" style="--c:linear-gradient(90deg,#4ade80,#a3e635,#fbbf24,#f87171)">
      <div class="flex"><div class="t">&#10084;&#65039; Plant Health Score</div><div><b id="hsv" style="font-size:1.4rem">--</b> <span class="s" id="hsl"></span></div></div>
      <div class="bar"><i id="hsb"></i></div>
    </div>
  </div>

  <div class="box">
    <div class="t">Controls</div>
    <div class="tgs">
      <div class="tg"><div><b>&#128703; Water Pump</b><span id="pst">Idle</span></div><label class="sw pm"><input type="checkbox" id="tPump" onchange="ctl('pump',this.checked)"><span class="sl"></span></label></div>
      <div class="tg"><div><b>&#129302; Auto-water</b><span>Pump when soil is dry</span></div><label class="sw"><input type="checkbox" id="tAuto" onchange="ctl('auto',this.checked)"><span class="sl"></span></label></div>
      <div class="tg"><div><b>&#128448;&#65039; 12V Fan</b><span id="fst">Auto/Manual</span></div><label class="sw pm"><input type="checkbox" id="tFan" onchange="ctl('fan',this.checked)"><span class="sl"></span></label></div>
      <div class="tg"><div><b>&#127811; Servo Vent</b><span id="vst">Open/Close</span></div><label class="sw pm"><input type="checkbox" id="tVent" onchange="ctl('vent',this.checked)"><span class="sl"></span></label></div>
      <div class="tg"><div><b>&#128190; Data Logging</b><span>Hour / day / month history</span></div><label class="sw"><input type="checkbox" id="tLog" onchange="ctl('log',this.checked)"><span class="sl"></span></label></div>
      <div class="tg"><div><b>&#128250; OLED Display</b><span>Turn screen on / off</span></div><label class="sw"><input type="checkbox" id="tOled" onchange="ctl('oled',this.checked)"><span class="sl"></span></label></div>
      <div class="tg"><div><b>&#128241; Phone Alerts</b><span>Telegram</span></div><label class="sw"><input type="checkbox" id="tSms" onchange="ctl('sms',this.checked)"><span class="sl"></span></label></div>
    </div>
    <div class="warn" id="wPump">Pump is disabled in firmware (ENABLE_PUMP 0).</div>
  </div>

  <div class="box">
    <div class="t">History &amp; Reports</div>
    <div class="tabs" id="tabs">
      <button class="tab on" data-r="live">Live 30m</button>
      <button class="tab" data-r="24h">24 Hours</button>
      <button class="tab" data-r="7d">7 Days</button>
      <button class="tab" data-r="30d">30 Days</button>
      <button class="tab" data-r="12m">12 Months</button>
    </div>
    <div class="s" id="rlab" style="margin-bottom:10px">--</div>
    <div class="tiles" id="tiles"></div>
    <button class="pri" onclick="rep()">&#128196; Download PDF Report</button>
    <button onclick="csv()">&#128202; Export CSV</button>
  </div>

  <div class="box"><div class="t"><span class="dot" style="background:#38bdf8"></span>Soil moisture</div><svg class="ch" id="c1" viewBox="0 0 500 190"></svg></div>
  <div class="box"><div class="t"><span class="dot" style="background:#facc15"></span>Light (log scale)</div><svg class="ch" id="c2" viewBox="0 0 500 190"></svg></div>
  <div class="box"><div class="t"><span class="dot" style="background:#fb7185"></span>Temperature</div><svg class="ch" id="c3" viewBox="0 0 500 190"></svg></div>
  <div class="box"><div class="t"><span class="dot" style="background:#c084fc"></span>Air humidity</div><svg class="ch" id="c4" viewBox="0 0 500 190"></svg></div>

  <div class="box">
    <div class="t">&#128703; Watering log</div>
    <div id="evl" style="margin-top:6px"><div class="s">No watering yet</div></div>
  </div>

  <div class="box">
    <div class="t" style="margin-bottom:6px">Device</div>
    <div class="row"><span>Moisture min / max (since boot)</span><span id="smm">--</span></div>
    <div class="row"><span>Light min / max (since boot)</span><span id="sll">--</span></div>
    <div class="row"><span>Raw ADC (dry / wet cal)</span><span id="sraw">--</span></div>
    <div class="row"><span>Monitoring since</span><span id="ssince">--</span></div>
    <div class="row"><span>Total monitored</span><span id="smon">--</span></div>
    <div class="row"><span>Uptime</span><span id="sup">--</span></div>
    <div class="row"><span>Watering runs (total)</span><span id="spump">--</span></div>
    <div class="t" style="margin-top:14px">Settings</div>
    <div class="s" style="margin-top:8px">Plant profile (sets dry threshold, ideal temperature, humidity, light)</div>
    <select id="prof" onchange="setProf(this.value)"></select>
    <div class="s" style="margin-top:8px">Dry threshold: <b id="thrv">30</b> %</div>
    <input type="range" id="thr" min="5" max="80" value="30">
    <div style="margin-top:6px">
      <button onclick="cal('air')">Calibrate DRY (sensor in air)</button>
      <button onclick="cal('water')">Calibrate WET (sensor in water)</button>
      <button onclick="clr()" style="background:#7f1d1d">Clear history</button>
    </div>
    <div class="warn" id="wCal">Sensor reading is outside the calibration range - please recalibrate DRY and WET.</div>
  </div>
</div>
<div id="tip"></div><div id="toast"></div>

<script>
const $=id=>document.getElementById(id);
const MON=['Jan','Feb','Mar','Apr','May','Jun','Jul','Aug','Sep','Oct','Nov','Dec'];
let D=null,RANGE='live',S=null,EV={e:[],total:0},busy=false,lastRef=0;
const RG={live:{r:'live',win:1800,lab:'Live - last 30 min'},'24h':{r:'5m',win:86400,lab:'Last 24 hours'},'7d':{r:'1h',win:604800,lab:'Last 7 days'},'30d':{r:'1h',win:2592000,lab:'Last 30 days'},'12m':{r:'1d',win:31536000,lab:'Last 12 months'}};
const p2=n=>(n<10?'0':'')+n;
function fmtT(t,rg){const d=new Date(t*1000);
  if(rg==='live'||rg==='24h')return p2(d.getHours())+':'+p2(d.getMinutes());
  if(rg==='12m')return MON[d.getMonth()]+" '"+p2(d.getFullYear()%100);
  return d.getDate()+' '+MON[d.getMonth()];}
function fmtFull(t){const d=new Date(t*1000);return d.getDate()+' '+MON[d.getMonth()]+' '+d.getFullYear()+' '+p2(d.getHours())+':'+p2(d.getMinutes());}
function fmtDur(s){const d=Math.floor(s/86400),h=Math.floor(s%86400/3600),m=Math.floor(s%3600/60);return (d?d+'d ':'')+h+'h '+m+'m';}
function toast(t){const e=$('toast');e.innerText=t;e.classList.add('show');clearTimeout(toast.t);toast.t=setTimeout(()=>e.classList.remove('show'),3000);}
function luxLabel(x){if(x<10)return 'Dark';if(x<200)return 'Dim / indoor shade';if(x<1000)return 'Indoor bright';if(x<10000)return 'Bright / cloudy day';return 'Direct sunlight';}
const logY=v=>Math.min(1,Math.log10(Math.max(v,0)+1)/5);
const luxFmt=v=>v>=1000?(v/1000)+'k':v;
const cl01=v=>Math.max(0,Math.min(1,v));
const st=a=>a.length?{min:Math.min(...a),max:Math.max(...a),avg:a.reduce((x,y)=>x+y,0)/a.length}:{min:0,max:0,avg:0};
function nice(a,def,pad,minSpan){let lo=def[0],hi=def[1];
  if(a.length){lo=Math.floor(Math.min(...a)-pad);hi=Math.ceil(Math.max(...a)+pad);
    if(hi-lo<minSpan){const m=(hi+lo)/2;lo=Math.floor(m-minSpan/2);hi=lo+minSpan;}}
  const ticks=[];for(let i=0;i<=4;i++)ticks.push(Math.round(lo+(hi-lo)*i/4));
  return{lo:lo,hi:hi,ticks:ticks};}
function health(d){let s=100;
  if(d.moisture<d.dry)s-=Math.min(45,(d.dry-d.moisture)*1.5);else if(d.moisture>85)s-=Math.min(30,(d.moisture-85)*2);
  if(d.temp>d.tmax)s-=Math.min(25,(d.temp-d.tmax)*3);else if(d.temp<d.tmin)s-=Math.min(25,(d.tmin-d.temp)*3);
  if(d.hum<d.hmin)s-=Math.min(10,(d.hmin-d.hum)/2);else if(d.hum>d.hmax)s-=Math.min(10,(d.hum-d.hmax)/2);
  s=Math.max(0,Math.round(s));
  return{score:s,label:s>=85?'Excellent':s>=65?'Good':s>=40?'Needs care':'Critical',color:s>=85?'#4ade80':s>=65?'#a3e635':s>=40?'#fbbf24':'#f87171'};}

/* ---------- data ---------- */
async function loadSeries(rg){
  const c=RG[rg];const j=await (await fetch('/api/log?r='+c.r)).json();
  const to=Math.floor(Date.now()/1000),from=to-c.win;
  const o={t:[],m:[],l:[],c:[],h:[],iv:j.iv,from:from,to:to,rg:rg};
  for(let i=0;i<j.n;i++){let t=j.t[i];
    if(rg==='live'&&!t)t=to-(j.n-1-i)*j.iv;
    if(!t||t<from)continue;
    o.t.push(t);o.m.push(j.m[i]);o.l.push(j.l[i]);o.c.push(j.c[i]/10);o.h.push(j.h[i]);}
  return o;}

async function loadData(){
  try{
    const d=await (await fetch('/api/data?t='+Date.now())).json();D=d;
    const drift=Math.abs(d.epoch-Date.now()/1000);
    if(!d.tok||(d.ap&&drift>120))fetch('/api/settime?t='+Math.floor(Date.now()/1000));
    $('vm').innerText=d.moisture+' %';
    $('vl').innerText=d.lux.toFixed(1)+' lx';
    $('vt').innerText=d.temp.toFixed(1)+' \u00B0C';
    $('vh').innerText=d.hum.toFixed(1)+' %';
    $('ll').innerText=luxLabel(d.lux);
    $('vv').innerText=d.vpd.toFixed(2)+' kPa';
    $('vvl').innerText=d.vpd<0.4?'Too humid - low transpiration':d.vpd<0.8?'Low - fine for seedlings':d.vpd<1.2?'Ideal for growth':d.vpd<1.6?'High - watch watering':'Very high - plant stress';
    $('vb').innerText=fmtDur(d.bright);
    showAlerts(d.al);
    if(document.activeElement!==$('prof'))$('prof').value=d.prof;
    const b=$('badge'),rg=$('rg');let col='#4ade80';
    if(d.moisture<d.dry){b.className='badge b-dry';b.innerText='Dry - needs water';col='#f87171';}
    else if(d.moisture>80){b.className='badge b-wet';b.innerText='Wet';col='#38bdf8';}
    else{b.className='badge b-good';b.innerText='Good';}
    rg.style.stroke=col;rg.style.strokeDashoffset=213.6*(1-d.moisture/100);
    const hs=health(d);$('hsv').innerText=hs.score+'/100';$('hsl').innerText=hs.label;$('hsv').style.color=hs.color;
    const hb=$('hsb');hb.style.width=hs.score+'%';hb.style.background=hs.color;
    $('sub').innerText=d.ip+'  |  offline mode';
    $('cWifi').innerText=d.ap?'\uD83D\uDCF6 Hotspot mode':'\uD83D\uDCF6 '+d.rssi+' dBm';
    $('cTime').innerText=d.tok?'\uD83D\uDD52 '+fmtFull(d.epoch):'\uD83D\uDD52 clock syncing...';
    $('cLog').innerText=d.log?'\uD83D\uDCBE Logging ON':'\uD83D\uDCBE Logging OFF';
    $('cMon').innerText='\u23F1 Monitored '+fmtDur(d.mon);
    $('smm').innerText=d.minM+'% / '+d.maxM+'%';
    $('sll').innerText=d.minL.toFixed(0)+' / '+d.maxL.toFixed(0)+' lx';
    $('sraw').innerText=d.raw+'  ('+d.air+' / '+d.water+')';
    $('ssince').innerText=d.since?fmtFull(d.since):'--';
    $('smon').innerText=fmtDur(d.mon);
    $('sup').innerText=fmtDur(d.up);
    $('spump').innerText=d.pumps;
    $('tPump').checked=d.pump;$('tAuto').checked=d.auto;$('tLog').checked=d.log;$('tOled').checked=d.oled;$('tSms').checked=d.sms;
    $('tFan').checked = d.fan;
    $('tVent').checked = d.vent;
    $('tPump').disabled=!d.ten;$('tAuto').disabled=!d.ten;$('wPump').style.display=d.ten?'none':'block';
    const ps=$('pst');ps.innerText=d.pump?'Running...':'Idle';ps.className=d.pump?'pulse':'';
    const lo=Math.min(d.air,d.water),hi=Math.max(d.air,d.water);
    $('wCal').style.display=(d.raw>hi+100||d.raw<lo-100)?'block':'none';
    if(document.activeElement!==$('thr')){$('thr').value=d.dry;$('thrv').innerText=d.dry;}
  }catch(e){$('sub').innerText='connection lost...';}
}

async function ctl(k,v){
  try{await fetch('/api/control?'+k+'='+(v?1:0));toast((k==='pump'?'Pump ':k==='auto'?'Auto-water ':k==='fan'?'12V Fan ':k==='vent'?'Servo Vent ':k==='log'?'Logging ':k==='sms'?'Phone alerts ':'OLED ')+(v?'ON':'OFF'));}catch(e){toast('Failed');}
  loadData();}
async function cal(t){
  const r=await (await fetch('/api/calibrate?type='+t)).json();
  toast(r.ok?'Saved '+t+' = '+r.value:'Rejected: dry and wet values too close');loadData();}
$('thr').oninput=()=>{$('thrv').innerText=$('thr').value;};
$('thr').onchange=()=>fetch('/api/config?dry='+$('thr').value);

/* ---------- charts ---------- */
const CH={};
function chart(id,S,key,col,yf,ticks,fmt,u){
  const W=500,H=190,L=42,R=10,T=10,B=24,iw=W-L-R,ih=H-T-B;
  const xp=t=>L+iw*(t-S.from)/(S.to-S.from),yp=v=>T+ih*(1-yf(v));
  let s='<defs><linearGradient id="g'+id+'" x1="0" y1="0" x2="0" y2="1"><stop offset="0" stop-color="'+col+'" stop-opacity=".5"/><stop offset="1" stop-color="'+col+'" stop-opacity="0"/></linearGradient></defs>';
  ticks.forEach(tk=>{const y=yp(tk);
    s+='<line x1="'+L+'" y1="'+y+'" x2="'+(W-R)+'" y2="'+y+'" stroke="#27325a" stroke-dasharray="3 4"/>';
    s+='<text x="'+(L-5)+'" y="'+(y+3)+'" fill="#8b9ac8" font-size="9" text-anchor="end">'+fmt(tk)+'</text>';});
  for(let i=0;i<=4;i++){const t=S.from+(S.to-S.from)*i/4;
    s+='<text x="'+xp(t)+'" y="'+(H-7)+'" fill="#8b9ac8" font-size="9" text-anchor="'+(i==0?'start':i==4?'end':'middle')+'">'+fmtT(t,S.rg)+'</text>';}
  const xs=S.t,ys=S[key];let seg=[],segs=[];
  for(let i=0;i<xs.length;i++){if(i&&xs[i]-xs[i-1]>S.iv*3){segs.push(seg);seg=[];}seg.push(i);}
  if(seg.length)segs.push(seg);
  segs.forEach(sg=>{
    const pts=sg.map(i=>xp(xs[i]).toFixed(1)+','+yp(ys[i]).toFixed(1));
    if(sg.length>1){
      s+='<polygon fill="url(#g'+id+')" points="'+pts.join(' ')+' '+xp(xs[sg[sg.length-1]]).toFixed(1)+','+(T+ih)+' '+xp(xs[sg[0]]).toFixed(1)+','+(T+ih)+'"/>';
      s+='<polyline fill="none" stroke="'+col+'" stroke-width="2.2" stroke-linejoin="round" points="'+pts.join(' ')+'"/>';
    }else s+='<circle cx="'+xp(xs[sg[0]]).toFixed(1)+'" cy="'+yp(ys[sg[0]]).toFixed(1)+'" r="3" fill="'+col+'"/>';
  });
  if(!xs.length)s+='<text x="'+(W/2)+'" y="'+(H/2)+'" fill="#8b9ac8" font-size="12" text-anchor="middle">Collecting data...</text>';
  const svg=$(id);svg.innerHTML=s;CH[id]={xs:xs,ys:ys,xp:xp,u:u};
  svg.onmousemove=hover;svg.ontouchmove=hover;svg.onmouseleave=svg.ontouchend=()=>{$('tip').style.display='none';};
}
function hover(e){
  const svg=e.currentTarget,c=CH[svg.id];if(!c||!c.xs.length)return;
  const r=svg.getBoundingClientRect(),pt=e.touches?e.touches[0]:e;
  const x=(pt.clientX-r.left)/r.width*500;let b=0,bd=1e9;
  c.xs.forEach((t,i)=>{const d=Math.abs(c.xp(t)-x);if(d<bd){bd=d;b=i;}});
  const tip=$('tip');tip.style.display='block';
  tip.innerHTML='<b>'+c.u(c.ys[b])+'</b><br>'+fmtFull(c.xs[b]);
  tip.style.left=Math.min(pt.clientX+12,innerWidth-150)+'px';tip.style.top=(r.top+8)+'px';}

function draw(){
  const c=RG[RANGE];$('rlab').innerText=c.lab+'  -  '+S.t.length+' samples';
  chart('c1',S,'m','#38bdf8',v=>v/100,[0,25,50,75,100],v=>v+'%',v=>v+' %');
  chart('c2',S,'l','#facc15',logY,[0,10,100,1000,10000,100000],luxFmt,v=>v+' lx');
  const tr=nice(S.c,[15,40],2,10);
  chart('c3',S,'c','#fb7185',v=>cl01((v-tr.lo)/(tr.hi-tr.lo)),tr.ticks,v=>v+'\u00B0',v=>v.toFixed(1)+' \u00B0C');
  chart('c4',S,'h','#c084fc',v=>v/100,[0,25,50,75,100],v=>v+'%',v=>v.toFixed(0)+' %');
  const sm=st(S.m),sl=st(S.l),sc=st(S.c),sh=st(S.h);
  const dry=D?Math.round(100*S.m.filter(v=>v<D.dry).length/Math.max(1,S.m.length)):0;
  const tl=(n,col,a,u,dp)=>'<div class="tile" style="--c:'+col+'"><div class="t">'+n+'</div><div class="tv">'+a.avg.toFixed(dp)+u+'</div><div class="s">min '+a.min.toFixed(dp)+' / max '+a.max.toFixed(dp)+'</div></div>';
  $('tiles').innerHTML=tl('Moisture avg','#38bdf8',sm,'%',0)+tl('Light avg','#facc15',sl,' lx',0)+tl('Temp avg','#fb7185',sc,'\u00B0C',1)+tl('Humidity avg','#c084fc',sh,'%',0)+
    '<div class="tile" style="--c:#f87171"><div class="t">Time dry</div><div class="tv">'+dry+'%</div><div class="s">below threshold</div></div>';
}
function drawEv(){
  const e=(EV.e||[]).slice().reverse().slice(0,10);
  $('evl').innerHTML=e.length?e.map(x=>'<div class="ev">'+(x.t?fmtFull(x.t):'time n/a')+' &middot; '+(x.s?'\uD83E\uDD16 Auto':'\uD83D\uDC46 Manual')+' &middot; '+x.d+' s</div>').join(''):'<div class="s">No watering yet</div>';}
async function refresh(){
  if(busy)return;busy=true;
  try{S=await loadSeries(RANGE);draw();EV=await (await fetch('/api/events')).json();drawEv();}catch(e){}
  busy=false;lastRef=Date.now();}
document.querySelectorAll('.tab').forEach(b=>b.onclick=()=>{
  document.querySelectorAll('.tab').forEach(x=>x.classList.remove('on'));b.classList.add('on');RANGE=b.dataset.r;refresh();});

/* ---------- downloads ---------- */
function dl(blob,name){const a=document.createElement('a');a.href=URL.createObjectURL(blob);a.download=name;document.body.appendChild(a);a.click();setTimeout(()=>{URL.revokeObjectURL(a.href);a.remove();},4000);}
const stamp=()=>new Date().toLocaleString('sv-SE').replace(/[: ]/g,'-');
function csv(){
  if(!S||!S.t.length){toast('No data in this range yet');return;}
  let s='Time,Moisture %,Light lx,Temperature C,Humidity %\n';
  S.t.forEach((t,i)=>{s+=new Date(t*1000).toLocaleString('sv-SE')+','+S.m[i]+','+S.l[i]+','+S.c[i].toFixed(1)+','+S.h[i]+'\n';});
  dl(new Blob([s],{type:'text/csv'}),'PlantData_'+RANGE+'_'+stamp()+'.csv');}

/* ---------- tiny PDF writer (no libraries) ---------- */
function PDF(){
  const pages=[];let cur;
  const col=h=>{h=h.replace('#','');return [0,2,4].map(i=>(parseInt(h.substr(i,2),16)/255).toFixed(3)).join(' ');};
  const esc=s=>String(s).replace(/[^\x20-\x7E]/g,'?').replace(/([\\()])/g,'\\$1');
  const f=n=>(+n).toFixed(1),Y=y=>842-y;
  return{
    page(){cur=[];pages.push(cur);},
    go(i){cur=pages[i];},
    n(){return pages.length;},
    rect(x,y,w,h,c){cur.push(col(c)+' rg '+f(x)+' '+f(Y(y+h))+' '+f(w)+' '+f(h)+' re f');},
    text(x,y,sz,s,b,c){cur.push('BT /F'+(b?2:1)+' '+sz+' Tf '+col(c||'#0f172a')+' rg '+f(x)+' '+f(Y(y))+' Td ('+esc(s)+') Tj ET');},
    line(x1,y1,x2,y2,c,w){cur.push(col(c)+' RG '+(w||1)+' w '+f(x1)+' '+f(Y(y1))+' m '+f(x2)+' '+f(Y(y2))+' l S');},
    poly(pts,c,w){if(pts.length<2)return;cur.push(col(c)+' RG '+(w||1)+' w 1 j '+pts.map((p,i)=>f(p[0])+' '+f(Y(p[1]))+(i?' l':' m')).join(' ')+' S');},
    build(){
      const objs=[];const n=pages.length;
      objs[1]='<< /Type /Catalog /Pages 2 0 R >>';
      objs[2]='<< /Type /Pages /Kids ['+pages.map((_,i)=>(5+2*i)+' 0 R').join(' ')+'] /Count '+n+' >>';
      objs[3]='<< /Type /Font /Subtype /Type1 /BaseFont /Helvetica /Encoding /WinAnsiEncoding >>';
      objs[4]='<< /Type /Font /Subtype /Type1 /BaseFont /Helvetica-Bold /Encoding /WinAnsiEncoding >>';
      pages.forEach((c,i)=>{const s=c.join('\n');
        objs[5+2*i]='<< /Type /Page /Parent 2 0 R /MediaBox [0 0 595 842] /Resources << /Font << /F1 3 0 R /F2 4 0 R >> >> /Contents '+(6+2*i)+' 0 R >>';
        objs[6+2*i]='<< /Length '+s.length+' >>\nstream\n'+s+'\nendstream';});
      let out='%PDF-1.4\n';const off=[];
      for(let i=1;i<objs.length;i++){off[i]=out.length;out+=i+' 0 obj\n'+objs[i]+'\nendobj\n';}
      const xr=out.length;out+='xref\n0 '+objs.length+'\n0000000000 65535 f \n';
      for(let i=1;i<objs.length;i++)out+=String(off[i]).padStart(10,'0')+' 00000 n \n';
      out+='trailer\n<< /Size '+objs.length+' /Root 1 0 R >>\nstartxref\n'+xr+'\n%%EOF';
      return out;}
  };
}

function rep(){
  if(!S||!S.t.length||!D){toast('No data in this range yet');return;}
  const P=PDF(),c=RG[RANGE];
  const sm=st(S.m),sl=st(S.l),sc=st(S.c),sh=st(S.h),hs=health(D);
  const N=S.t.length;
  const dryPct=Math.round(100*S.m.filter(v=>v<D.dry).length/N);
  const wetPct=Math.round(100*S.m.filter(v=>v>85).length/N);
  const ev=(EV.e||[]).filter(x=>x.t>=S.from&&x.t<=S.to);
  let y=120;
  P.page();
  P.rect(0,0,595,92,'#1e1b4b');P.rect(0,92,595,5,'#38bdf8');P.rect(0,97,595,3,'#a78bfa');
  P.text(36,40,24,'Plant Monitor Report',1,'#ffffff');
  P.text(36,60,10,'Period: '+c.lab+'   ('+fmtFull(S.from)+'  to  '+fmtFull(S.to)+')',0,'#c7d2fe');
  P.text(36,76,9,'Generated: '+new Date().toLocaleString()+'     Device: '+D.ip+'     Samples: '+N,0,'#a5b4fc');
  const sec=(t,k)=>{P.rect(36,y-11,4,15,k);P.text(46,y,13,t,1,'#1e1b4b');y+=20;};
  const ensure=h=>{if(y+h>805){P.page();P.rect(0,0,595,26,'#1e1b4b');P.text(36,17,10,'Plant Monitor Report - '+c.lab,1,'#ffffff');y=52;}};

  sec('Summary','#38bdf8');
  P.rect(36,y-12,523,18,'#e0e7ff');
  ['Metric','Minimum','Average','Maximum','Last value'].forEach((h,i)=>P.text([46,200,285,370,455][i],y,9,h,1,'#312e81'));
  y+=20;
  [['Soil moisture (%)',sm,0,D.moisture,'#0ea5e9'],['Light (lx)',sl,0,D.lux,'#eab308'],['Temperature (C)',sc,1,D.temp,'#f43f5e'],['Humidity (%)',sh,0,D.hum,'#a855f7']].forEach((r,i)=>{
    if(i%2)P.rect(36,y-12,523,18,'#f8fafc');P.rect(36,y-12,4,18,r[4]);
    P.text(46,y,10,r[0]);P.text(200,y,10,r[1].min.toFixed(r[2]));P.text(285,y,10,r[1].avg.toFixed(r[2]));P.text(370,y,10,r[1].max.toFixed(r[2]));P.text(455,y,10,Number(r[3]).toFixed(r[2]));y+=18;});
  y+=6;
  P.text(46,y,9,'Dry threshold: '+D.dry+'%   |   Time below threshold: '+dryPct+'%   |   Total monitored: '+fmtDur(D.mon)+(D.since?'   |   Since: '+fmtDate(D.since):''),0,'#475569');y+=13;
  P.text(46,y,9,'Profile: '+D.pn+'  |  Ideal temp '+D.tmin+'-'+D.tmax+' C  |  Humidity '+D.hmin+'-'+D.hmax+'%  |  VPD now '+D.vpd.toFixed(2)+' kPa  |  Bright light today '+fmtDur(D.bright),0,'#475569');y+=22;

  sec('Plant health','#4ade80');
  P.text(46,y,11,'Health score: '+hs.score+' / 100  -  '+hs.label,1);y+=12;
  P.rect(46,y,300,9,'#e2e8f0');P.rect(46,y,3*hs.score,9,hs.color);y+=28;

  const F=[];
  F.push('Soil moisture averaged '+sm.avg.toFixed(0)+'% (range '+sm.min+'-'+sm.max+'%) against a dry threshold of '+D.dry+'%.');
  if(dryPct>30)F.push('Soil was below the dry threshold for '+dryPct+'% of the period. Water more often, or enable Auto-water and check the pump.');
  else if(dryPct>0)F.push('Soil dipped below the dry threshold for '+dryPct+'% of the period - acceptable, but watch the trend.');
  else F.push('Soil stayed above the dry threshold for the whole period - watering looks good.');
  if(wetPct>30)F.push('Soil was very wet (above 85%) for '+wetPct+'% of the period. Risk of root rot - reduce watering and check drainage.');
  if(sl.max<D.lmin)F.push('Peak light was only '+sl.max.toFixed(0)+' lx - below the '+D.lmin+' lx daytime target for '+D.pn+'. Move closer to a window or add a grow light.');
  else if(sl.max>20000)F.push('Peak light reached '+sl.max.toFixed(0)+' lx (direct sun). Check that the leaves are not scorching in the afternoon.');
  else F.push('Light levels looked reasonable (average '+sl.avg.toFixed(0)+' lx, peak '+sl.max.toFixed(0)+' lx).');
  if(sc.max>D.tmax)F.push('Temperature peaked at '+sc.max.toFixed(1)+' C - heat stress risk. Provide shade or ventilation in the afternoon.');
  else if(sc.min<D.tmin)F.push('Temperature fell to '+sc.min.toFixed(1)+' C - cold stress risk for tropical plants.');
  else F.push('Temperature stayed in a comfortable range ('+sc.min.toFixed(1)+' - '+sc.max.toFixed(1)+' C).');
  if(sh.avg<D.hmin)F.push('Air humidity was low (avg '+sh.avg.toFixed(0)+'%). Consider misting or a pebble tray.');
  else if(sh.avg>D.hmax)F.push('Air humidity was very high (avg '+sh.avg.toFixed(0)+'%). Improve airflow to reduce fungal risk.');
  else F.push('Air humidity was healthy (avg '+sh.avg.toFixed(0)+'%).');
  F.push('Watering events in this period: '+ev.length+(ev.length?' (total pump time '+ev.reduce((a,x)=>a+x.d,0)+' s).':'.'));
  if(D.raw>Math.max(D.air,D.water)+100||D.raw<Math.min(D.air,D.water)-100)F.push('NOTE: the moisture sensor is reading outside its calibration range. Recalibrate DRY and WET before trusting the moisture values.');

  sec('Findings & recommendations','#fbbf24');
  const wrap=(s,n)=>{const w=s.split(' ');let l='',o=[];w.forEach(x=>{if((l+' '+x).length>n){o.push(l);l=x;}else l=l?l+' '+x:x;});if(l)o.push(l);return o;};
  F.forEach(t=>{const ls=wrap(t,98);ensure(ls.length*13+4);ls.forEach((l,i)=>{P.text(i?56:46,y,10,(i?'':'- ')+l);y+=13;});y+=3;});
  y+=8;

  const tr=nice(S.c,[15,40],2,10);
  const pc=(title,key,color,yf,ticks,fmt)=>{
    ensure(190);const x=36,w=523,h=165;
    P.text(x,y,11,title,1,'#1e1b4b');y+=6;P.rect(x,y,w,h,'#f8fafc');
    const L=x+40,R=x+w-10,T=y+10,B=y+h-22;
    ticks.forEach(tk=>{const yy=T+(B-T)*(1-yf(tk));P.line(L,yy,R,yy,'#e2e8f0',.6);P.text(L-6-String(fmt(tk)).length*4.4,yy+3,8,fmt(tk),0,'#64748b');});
    for(let i=0;i<=4;i++){const t=S.from+(S.to-S.from)*i/4,xx=L+(R-L)*i/4;P.text(xx-(i==4?24:i?12:0),B+14,8,fmtT(t,RANGE),0,'#64748b');}
    const stride=Math.max(1,Math.ceil(N/250));let seg=[],lt=0;
    for(let i=0;i<N;i+=stride){const t=S.t[i];
      if(lt&&t-lt>S.iv*3*stride){P.poly(seg,color,1.5);seg=[];}
      seg.push([L+(R-L)*(t-S.from)/(S.to-S.from),T+(B-T)*(1-yf(S[key][i]))]);lt=t;}
    P.poly(seg,color,1.5);
    y+=h+16;};
  pc('Soil moisture (%)','m','#0ea5e9',v=>v/100,[0,25,50,75,100],v=>v+'%');
  pc('Light (lx, log scale)','l','#eab308',logY,[0,10,100,1000,10000,100000],luxFmt);
  pc('Temperature (C)','c','#f43f5e',v=>cl01((v-tr.lo)/(tr.hi-tr.lo)),tr.ticks,v=>String(v));
  pc('Air humidity (%)','h','#a855f7',v=>v/100,[0,25,50,75,100],v=>v+'%');

  ensure(60);sec('Logged samples','#a78bfa');
  const X=[46,150,255,350,450];
  P.rect(36,y-12,523,18,'#e0e7ff');
  ['Time','Moisture %','Light lx','Temp C','Humidity %'].forEach((h,i)=>P.text(X[i],y,9,h,1,'#312e81'));y+=19;
  const sp=Math.max(1,Math.ceil(N/28));let k=0;
  for(let i=0;i<N;i+=sp,k++){ensure(16);
    if(k%2)P.rect(36,y-11,523,14,'#f8fafc');
    P.text(X[0],y,9,fmtFull(S.t[i]));P.text(X[1],y,9,String(S.m[i]));P.text(X[2],y,9,String(S.l[i]));P.text(X[3],y,9,S.c[i].toFixed(1));P.text(X[4],y,9,String(S.h[i]));y+=14;}

  const pn=P.n();
  for(let i=0;i<pn;i++){P.go(i);P.line(36,818,559,818,'#cbd5e1',.5);
    P.text(36,831,8,'Plant Monitor - ESP32 (auto-generated report)',0,'#64748b');P.text(505,831,8,'Page '+(i+1)+' of '+pn,0,'#64748b');}
  dl(new Blob([P.build()],{type:'application/pdf'}),'PlantReport_'+RANGE+'_'+stamp()+'.pdf');
  toast('PDF report created');
}
function fmtDate(t){const d=new Date(t*1000);return d.getDate()+' '+MON[d.getMonth()]+' '+d.getFullYear();}

const PN=['General houseplant','Succulent / Cactus','Tomato / Vegetables','Fern / Tropical','Herbs (Basil, Mint)','Orchid'];
PN.forEach((n,i)=>{const o=document.createElement('option');o.value=i;o.textContent=n;$('prof').appendChild(o);});
const AL={0:'Soil is too dry - water needed',1:'Soil is too wet - reduce watering',2:'Too hot for this plant',3:'Too cold for this plant',4:'Air humidity is too low',5:'Air humidity is too high',6:'Light is too low for daytime',
  8:'DHT22 not responding - check wiring (GPIO4)',9:'Light sensor BH1750 not responding - check I2C wiring',10:'Soil sensor reading invalid - check wiring (GPIO34)'};
function showAlerts(m){let h='';Object.keys(AL).forEach(k=>{if(m&(1<<k))h+='<div class="al'+(k>=8?' bad':'')+'">'+(k>=8?'\u26A0\uFE0F ':'\uD83D\uDD14 ')+AL[k]+'</div>';});$('alerts').innerHTML=h;}
async function setProf(v){await fetch('/api/config?profile='+v);toast('Profile applied');loadData();}
async function clr(){if(!confirm('Delete ALL logged history and counters? This cannot be undone.'))return;await fetch('/api/clear');toast('History cleared');refresh();loadData();}
setInterval(loadData,2000);
setInterval(()=>{if(RANGE==='live'||Date.now()-lastRef>60000)refresh();},10000);
loadData();refresh();
</script>
</body>
</html>
)rawliteral";

// ===================== TIME HELPERS =====================
bool timeValid() { return time(nullptr) > 1700000000; }

// ===================== TIER ENGINE =====================
const Rec& tierAt(const Tier& T, int i) {
  int start = (T.count < T.cap) ? 0 : T.head;
  return T.buf[(start + i) % T.cap];
}
void tierPush(Tier& T, const Rec& r) {
  T.buf[T.head] = r;
  T.head = (T.head + 1) % T.cap;
  if (T.count < T.cap) T.count++;
}
void tierReset(Tier& T) { T.n = 0; T.sM = T.sL = T.sC = T.sH = 0; }

void tierRewrite(Tier& T) {
  File f = LittleFS.open(T.file, "w");
  if (!f) return;
  for (int i = 0; i < T.count; i++) { Rec r = tierAt(T, i); f.write((const uint8_t*)&r, sizeof(Rec)); }
  f.close();
}
void tierFlush(Tier& T) {
  if (!T.file || !T.unsaved) return;
  File f = LittleFS.open(T.file, "a");
  if (!f) return;
  for (int k = T.unsaved; k >= 1; k--) {
    int idx = ((int)T.head - k + T.cap) % T.cap;
    f.write((const uint8_t*)&T.buf[idx], sizeof(Rec));
  }
  size_t sz = f.size();
  f.close();
  T.unsaved = 0;
  if (sz > (size_t)T.cap * 2 * sizeof(Rec)) tierRewrite(T);   
}
void tierLoad(Tier& T) {
  if (!T.file || !LittleFS.exists(T.file)) return;
  File f = LittleFS.open(T.file, "r");
  if (!f) return;
  size_t total = f.size() / sizeof(Rec);
  size_t skip = total > T.cap ? total - T.cap : 0;
  f.seek(skip * sizeof(Rec));
  Rec r;
  while (f.read((uint8_t*)&r, sizeof(Rec)) == sizeof(Rec)) tierPush(T, r);
  f.close();
}
void tierCommit(Tier& T) {
  if (!T.n) return;
  Rec r;
  if (T.useMillis) r.t = timeValid() ? (uint32_t)time(nullptr) : 0;
  else r.t = T.bucket * T.iv - (T.local ? TZ_OFFSET_SEC : 0);
  r.m = (uint8_t)constrain(lroundf(T.sM / T.n), 0, 100);
  r.l = (uint16_t)constrain(lroundf(T.sL / T.n), 0, 65535);
  r.c = (int16_t)lroundf(T.sC / T.n * 10.0f);
  r.h = (uint8_t)constrain(lroundf(T.sH / T.n), 0, 100);
  tierPush(T, r);
  tierReset(T);
  if (T.file) { T.unsaved++; if (T.unsaved >= T.flushEvery) tierFlush(T); }
}
void tierAdd(Tier& T, float m, float l, float c, float h) {
  uint32_t b;
  if (T.useMillis) b = millis() / 1000 / T.iv;
  else {
    if (!timeValid()) return;
    b = (uint32_t)((time(nullptr) + (T.local ? TZ_OFFSET_SEC : 0)) / T.iv);
  }
  if (T.n && b != T.bucket) tierCommit(T);
  if (!T.n) T.bucket = b;
  T.sM += m; T.sL += l; T.sC += c; T.sH += h; T.n++;
}

// ===================== PUMP =====================
#if ENABLE_PUMP
void pumpWrite(bool on) {
  digitalWrite(PUMP_PIN, on ? PUMP_ACTIVE_LEVEL : (PUMP_ACTIVE_LEVEL == HIGH ? LOW : HIGH));
}
#endif

void pumpStart(uint8_t src, unsigned long maxMs) {
#if ENABLE_PUMP
  if (pumpOn) return;
  pumpOn = true; pumpSrc = src; pumpStartMs = millis(); pumpLimitMs = maxMs;
  pumpWrite(true);
#endif
}
void pumpStop() {
#if ENABLE_PUMP
  if (!pumpOn) return;
  pumpWrite(false);
  pumpOn = false; pumpEndMs = millis(); pumpEverRun = true;
  Evt e; e.t = timeValid() ? (uint32_t)time(nullptr) : 0; e.src = pumpSrc; e.sec = (uint16_t)((pumpEndMs - pumpStartMs) / 1000);
  evts[evHead] = e; evHead = (evHead + 1) % 20; if (evCount < 20) evCount++;
  pumpTotal++;
  prefs.begin("plant", false); prefs.putUInt("pumps", pumpTotal); prefs.end();
#endif
}
void pumpLoop() {
#if ENABLE_PUMP
  unsigned long now = millis();
  if (pumpOn && now - pumpStartMs >= pumpLimitMs) pumpStop();
  uint32_t day = timeValid() ? (uint32_t)((time(nullptr) + TZ_OFFSET_SEC) / 86400) : now / 86400000UL;
  if (day != autoDay) { autoDay = day; autoRunsToday = 0; }
  if (autoPump && !pumpOn && rawPrimed && now > 30000 && moisturePct < dryThr &&
      (!pumpEverRun || now - pumpEndMs >= PUMP_COOLDOWN_MS) && autoRunsToday < AUTO_MAX_PER_DAY) {
    autoRunsToday++;
    pumpStart(1, PUMP_RUN_MS);
  }
#endif
}

// ===================== SETTINGS =====================
void saveSettings() {
  prefs.begin("plant", false);
  prefs.putBool("auto", autoPump); prefs.putBool("log", logOn); prefs.putBool("oled", oledOn);
  prefs.putBool("sms", alertsOn);
  prefs.end();
}
void applyOled() {
  display.ssd1306_command(oledOn ? SSD1306_DISPLAYON : SSD1306_DISPLAYOFF);
}

// ===================== ALERTS / VPD =====================
float calcVpd(float t, float rh) {                 
  float svp = 0.6108f * expf(17.27f * t / (t + 237.3f));
  return svp * (1.0f - rh / 100.0f);
}

uint16_t computeAlerts() {
  const Profile& P = PROFILES[profileIdx];
  bool fD = faults & 1, fL = faults & 2, fS = faults & 4;
  uint16_t a = 0;
  if (!fS && rawPrimed) {
    if (moisturePct < dryThr) a |= (1 << 0);
    if (moisturePct > 85)     a |= (1 << 1);
  }
  if (!fD && dhtPrimed) {
    if (temperature > P.tMax) a |= (1 << 2);
    if (temperature < P.tMin) a |= (1 << 3);
    if (humidity < P.hMin)    a |= (1 << 4);
    if (humidity > P.hMax)    a |= (1 << 5);
  }
  if (!fL && timeValid()) {                          
    time_t n = time(nullptr); struct tm ti; localtime_r(&n, &ti);
    if (ti.tm_hour >= 10 && ti.tm_hour < 15 && lux < P.lMin) a |= (1 << 6);
  }
  a |= ((uint16_t)faults << 8);
  return a;
}

// ===================== TELEGRAM BOT =====================
#if ENABLE_TELEGRAM
String urlEnc(const String& s) {
  String o; char b[4];
  for (size_t i = 0; i < s.length(); i++) {
    char c = s[i];
    if (isalnum((unsigned char)c) || c == '-' || c == '_' || c == '.' || c == '~') o += c;
    else { snprintf(b, sizeof(b), "%%%02X", (uint8_t)c); o += b; }
  }
  return o;
}

bool tgSend(const String& text) {
  if (apMode || WiFi.status() != WL_CONNECTED || !TG_TOKEN[0] || !TG_CHAT[0]) return false;
  esp_task_wdt_reset();
  WiFiClientSecure c; c.setInsecure();
  HTTPClient h; h.setTimeout(4000);
  if (!h.begin(c, String("https://api.telegram.org/bot") + TG_TOKEN + "/sendMessage")) return false;
  h.addHeader("Content-Type", "application/x-www-form-urlencoded");
  int code = h.POST("chat_id=" + String(TG_CHAT) + "&text=" + urlEnc(text));
  h.end();
  esp_task_wdt_reset();
  return code == 200;
}

void tgSendStatus(const String& header) {
  String m = header + "\n";
  m += "Moisture: " + String(moisturePct) + "% (Dry < " + String(dryThr) + "%)\n";
  m += "Temp: " + String(temperature, 1) + " C\n";
  m += "Humidity: " + String(humidity, 1) + "%\n";
  m += "Light: " + String(lux, 1) + " lx\n";
  m += "VPD: " + String(calcVpd(temperature, humidity), 2) + " kPa\n";
  m += "Profile: " + String(PROFILES[profileIdx].name) + "\n";
  m += "Pump: " + String(pumpOn ? "RUNNING" : "OFF") + " | Auto: " + String(autoPump ? "ON" : "OFF") + "\n";
  m += "Fan: " + String(autoFanState ? "ON" : "OFF") + " | Vent: " + String(servoVentOpen ? "OPEN" : "CLOSED") + "\n";
  m += "Alerts: " + String(alertsOn ? "ON" : "MUTED") + "\n";
  if (faults) {
    m += "Faults:";
    if (faults & 1) m += " [DHT22]";
    if (faults & 2) m += " [BH1750]";
    if (faults & 4) m += " [Soil ADC]";
    m += "\n";
  }
  m += "Dashboard: http://" + WiFi.localIP().toString();
  tgSend(m);
}

void tgHandleCommand(String cmd) {
  cmd.trim();
  int atIdx = cmd.indexOf('@');
  if (atIdx > 0) cmd = cmd.substring(0, atIdx); 
  cmd.toLowerCase();

  Serial.println("Telegram cmd received: " + cmd);

  if (cmd == "/start" || cmd == "/help") {
    String help = "Greenhouse Bot Commands:\n"
                  "/status - Live sensor readings\n"
                  "/fan_on - Turn 12V fan ON (STM32)\n"
                  "/fan_off - Turn 12V fan OFF (STM32)\n"
                  "/vent_open - Open servo valve (STM32)\n"
                  "/vent_close - Close servo valve (STM32)\n"
                  "/pump_on - Turn water pump ON\n"
                  "/pump_off - Turn water pump OFF\n"
                  "/alerts_on - Enable phone alerts\n"
                  "/alerts_off - Mute phone alerts";
    tgSend(help);
  } else if (cmd == "/status") {
    tgSendStatus("Greenhouse Live Status:");
  } else if (cmd == "/fan_on") {
    manualOverride = true;
    manualOverrideUntil = millis() + OVERRIDE_DURATION_MS;
    autoFanState = true;
    Serial2.println("<FAN_ON>");
    tgSend("UART Command sent: Fan turned ON (Auto paused for 10m).");
  } else if (cmd == "/fan_off") {
    manualOverride = true;
    manualOverrideUntil = millis() + OVERRIDE_DURATION_MS;
    autoFanState = false;
    Serial2.println("<FAN_OFF>");
    tgSend("UART Command sent: Fan turned OFF (Auto paused for 10m).");
  } else if (cmd == "/vent_open") {
    servoVentOpen = true;
    Serial2.println("<VENT_OPEN>");
    tgSend("UART Command sent: Servo Vent Opened.");
  } else if (cmd == "/vent_close") {
    servoVentOpen = false;
    Serial2.println("<VENT_CLOSE>");
    tgSend("UART Command sent: Servo Vent Closed.");
  } else if (cmd == "/pump_on" || cmd == "/water") {
#if ENABLE_PUMP
    pumpStart(0, PUMP_MANUAL_MAX_MS);
    tgSend("Water pump turned ON (auto-stops in 60s).");
#else
    tgSend("Pump is disabled in firmware.");
#endif
  } else if (cmd == "/pump_off" || cmd == "/stop") {
#if ENABLE_PUMP
    pumpStop();
    tgSend("Water pump turned OFF.");
#else
    tgSend("Pump is disabled in firmware.");
#endif
  } else if (cmd == "/alerts_on") {
    alertsOn = true; saveSettings();
    tgSend("Automatic phone alerts ENABLED.");
  } else if (cmd == "/alerts_off") {
    alertsOn = false; saveSettings();
    tgSend("Automatic phone alerts MUTED.");
  } else if (cmd.startsWith("/")) {
    tgSend("Unknown command. Send /help for a list of commands.");
  }
}

void tgPollLoop() {
  static unsigned long lastPoll = 0;
  unsigned long now = millis();
  if (apMode || WiFi.status() != WL_CONNECTED || !TG_TOKEN[0]) return;
  if (now - lastPoll < TG_POLL_MS) return;
  lastPoll = now;

  esp_task_wdt_reset();
  WiFiClientSecure c; c.setInsecure();
  HTTPClient h; h.setTimeout(3500);
  String url = String("https://api.telegram.org/bot") + TG_TOKEN + "/getUpdates?limit=1&timeout=0";
  if (tgLastUpdateId > 0) url += "&offset=" + String(tgLastUpdateId + 1);

  if (!h.begin(c, url)) return;
  int code = h.GET();
  if (code == 200) {
    String payload = h.getString();
    int uPos = payload.indexOf("\"update_id\":");
    if (uPos >= 0) {
      long uId = strtol(payload.c_str() + uPos + 12, nullptr, 10);
      bool firstSync = (tgLastUpdateId == 0);
      tgLastUpdateId = uId;

      bool authorized = (!TG_CHAT[0]) || (payload.indexOf(String("\"id\":") + TG_CHAT) >= 0);

      int tPos = payload.lastIndexOf("\"text\":\"");
      if (!firstSync && authorized && tPos >= 0) {
        tPos += 8;
        String cmd = "";
        while (tPos < (int)payload.length()) {
          char ch = payload[tPos++];
          if (ch == '\\' && tPos < (int)payload.length()) { cmd += payload[tPos++]; continue; }
          if (ch == '\"') break;
          cmd += ch;
        }
        h.end();
        esp_task_wdt_reset();
        tgHandleCommand(cmd);
        return;
      }
    }
  }
  h.end();
  esp_task_wdt_reset();
}
#endif

bool notify(const String& m) {
  if (!alertsOn) return false;
  bool ok = false;
#if ENABLE_TELEGRAM
  ok = tgSend(m) || ok;
#endif
  return ok;
}

#if ENABLE_TELEGRAM
const char* ALERT_TXT[11] = {
  "Soil is too dry - water needed", "Soil is too wet - reduce watering", "Too hot for this plant", "Too cold for this plant",
  "Air humidity too low", "Air humidity too high", "Light too low for daytime", "",
  "DHT22 sensor not responding - check wiring", "Light sensor not responding - check I2C wiring", "Soil sensor reading invalid - check wiring"
};
unsigned long lastAlertMs[11];
void alertLoop() {
  static unsigned long nextTry = 0;
  unsigned long now = millis();
  if (now < 60000 || (long)(now - nextTry) < 0) return;
  nextTry = now + 5000;
  if (!alertsOn || !ENABLE_TELEGRAM) return;   
  uint16_t a = computeAlerts();
  for (int b = 0; b < 11; b++) {
    if (!(a & (1 << b)) || !ALERT_TXT[b][0]) continue;
    if (lastAlertMs[b] && now - lastAlertMs[b] < ALERT_COOLDOWN_MS) continue;
    String m = String("Plant Monitor: ") + ALERT_TXT[b] + "\nMoisture " + moisturePct + "%, Temp " + String(temperature, 1) +
               " C, Humidity " + String(humidity, 0) + "%, Light " + String(lux, 0) + " lx";
    if (notify(m)) lastAlertMs[b] = millis();
    else nextTry = millis() + 60000;                 
    break;                                           
  }
}
#endif

void tierClear(Tier& T) {
  T.count = 0; T.head = 0; T.unsaved = 0; tierReset(T);
  if (T.file) LittleFS.remove(T.file);
}
void handleClear() {                                 
  tierClear(t5m); tierClear(t1h); tierClear(t1d);
  evCount = 0; evHead = 0; pumpTotal = 0; monSec = 0; sinceEpoch = 0;
  prefs.begin("plant", false);
  prefs.putUInt("pumps", 0); prefs.putUInt("mon", 0); prefs.putUInt("since", 0);
  prefs.end();
  server.send(200, "application/json", "{\"ok\":true}");
}

// ===================== HTTP HANDLERS =====================
void handleRoot() { server.send_P(200, "text/html", INDEX_HTML); }

void handleData() {
  auto B = [](bool v) { return v ? "true" : "false"; };
  String j; j.reserve(900);
  j += "{\"moisture\":" + String(moisturePct);
  j += ",\"raw\":" + String((int)smoothRaw);
  j += ",\"lux\":" + String(lux, 1);
  j += ",\"temp\":" + String(temperature, 1);
  j += ",\"hum\":" + String(humidity, 1);
  j += ",\"minM\":" + String(minMoist <= maxMoist ? minMoist : 0);
  j += ",\"maxM\":" + String(maxMoist);
  j += ",\"minL\":" + String(minLux > maxLux ? 0.0f : minLux, 1);
  j += ",\"maxL\":" + String(maxLux, 1);
  j += ",\"air\":" + String(airVal);
  j += ",\"water\":" + String(waterVal);
  j += ",\"dry\":" + String(dryThr);
  j += ",\"ap\":"; j += B(apMode);
  j += ",\"rssi\":" + String(apMode ? 0 : WiFi.RSSI());
  j += ",\"up\":" + String(millis() / 1000);
  j += ",\"epoch\":" + String((uint32_t)time(nullptr));
  j += ",\"tok\":"; j += B(timeValid());
  j += ",\"pump\":"; j += B(pumpOn);
  j += ",\"fan\":"; j += B(autoFanState);
  j += ",\"vent\":"; j += B(servoVentOpen);
  j += ",\"auto\":"; j += B(autoPump);
  j += ",\"log\":"; j += B(logOn);
  j += ",\"oled\":"; j += B(oledOn);
  j += ",\"sms\":"; j += B(alertsOn);
  j += ",\"ten\":"; j += (ENABLE_PUMP ? "true" : "false");
  j += ",\"pumps\":" + String(pumpTotal);
  j += ",\"mon\":" + String(monSec);
  j += ",\"since\":" + String(sinceEpoch);
  const Profile& PF = PROFILES[profileIdx];
  j += ",\"prof\":" + String((int)profileIdx);
  j += ",\"pn\":\"" + String(PF.name) + "\"";
  j += ",\"tmin\":" + String((int)PF.tMin) + ",\"tmax\":" + String((int)PF.tMax);
  j += ",\"hmin\":" + String((int)PF.hMin) + ",\"hmax\":" + String((int)PF.hMax);
  j += ",\"lmin\":" + String((int)PF.lMin);
  j += ",\"vpd\":" + String(calcVpd(temperature, humidity), 2);
  j += ",\"bright\":" + String(brightSec);
  j += ",\"al\":" + String((int)computeAlerts());
  j += ",\"flt\":" + String((int)faults);
  j += ",\"ip\":\"" + (apMode ? WiFi.softAPIP().toString() : WiFi.localIP().toString()) + "\"}";
  server.sendHeader("Cache-Control", "no-cache");
  server.send(200, "application/json", j);
}

void streamTier(Tier& T) {
  server.sendHeader("Cache-Control", "no-cache");
  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.send(200, "application/json", "");
  String b; b.reserve(1100);
  b += "{\"iv\":" + String(T.iv) + ",\"n\":" + String(T.count);
  const char* keys[5] = { "t", "m", "l", "c", "h" };
  for (int f = 0; f < 5; f++) {
    b += ",\""; b += keys[f]; b += "\":[";
    for (int i = 0; i < T.count; i++) {
      const Rec& r = tierAt(T, i);
      if (i) b += ',';
      switch (f) {
        case 0: b += String((unsigned long)r.t); break;
        case 1: b += String((int)r.m); break;
        case 2: b += String((int)r.l); break;
        case 3: b += String((int)r.c); break;
        default: b += String((int)r.h); break;
      }
      if (b.length() > 900) { server.sendContent(b); b = ""; }
    }
    b += "]";
  }
  b += "}";
  server.sendContent(b);
  server.sendContent("");
}

void handleLog() {
  String r = server.arg("r");
  if (r == "live") streamTier(tLive);
  else if (r == "5m") streamTier(t5m);
  else if (r == "1h") streamTier(t1h);
  else if (r == "1d") streamTier(t1d);
  else server.send(400, "application/json", "{}");
}

void handleEvents() {
  String j = "{\"total\":" + String(pumpTotal) + ",\"e\":[";
  int start = (evCount < 20) ? 0 : evHead;
  for (int i = 0; i < evCount; i++) {
    const Evt& e = evts[(start + i) % 20];
    if (i) j += ",";
    j += "{\"t\":" + String((unsigned long)e.t) + ",\"s\":" + String((int)e.src) + ",\"d\":" + String((int)e.sec) + "}";
  }
  j += "]}";
  server.sendHeader("Cache-Control", "no-cache");
  server.send(200, "application/json", j);
}

void handleControl() {
  bool save = false;
  if (server.hasArg("pump")) { if (server.arg("pump") == "1") pumpStart(0, PUMP_MANUAL_MAX_MS); else pumpStop(); }
  if (server.hasArg("fan")) {
    manualOverride = true;
    manualOverrideUntil = millis() + OVERRIDE_DURATION_MS;
    autoFanState = (server.arg("fan") == "1");
    if (autoFanState) Serial2.println("<FAN_ON>");
    else Serial2.println("<FAN_OFF>");
  }
  if (server.hasArg("vent")) {
    servoVentOpen = (server.arg("vent") == "1");
    if (servoVentOpen) Serial2.println("<VENT_OPEN>");
    else Serial2.println("<VENT_CLOSE>");
  }
  if (server.hasArg("auto")) { autoPump = server.arg("auto") == "1"; save = true; }
  if (server.hasArg("log")) {
    logOn = server.arg("log") == "1"; save = true;
    if (!logOn) { tierReset(t5m); tierReset(t1h); tierReset(t1d); }
  }
  if (server.hasArg("oled")) { oledOn = server.arg("oled") == "1"; save = true; applyOled(); }
  if (server.hasArg("sms")) { alertsOn = server.arg("sms") == "1"; save = true; }
  if (save) saveSettings();
  if (server.hasArg("test")) {                       
    bool was = alertsOn; alertsOn = true;
    bool ok = notify("Plant Monitor test OK");
    alertsOn = was;
    server.send(200, "application/json", ok ? "{\"ok\":true}" : "{\"ok\":false}");
    return;
  }
  server.send(200, "application/json", "{\"ok\":true}");
}

void handleTime() {
  unsigned long t = strtoul(server.arg("t").c_str(), nullptr, 10);
  if (t > 1700000000UL) {
    struct timeval tv; tv.tv_sec = (time_t)t; tv.tv_usec = 0;
    settimeofday(&tv, nullptr);
  }
  server.send(200, "application/json", "{\"ok\":true}");
}

void handleCalibrate() {
  String type = server.arg("type");
  int value = (int)smoothRaw;
  int newAir = airVal, newWater = waterVal;
  if (type == "air") newAir = value;
  else if (type == "water") newWater = value;
  else { server.send(400, "application/json", "{\"ok\":false}"); return; }

  if (abs(newAir - newWater) < 300) {
    server.send(200, "application/json", "{\"ok\":false}");
    return;
  }
  airVal = newAir; waterVal = newWater;
  prefs.begin("plant", false);
  prefs.putInt("air", airVal); prefs.putInt("water", waterVal);
  prefs.end();
  server.send(200, "application/json", "{\"ok\":true,\"value\":" + String(value) + "}");
}

void handleConfig() {
  if (server.hasArg("profile")) {
    profileIdx = (uint8_t)constrain((int)server.arg("profile").toInt(), 0, PROFILE_COUNT - 1);
    dryThr = PROFILES[profileIdx].dry;               
  }
  if (server.hasArg("dry")) dryThr = constrain(server.arg("dry").toInt(), 5, 80);
  prefs.begin("plant", false);
  prefs.putInt("dry", dryThr);
  prefs.putUChar("prof", profileIdx);
  prefs.end();
  server.send(200, "application/json", "{\"ok\":true}");
}

// ===================== SENSORS =====================
void readSensors() {
  uint32_t sum = 0;
  for (int i = 0; i < 16; i++) { sum += analogRead(MOISTURE_PIN); delayMicroseconds(200); }
  float raw = sum / 16.0f;
  smoothRaw = rawPrimed ? (0.3f * raw + 0.7f * smoothRaw) : raw;
  rawPrimed = true;
  if (airVal == waterVal) moisturePct = 0;
  else {
    float p = (airVal - smoothRaw) * 100.0f / (float)(airVal - waterVal);
    moisturePct = constrain((int)lroundf(p), 0, 100);
  }

  float l = lightMeter.readLightLevel();
  if (l >= 0) { lux = l; lightFails = 0; } else if (lightFails < 255) lightFails++;

  float readT = dht.readTemperature();
  float readH = dht.readHumidity();
  if (!isnan(readT)) {
    smoothTemp = dhtPrimed ? (0.2f * readT + 0.8f * smoothTemp) : readT;
    temperature = smoothTemp;
  }
  if (!isnan(readH)) {
    smoothHum = dhtPrimed ? (0.2f * readH + 0.8f * smoothHum) : readH;
    humidity = smoothHum;
  }
  if (!isnan(readT) && !isnan(readH)) dhtPrimed = true;
  if (isnan(readT) || isnan(readH)) { if (dhtFails < 255) dhtFails++; } else dhtFails = 0;

  minMoist = min(minMoist, moisturePct); maxMoist = max(maxMoist, moisturePct);
  minLux = min(minLux, lux); maxLux = max(maxLux, lux);

  digitalWrite(LED_PIN, moisturePct < dryThr ? !digitalRead(LED_PIN) : LOW);

  faults = (dhtFails >= 5 ? 1 : 0) | (lightFails >= 5 ? 2 : 0) | ((smoothRaw < 150 || smoothRaw > 4050) ? 4 : 0);

  // --- AUTOMATIC TEMPERATURE FAN CONTROL (WITH MANUAL OVERRIDE) ---
  const Profile& currentProfile = PROFILES[profileIdx];
  unsigned long nowMs = millis();

  if (manualOverride && nowMs > manualOverrideUntil) {
    manualOverride = false;
  }

  if (!manualOverride) {
    if (temperature > currentProfile.tMax && !autoFanState) {
      autoFanState = true;
      Serial2.println("<FAN_ON>");
    } 
    else if (temperature <= (currentProfile.tMax - 2.0f) && autoFanState) {
      autoFanState = false;
      Serial2.println("<FAN_OFF>");
    }
  }

  tierAdd(tLive, moisturePct, lux, temperature, humidity);
  if (logOn) {
    tierAdd(t5m, moisturePct, lux, temperature, humidity);
    tierAdd(t1h, moisturePct, lux, temperature, humidity);
    tierAdd(t1d, moisturePct, lux, temperature, humidity);
  }

  int checksum = moisturePct + (int)lux + (int)temperature + (int)humidity;
  Serial2.printf("<%d,%.1f,%.1f,%.1f,%d>\n", moisturePct, lux, temperature, humidity, checksum);
}

void secTick() {
  uint32_t bd = timeValid() ? (uint32_t)((time(nullptr) + TZ_OFFSET_SEC) / 86400) : 0;
  if (bd != brightDay) { brightDay = bd; brightSec = 0; }
  if (lux >= LIGHT_BRIGHT_LX) brightSec++;
  if (logOn) {
    monSec++;
    if (monSec % 600 == 0) { prefs.begin("plant", false); prefs.putUInt("mon", monSec); prefs.end(); }
  }
  if (!sinceEpoch && timeValid()) {
    sinceEpoch = (uint32_t)time(nullptr);
    prefs.begin("plant", false); prefs.putUInt("since", sinceEpoch); prefs.end();
  }
}

// ===================== OLED =====================
void drawPage(int page) {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE); display.setTextSize(1);
  display.setCursor(0, 0);
  display.print(apMode ? "AP: " : "IP: ");
  display.print(apMode ? WiFi.softAPIP().toString() : WiFi.localIP().toString());
  if (faults) { display.setCursor(120, 0); display.print("!"); }     
  display.drawLine(0, 10, 127, 10, SSD1306_WHITE);

  if (page == 0) {
    display.setCursor(0, 15); display.print("Temp/Hum: "); display.print(temperature, 1); display.print("C "); display.print((int)humidity); display.print("%");
    display.setCursor(0, 28); display.print("Moisture: "); display.print(moisturePct); display.print(" %");
    display.drawRect(0, 46, 128, 14, SSD1306_WHITE);
    int w = constrain(map(moisturePct, 0, 100, 0, 124), 0, 124);
    if (w > 0) display.fillRect(2, 48, w, 10, SSD1306_WHITE);
    int tx = 2 + map(dryThr, 0, 100, 0, 124);
    display.drawLine(tx, 44, tx, 62, SSD1306_WHITE);
  } else if (page == 1) {
    display.setCursor(0, 15); display.print("Light: "); display.print(lux, 1); display.print(" lx");
    display.setCursor(0, 26); display.print("Min/Max: "); display.print(minMoist); display.print("/"); display.print(maxMoist); display.print("%");
    display.setCursor(0, 37); if (apMode) display.print("Wi-Fi: hotspot"); else { display.print("Wi-Fi: "); display.print(WiFi.RSSI()); display.print(" dBm"); }
    display.setCursor(0, 48); unsigned long s = millis() / 1000; display.print("Up: "); display.print(s / 3600); display.print("h "); display.print((s % 3600) / 60); display.print("m");
  } else {
    display.setCursor(0, 15);
    if (timeValid()) {
      time_t nt = time(nullptr); struct tm ti; localtime_r(&nt, &ti);
      char b[24]; strftime(b, sizeof(b), "%d %b %Y %H:%M", &ti); display.print(b);
    } else display.print("Clock: not set");
    display.setCursor(0, 26); display.print("Pump:"); display.print(pumpOn ? "ON" : "off"); display.print(" Fan:"); display.print(autoFanState ? "ON" : "off");
    display.setCursor(0, 37); display.print("Vent:"); display.print(servoVentOpen ? "OPEN" : "cls"); display.print(" Alrt:"); display.print(alertsOn ? "ON" : "off");
    display.setCursor(0, 48); display.print("Mon: "); display.print(monSec / 86400); display.print("d "); display.print((monSec % 86400) / 3600); display.print("h ");
  }
  display.display();
}

// ===================== SETUP / LOOP =====================
void setup() {
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);     

  pinMode(LED_PIN, OUTPUT);
#if ENABLE_PUMP
  pinMode(PUMP_PIN, OUTPUT); pumpWrite(false);      
#endif
  analogSetPinAttenuation(MOISTURE_PIN, ADC_11db);

  LittleFS.begin(true);                              

  prefs.begin("plant", true);
  airVal = prefs.getInt("air", 3500); waterVal = prefs.getInt("water", 1500); dryThr = prefs.getInt("dry", 30);
  autoPump = prefs.getBool("auto", false); logOn = prefs.getBool("log", true); oledOn = prefs.getBool("oled", true);
  alertsOn = prefs.getBool("sms", true);
  monSec = prefs.getUInt("mon", 0); sinceEpoch = prefs.getUInt("since", 0); pumpTotal = prefs.getUInt("pumps", 0);
  profileIdx = (uint8_t)constrain((int)prefs.getUChar("prof", 0), 0, PROFILE_COUNT - 1);
  prefs.end();

  tierLoad(t5m); tierLoad(t1h); tierLoad(t1d);       

  Wire.begin(21, 22);
  display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS); display.setRotation(2);
  applyOled();
  lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE);
  dht.begin();

  pinMode(BTN_PIN, INPUT_PULLUP);

  WiFi.mode(WIFI_STA); WiFi.setHostname(HOSTNAME); WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 15000) delay(250);
  apMode = (WiFi.status() != WL_CONNECTED);
  if (apMode) { WiFi.mode(WIFI_AP); WiFi.softAP(AP_SSID, AP_PASSWORD); delay(1500); }

  configTime(TZ_OFFSET_SEC, 0, "pool.ntp.org", "time.google.com");   

  if (MDNS.begin(HOSTNAME)) MDNS.addService("http", "tcp", 80);
  server.on("/", handleRoot);
  server.on("/api/data", handleData);
  server.on("/api/log", handleLog);
  server.on("/api/events", handleEvents);
  server.on("/api/control", handleControl);
  server.on("/api/settime", handleTime);
  server.on("/api/calibrate", handleCalibrate);
  server.on("/api/config", handleConfig);
  server.on("/api/clear", handleClear);
  server.begin();

  ArduinoOTA.setHostname(HOSTNAME);
  ArduinoOTA.setPassword(OTA_PASSWORD);
  ArduinoOTA.onStart([]() { pumpStop(); esp_task_wdt_delete(NULL); });   
  ArduinoOTA.onError([](ota_error_t e) { ESP.restart(); });
  ArduinoOTA.begin();
#if ENABLE_TELEGRAM
  if (!apMode) {
    tgSend("Plant Monitor online - http://" + WiFi.localIP().toString() + "\nSend /status or /help anytime.");
  }
#endif

#if ESP_ARDUINO_VERSION_MAJOR >= 3
  esp_task_wdt_config_t wdtCfg = { (uint32_t)WDT_TIMEOUT * 1000, 0, true };   
  esp_task_wdt_reconfigure(&wdtCfg);
#else
  esp_task_wdt_init(WDT_TIMEOUT, true);
#endif
  esp_task_wdt_add(NULL);
}

unsigned long lastSensor = 0, lastPage = 0, lastSec = 0, lastWifi = 0;
int page = 0;

void loop() {
  esp_task_wdt_reset();          
  ArduinoOTA.handle();
  server.handleClient();
  unsigned long now = millis();
  if (now - lastSensor >= SENSOR_INTERVAL_MS) { lastSensor = now; readSensors(); if (oledOn) drawPage(page); }
  if (now - lastPage >= PAGE_INTERVAL_MS) { lastPage = now; page = (page + 1) % 3; }
  if (now - lastSec >= 1000) { lastSec += 1000; secTick(); }
  pumpLoop();
#if ENABLE_TELEGRAM
  tgPollLoop();
#endif
#if ENABLE_TELEGRAM
  alertLoop();
#endif
  static bool btnPrev = HIGH; static unsigned long btnT = 0;   
  bool btnNow = digitalRead(BTN_PIN);
  if (btnPrev == HIGH && btnNow == LOW && now - btnT > 250) { btnT = now; alertsOn = !alertsOn; saveSettings(); }
  btnPrev = btnNow;
  if (now - lastWifi >= 30000) { lastWifi = now; if (!apMode && WiFi.status() != WL_CONNECTED) WiFi.reconnect(); }
}