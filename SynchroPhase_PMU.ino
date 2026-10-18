// ============================================================
//  SYNCHROPHASE - GPS-SYNCHRONISED PHASOR MEASUREMENT UNIT
//  ESP32 + NEO-M8N GPS (PPS) + 3x ZMPT101B
// ------------------------------------------------------------
//  A PMU is the power grid's high-speed eye. Unlike a SCADA
//  reading that arrives every few seconds with no common time
//  reference, a PMU reports the voltage PHASOR (magnitude +
//  angle) of all three phases, time-stamped against GPS to
//  the microsecond, 50 times per second - so phasors measured
//  hundreds of kilometres apart can be compared angle-to-
//  angle. That is the Wide-Area Measurement System (WAMS)
//  that modern grid operators rely on.
//
//  V2 - MULTI-FILE STRUCTURE
//  The v1 firmware was a single 409-line sketch. v2 keeps its
//  behaviour, scaling and wire format identical, but splits
//  the concerns into header modules sitting next to this
//  sketch (Arduino compiles them as one unit):
//
//    gps_time.h  GPS PPS interrupt + NMEA RMC parsing; owns
//                the SOC / FRACSEC time base.
//    phasor.h    4.8 kHz sampling, ring buffers, DFT phasor
//                estimation with frequency-tracked resampling,
//                symmetrical components, frequency + ROCOF.
//    c37.h       The wire protocol: CRC-CCITT, the 38-byte
//                data frame, and a CFG-style config frame a
//                client requests by sending byte 0x05.
//    (this file) Wi-Fi, the web dashboard, TCP stream server
//                and the main loop that glues it together.
//
//  A matching Python client (pmu_client.py) ships alongside
//  and parses both frame types.
//
//  Author : Deepesh Chauhan, B.Tech Electrical Engineering
//           RGGEC Kangra
// ============================================================

#include <WiFi.h>
#include <WebServer.h>
#include <math.h>

#include "gps_time.h"
#include "phasor.h"
#include "c37.h"

// ---------------- WI-FI (replace with your credentials) ------
const char* WIFI_SSID = "YOUR_WIFI_NAME";
const char* WIFI_PASS = "YOUR_WIFI_PASSWORD";

// ---------------- Servers -------------------------------------
// Port 80: dashboard. Port 4711: C37.118-style phasor stream.
#define PMU_STREAM_PORT 4711
WebServer  server(80);
WiFiServer pmuServer(PMU_STREAM_PORT);
WiFiClient pmuClient;

// ---------------- Event log -----------------------------------
// A short rolling log shown on the dashboard. Deliberately
// simple string storage - this is a bench instrument, not a
// SCADA historian.
String eventLog = "Boot\n";

void logEvent(const String& msg) {
  eventLog += msg + "\n";
  if (eventLog.length() > 1600) eventLog = eventLog.substring(500);
  Serial.println(msg);
}

// =============================================================
//  STREAM CLIENT MANAGEMENT
//  One client at a time (bench instrument). While connected,
//  any byte the client sends is a command: 0x05 asks for one
//  config frame, answered immediately; the data stream then
//  continues on its 20 ms cadence undisturbed.
// =============================================================
void serviceStreamClient() {
  if (!pmuClient || !pmuClient.connected()) {
    WiFiClient nc = pmuServer.available();
    if (nc) {
      pmuClient = nc;
      logEvent("C37.118 client connected");
    }
    return;
  }
  while (pmuClient.available()) {
    uint8_t cmd = pmuClient.read();
    if (cmd == C37_CMD_REQUEST_CFG) {
      uint8_t frame[C37_MAX_FRAME_LEN];
      int len = c37BuildConfigFrame(frame, gpsSoc(), gpsFracsec());
      pmuClient.write(frame, len);
      logEvent("Config frame sent on client request");
    }
    // Unknown command bytes are ignored by design: the stream
    // protocol stays forward-compatible.
  }
}

// Emit one data frame built from the latest phasors + time.
// Called on the 20 ms frame cadence; a no-op with no client.
void sendDataFrame() {
  if (!pmuClient || !pmuClient.connected()) return;
  uint8_t frame[C37_MAX_FRAME_LEN];
  int len = c37BuildDataFrame(frame, gpsSoc(), gpsFracsec(),
                              gpsLocked(), ph, gridFreq, rocof);
  pmuClient.write(frame, len);
}

// =============================================================
//  DASHBOARD - live phasor diagram + values
// =============================================================
const char DASHBOARD[] PROGMEM = R"HTML(
<!DOCTYPE html><html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>SynchroPhase PMU</title>
<style>
 body{font-family:'Segoe UI',Arial;background:#0b1120;color:#e2e8f0;margin:0;padding:16px}
 h1{color:#38bdf8;text-align:center;margin:4px}
 .sub{text-align:center;color:#94a3b8;margin-bottom:14px}
 .wrap{max-width:980px;margin:auto}
 .cards{display:flex;gap:12px;flex-wrap:wrap;justify-content:center}
 .card{background:#1e293b;border-radius:12px;padding:14px 18px;min-width:140px;text-align:center}
 .card b{font-size:1.6em;color:#f8fafc}.card span{color:#94a3b8;font-size:.85em}
 .ok{color:#4ade80}.warn{color:#fbbf24}
 canvas{background:#0f172a;border:1px solid #334155;border-radius:12px}
 table{margin:auto;border-collapse:collapse}
 td,th{border-bottom:1px solid #334155;padding:5px 12px;text-align:right}
 pre{background:#0f172a;border-radius:8px;padding:10px;white-space:pre-wrap;max-height:140px;overflow:auto}
 .row{display:flex;gap:16px;flex-wrap:wrap;justify-content:center;align-items:flex-start}
</style></head><body><div class="wrap">
<h1>🛰️ SynchroPhase — PMU</h1>
<div class="sub">GPS-Synchronised Synchrophasors · IEEE C37.118-Style Streaming · 50 frames/s</div>
<div class="cards">
 <div class="card"><span>FREQUENCY</span><br><b id="fq">50.00</b><br><span>Hz</span></div>
 <div class="card"><span>ROCOF</span><br><b id="rc">0.00</b><br><span>Hz/s</span></div>
 <div class="card"><span>POS-SEQ |V|</span><br><b id="ps">0</b><br><span>V rms</span></div>
 <div class="card"><span>POS-SEQ ANGLE</span><br><b id="pa">0</b><br><span>degrees</span></div>
 <div class="card"><span>UNBALANCE</span><br><b id="ub">0</b><br><span>% |V-| / |V+|</span></div>
 <div class="card"><span>GPS / PPS</span><br><b id="gps">--</b><br><span>time sync</span></div>
</div>
<div class="row" style="margin-top:14px">
 <div><h3 style="text-align:center">Phasor Diagram</h3>
 <canvas id="ph" width="380" height="380"></canvas></div>
 <div><h3 style="text-align:center">Phase Phasors (UTC-referenced)</h3>
 <table id="tbl"><tr><th>Phase</th><th>|V| rms</th><th>Angle °</th></tr></table>
 <p style="color:#94a3b8;max-width:420px">Angles are referenced to the GPS-disciplined
 sampling clock, so two SynchroPhase units at different substations produce directly
 comparable angles — the entire point of a PMU.</p></div>
</div>
<h3>Event Log</h3><pre id="log"></pre>
<script>
function drawPh(d){
 const cv=document.getElementById('ph'),x=cv.getContext('2d');
 x.clearRect(0,0,380,380);
 const cx=190,cy=190,R=165;
 x.strokeStyle='#334155';x.beginPath();x.arc(cx,cy,R,0,7);x.stroke();
 x.beginPath();x.moveTo(cx-R,cy);x.lineTo(cx+R,cy);x.moveTo(cx,cy-R);x.lineTo(cx,cy+R);x.stroke();
 const cols=['#f87171','#facc15','#38bdf8'];
 const mx=Math.max(1,...d.ph.map(p=>p[0]));
 d.ph.forEach((p,i)=>{
   const a=p[1]*Math.PI/180,r=(p[0]/mx)*R*0.92;
   const ex=cx+r*Math.cos(a),ey=cy-r*Math.sin(a);
   x.strokeStyle=cols[i];x.lineWidth=3;x.beginPath();x.moveTo(cx,cy);x.lineTo(ex,ey);x.stroke();
   x.fillStyle=cols[i];x.beginPath();x.arc(ex,ey,4,0,7);x.fill();
 });
}
async function tick(){
 const d=await(await fetch('/data')).json();
 document.getElementById('fq').textContent=d.f.toFixed(3);
 document.getElementById('rc').textContent=d.rocof.toFixed(2);
 document.getElementById('ps').textContent=d.ps[0].toFixed(1);
 document.getElementById('pa').textContent=d.ps[1].toFixed(1);
 document.getElementById('ub').textContent=d.ub.toFixed(2);
 const g=document.getElementById('gps');
 g.textContent=d.gps?'LOCKED':'NO FIX';g.className=d.gps?'ok':'warn';
 let rows='<tr><th>Phase</th><th>|V| rms</th><th>Angle °</th></tr>';
 const names=['R','Y','B'];
 d.ph.forEach((p,i)=>{rows+='<tr><td>'+names[i]+'</td><td>'+p[0].toFixed(1)+'</td><td>'+p[1].toFixed(1)+'</td></tr>';});
 document.getElementById('tbl').innerHTML=rows;
 document.getElementById('log').textContent=d.log;
 drawPh(d);
}
setInterval(tick,200);tick();
</script></div></body></html>)HTML";

void handleRoot() { server.send_P(200, "text/html", DASHBOARD); }

void handleData() {
  String j = "{";
  j += "\"f\":" + String(freqSmooth, 3);
  j += ",\"rocof\":" + String(rocof, 2);
  j += ",\"ub\":" + String(unbalancePct, 2);
  j += ",\"ps\":[" + String(posSeq.mag, 1) + "," + String(posSeq.angDeg, 1) + "]";
  j += ",\"gps\":" + String(gpsLocked() ? "true" : "false");
  j += ",\"ph\":[";
  for (int i = 0; i < 3; i++) {
    if (i) j += ",";
    j += "[" + String(ph[i].mag, 1) + "," + String(ph[i].angDeg, 1) + "]";
  }
  j += "],\"log\":\"";
  String esc = eventLog; esc.replace("\n", "\\n"); esc.replace("\"", "'");
  j += esc + "\"}";
  server.send(200, "application/json", j);
}

// =============================================================
//  SETUP
//  Order matters: GPS time first (its interrupt can fire
//  immediately), then sampling, then the network services.
// =============================================================
void setup() {
  Serial.begin(115200);

  gpsBegin();        // NMEA UART + PPS interrupt
  phasorBegin();     // ADC + 4.8 kHz sampling timer

  WiFi.begin(WIFI_SSID, WIFI_PASS);
  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 8000) delay(300);
  if (WiFi.status() == WL_CONNECTED) {
    server.on("/", handleRoot);
    server.on("/data", handleData);
    server.begin();
    pmuServer.begin();
    logEvent("Dashboard: http://" + WiFi.localIP().toString() +
             " | C37.118 stream: TCP port 4711");
  }
  logEvent("SynchroPhase v2 ready. Waiting for GPS PPS lock...");
}

// =============================================================
//  LOOP
//  Cooperative cadence: every pass services sampling + GPS +
//  HTTP; every 20 ms a new phasor set is computed and one
//  data frame is streamed (50 fps, the C37.118 rate this
//  unit advertises). A 10 s heartbeat logs the sync state.
// =============================================================
unsigned long lastFrame = 0, lastLog = 0;
bool lockAnnounced = false;
void loop() {
  server.handleClient();
  gpsPoll();
  phasorService();
  serviceStreamClient();

  // Announce the moment GPS time latches - from here on the
  // STAT word clears and every frame carries honest time.
  if (gpsLocked() && !lockAnnounced) {
    lockAnnounced = true;
    logEvent("GPS time latched (SOC=" + String(gpsSoc()) +
             ", RMC seen=" + String(gpsRmcSeen()) + ")");
  }

  if (millis() - lastFrame >= 20) {          // 50 frames per second
    lastFrame = millis();
    if (phasorReady()) {
      computePhasors();
      sendDataFrame();
    }
  }

  if (gpsLocked() && millis() - lastLog > 10000) {
    lastLog = millis();
    logEvent("Sync OK: f=" + String(freqSmooth, 3) + " Hz, pos-seq " +
             String(posSeq.mag, 1) + " V at " + String(posSeq.angDeg, 1) + " deg");
  }
}
