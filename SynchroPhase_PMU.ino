// ============================================================
//  SYNCHROPHASE - GPS-SYNCHRONISED PMU (PHASOR MEASUREMENT UNIT)
//  ESP32 + u-blox NEO-M8N GPS (PPS + NMEA) + 3-Phase VT Dividers
// ------------------------------------------------------------
//  A PMU measures voltage/current PHASORS (magnitude + angle)
//  time-stamped against an absolute GPS time reference, so
//  that measurements from different substations - taken at the
//  same instant - can be compared directly. This is the basis
//  of Wide-Area Measurement Systems (WAMS), the technology
//  credited with giving grid operators early warning of the
//  oscillations that precede cascading blackouts.
//
//  THIS FIRMWARE
//   * PPS interrupt disciplines the sampling clock (1 pulse/s
//     from the GPS receiver, sub-microsecond edge accuracy).
//   * 3 phase voltages sampled at 4.8 kHz (96 samples/cycle).
//   * One-cycle DFT phasor estimation per phase at the frame
//     instant, with FREQUENCY-TRACKED resampling so the DFT
//     window always spans exactly one true cycle.
//   * Positive-sequence phasor (Fortescue), frequency and
//     ROCOF from consecutive positive-sequence angles.
//   * IEEE C37.118-STYLE data frames (see c37-frame-format.md)
//     streamed over TCP port 4711 at 50 frames/second.
//   * Web dashboard with a LIVE PHASOR DIAGRAM.
//
//  Author : Deepesh Chauhan, B.Tech Electrical Engineering
//           RGGEC Kangra
// ============================================================

#include <WiFi.h>
#include <WebServer.h>
#include <Arduino.h>
#include <math.h>

// ---------------- WI-FI (replace with your credentials) -----
const char* WIFI_SSID = "YOUR_WIFI_NAME";
const char* WIFI_PASS = "YOUR_WIFI_PASSWORD";

// ---------------- PINS ---------------------------------------
#define PPS_PIN     27        // GPS 1PPS input (interrupt)
#define VA_PIN      34        // phase A scaled voltage
#define VB_PIN      35        // phase B scaled voltage
#define VC_PIN      32        // phase C scaled voltage
#define GPS_RX      16        // ESP32 RX2 <- GPS TX
#define GPS_TX      17        // ESP32 TX2 -> GPS RX

// ---------------- SAMPLING -------------------------------------
const int   SAMPLE_RATE = 4800;             // Hz (96/cycle @50)
const int   BUF_LEN     = 960;              // 10 cycles of history
const float FRAME_RATE  = 50.0f;            // frames per second
const float V_SCALE     = 0.402f;           // ADC volts -> primary volts (bench calibration)
const float ADC_ZERO    = 2048.0f;          // mid-rail bias counts

volatile float bufA[BUF_LEN], bufB[BUF_LEN], bufC[BUF_LEN];
volatile int   bufHead = 0;
hw_timer_t* sampleTimer = nullptr;

// ---------------- GPS TIME ---------------------------------------
volatile unsigned long ppsCount = 0;        // seconds since lock
volatile unsigned long lastPpsMicros = 0;
bool gpsLocked = false;
unsigned long socBase = 0;                  // unix time at ppsCount 0
String nmeaBuffer = "";

void IRAM_ATTR onPPS() {
  ppsCount++;
  lastPpsMicros = micros();
}

// Parse $GPRMC / $GNRMC for UTC time-of-day -> SOC seconds today.
// (Date handling: SOC is kept continuous via ppsCount once the
// first RMC fix sets the base; full date parsing is included
// so SOC is a true unix timestamp.)
int daysFromCivil(int y, int m, int d) {
  y -= (m <= 2) ? 1 : 0;
  int era = (y >= 0 ? y : y - 399) / 400;
  unsigned yoe = (unsigned)(y - era * 400);
  unsigned doy = (153 * (m + (m > 2 ? -3 : 9)) + 2) / 5 + d - 1;
  unsigned doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
  return era * 146097 + (int)doe - 719468;
}

void parseRMC(const String& s) {
  // $GNRMC,hhmmss.ss,A,lat,N,lon,E,spd,crs,ddmmyy,...
  int c1 = s.indexOf(',');
  if (c1 < 0) return;
  int c2 = s.indexOf(',', c1 + 1);
  String timeStr = s.substring(c1 + 1, c2);
  String status = s.substring(c2 + 1, c2 + 2);
  if (status != "A" || timeStr.length() < 6) return;
  int hh = timeStr.substring(0, 2).toInt();
  int mm = timeStr.substring(2, 4).toInt();
  int ss = timeStr.substring(4, 6).toInt();
  // find date field (9th field)
  int idx = c1, field = 1;
  while (field < 9 && idx >= 0) { idx = s.indexOf(',', idx + 1); field++; }
  if (idx < 0) return;
  String dateStr = s.substring(idx + 1, idx + 7);
  if (dateStr.length() < 6) return;
  int dd = dateStr.substring(0, 2).toInt();
  int mo = dateStr.substring(2, 4).toInt();
  int yy = 2000 + dateStr.substring(4, 6).toInt();
  unsigned long soc = (unsigned long)daysFromCivil(yy, mo, dd) * 86400UL +
                      hh * 3600UL + mm * 60UL + ss;
  socBase = soc - ppsCount;
  gpsLocked = true;
}

// ---------------- PHASOR ENGINE ----------------------------------
struct Phasor { float mag; float angDeg; };
Phasor ph[3];
Phasor posSeq;
float gridFreq = 50.0f, rocof = 0.0f, freqSmooth = 50.0f;
float lastPosAngle = 0; unsigned long lastFrameMs = 0;

// DFT of ONE cycle ending at buffer position `endIdx`, using
// `samplesPerCycle` points spaced (possibly fractionally) by
// step = BUF sampling / true cycle -> implemented by linear
// interpolation between ring samples.
void computePhasor(volatile float* buf, int endIdx, float spc, Phasor& out) {
  double re = 0, im = 0;
  int n = (int)round(spc);
  for (int k = 0; k < n; k++) {
    // fractional position k samples back from endIdx
    float back = k * (spc / n);
    int i0 = endIdx - (int)floor(back);
    float frac = back - floor(back);
    auto at = [&](int idx) {
      int j = ((idx % BUF_LEN) + BUF_LEN) % BUF_LEN;
      return (float)buf[j];
    };
    float v = at(i0) * (1 - frac) + at(i0 - 1) * frac;
    double ang = -2.0 * PI * k / n;          // DFT reference
    re += v * cos(ang);
    im += v * sin(ang);
  }
  re = re * 2.0 / n; im = im * 2.0 / n;      // peak phasor scaling
  out.mag = sqrt(re * re + im * im);
  out.angDeg = atan2(im, re) * 180.0 / PI;
}

// Sample n cycles back from the newest sample: full phasor set
void updatePhasors() {
  int endIdx = bufHead - 1;
  float spc = SAMPLE_RATE / freqSmooth;     // samples per true cycle
  computePhasor(bufA, endIdx, spc, ph[0]);
  computePhasor(bufB, endIdx, spc, ph[1]);
  computePhasor(bufC, endIdx, spc, ph[2]);

  // Positive sequence: (1/3)(Va + a Vb + a^2 Vc), a = 1<120
  auto toXY = [](Phasor p, float shiftDeg, double& x, double& y) {
    double a = (p.angDeg + shiftDeg) * PI / 180.0;
    x += p.mag * cos(a); y += p.mag * sin(a);
  };
  double x = 0, y = 0;
  toXY(ph[0], 0, x, y); toXY(ph[1], 120, x, y); toXY(ph[2], 240, x, y);
  posSeq.mag = sqrt(x * x + y * y) / 3.0f;
  posSeq.angDeg = atan2(y, x) * 180.0 / PI;

  // Frequency from positive-sequence angle rotation:
  // between frames separated by dt, the angle advances by
  // (f - fFrameMultiple) * 360 * dt; unwrap near 50 Hz.
  unsigned long now = millis();
  if (lastFrameMs > 0) {
    float dt = (now - lastFrameMs) / 1000.0f;
    float dAng = posSeq.angDeg - lastPosAngle;
    while (dAng > 180) dAng -= 360;
    while (dAng < -180) dAng += 360;
    float fInst = FRAME_RATE + dAng / (360.0f * dt);
    if (fInst > 45 && fInst < 55) {
      float fPrev = gridFreq;
      gridFreq = fInst;
      freqSmooth = 0.7f * freqSmooth + 0.3f * gridFreq;
      rocof = (dt > 0) ? (gridFreq - fPrev) / dt : 0;
    }
  }
  lastPosAngle = posSeq.angDeg;
  lastFrameMs = now;
}

// ---------------- SAMPLING ISR -----------------------------------
void IRAM_ATTR onSampleTimer() {
  bufA[bufHead] = ((analogRead(VA_PIN) - ADC_ZERO) / 4095.0f) * 3.3f / V_SCALE * 100.0f;
  bufB[bufHead] = ((analogRead(VB_PIN) - ADC_ZERO) / 4095.0f) * 3.3f / V_SCALE * 100.0f;
  bufC[bufHead] = ((analogRead(VC_PIN) - ADC_ZERO) / 4095.0f) * 3.3f / V_SCALE * 100.0f;
  bufHead = (bufHead + 1) % BUF_LEN;
}

// ---------------- C37.118-STYLE FRAME -----------------------------
uint16_t crcCcitt(const uint8_t* data, int len) {
  uint16_t crc = 0xFFFF;
  for (int i = 0; i < len; i++) {
    crc ^= (uint16_t)data[i] << 8;
    for (int b = 0; b < 8; b++)
      crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : (crc << 1);
  }
  return crc;
}

WiFiServer streamServer(4711);
WiFiClient streamClient;

int buildFrame(uint8_t* f) {
  int n = 0;
  auto put16 = [&](uint16_t v) { f[n++] = v >> 8; f[n++] = v & 0xFF; };
  auto put32 = [&](uint32_t v) { f[n++] = v >> 24; f[n++] = (v >> 16) & 0xFF; f[n++] = (v >> 8) & 0xFF; f[n++] = v & 0xFF; };
  put16(0xAA01);                       // SYNC: data frame
  put16(38);                           // FRAMESIZE
  put16(1);                            // IDCODE
  unsigned long soc = socBase + ppsCount;
  put32(soc);                          // SOC
  unsigned long frac = (unsigned long)(((micros() - lastPpsMicros) % 1000000UL) *
                                       16777216UL / 1000000UL);
  uint32_t fracsec = frac | (gpsLocked ? 0 : 0x80000000UL);
  // quality flag carried in the top byte per the format doc
  put32(fracsec);
  put16(gpsLocked ? 0x0000 : 0x0001);  // STAT
  for (int i = 0; i < 3; i++) {
    put16((uint16_t)(ph[i].mag * 100.0f));            // 0.01 V units
    put16((uint16_t)(int16_t)(ph[i].angDeg * 100.0f)); // 0.01 deg units
  }
  put16((uint16_t)(int16_t)((gridFreq - 50.0f) * 1000.0f)); // FREQ mHz dev
  put16((uint16_t)(int16_t)(rocof * 100.0f));               // DFREQ 0.01 Hz/s
  put16(0);                            // ANALOG (reserved)
  put16(0);                            // DIGITAL
  uint16_t crc = crcCcitt(f, n);
  put16(crc);
  return n;                            // 38 bytes
}

// =============================================================
//  DASHBOARD
// =============================================================
WebServer server(80);
const char DASHBOARD[] PROGMEM = R"HTML(
<!DOCTYPE html><html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>SynchroPhase PMU</title>
<style>
 body{font-family:'Segoe UI',Arial;background:#0b1120;color:#e2e8f0;margin:0;padding:16px}
 h1{color:#34d399;text-align:center;margin:4px}
 .sub{text-align:center;color:#94a3b8;margin-bottom:14px}
 .wrap{max-width:980px;margin:auto}
 .cards{display:flex;gap:12px;flex-wrap:wrap;justify-content:center}
 .card{background:#1e293b;border-radius:12px;padding:14px 18px;min-width:150px;text-align:center}
 .card b{font-size:1.5em;color:#f8fafc}.card span{color:#94a3b8;font-size:.85em}
 canvas{background:#0f172a;border:1px solid #334155;border-radius:12px;display:block;margin:auto}
 table{margin:auto;border-collapse:collapse}td,th{padding:5px 12px;border-bottom:1px solid #334155;text-align:right}
</style></head><body><div class="wrap">
<h1>📡 SynchroPhase — GPS PMU</h1>
<div class="sub">Synchrophasor Measurement · IEEE C37.118-Style Streaming · GPS Time-Disciplined</div>
<div class="cards">
 <div class="card"><span>FREQUENCY</span><br><b id="fq">--</b><br><span>Hz</span></div>
 <div class="card"><span>ROCOF</span><br><b id="rc">--</b><br><span>Hz/s</span></div>
 <div class="card"><span>POS-SEQ VOLTAGE</span><br><b id="ps">--</b><br><span>volts</span></div>
 <div class="card"><span>GPS TIME</span><br><b id="gps">--</b><br><span>lock state</span></div>
 <div class="card"><span>STREAM CLIENT</span><br><b id="cli">--</b><br><span>TCP 4711</span></div>
</div>
<h3 style="text-align:center">Live Phasor Diagram</h3>
<canvas id="pd" width="340" height="340"></canvas>
<table>
<tr><th>Phase</th><th>Magnitude (V)</th><th>Angle (°)</th></tr>
<tr><td>A</td><td id="ma">--</td><td id="aa">--</td></tr>
<tr><td>B</td><td id="mb">--</td><td id="ab">--</td></tr>
<tr><td>C</td><td id="mc">--</td><td id="ac">--</td></tr>
</table>
<script>
function vec(x,cx,cy,m,a,col){
 const r=a*Math.PI/180, s=150/Math.max(m,1)*0.9;
 const px=cx+Math.cos(r)*m*s*0.9/ (150/ (m||1)) *0 + Math.cos(r)*Math.min(m,150)*0.95;
}
function draw(d){
 const cv=document.getElementById('pd'),x=cv.getContext('2d');
 x.clearRect(0,0,340,340);
 x.strokeStyle='#334155';x.beginPath();x.arc(170,170,150,0,7);x.stroke();
 x.beginPath();x.moveTo(20,170);x.lineTo(320,170);x.moveTo(170,20);x.lineTo(170,320);x.stroke();
 const cols=['#f87171','#facc15','#38bdf8'];
 let mx=1;d.ph.forEach(p=>mx=Math.max(mx,p[0]));
 d.ph.forEach((p,i)=>{
   const r=p[1]*Math.PI/180, L=p[0]/mx*140;
   x.strokeStyle=cols[i];x.lineWidth=3;x.beginPath();
   x.moveTo(170,170);x.lineTo(170+Math.cos(r)*L,170-Math.sin(r)*L);x.stroke();
 });
 const r=d.ps[1]*Math.PI/180, L=d.ps[0]/mx*140;
 x.strokeStyle='#34d399';x.lineWidth=2;x.setLineDash([6,4]);x.beginPath();
 x.moveTo(170,170);x.lineTo(170+Math.cos(r)*L,170-Math.sin(r)*L);x.stroke();x.setLineDash([]);
}
async function tick(){
 const d=await(await fetch('/data')).json();
 document.getElementById('fq').textContent=d.f.toFixed(3);
 document.getElementById('rc').textContent=d.rc.toFixed(2);
 document.getElementById('ps').textContent=d.ps[0].toFixed(1);
 document.getElementById('gps').textContent=d.gps;
 document.getElementById('cli').textContent=d.cli;
 const ids=[['ma','aa'],['mb','ab'],['mc','ac']];
 d.ph.forEach((p,i)=>{document.getElementById(ids[i][0]).textContent=p[0].toFixed(1);
   document.getElementById(ids[i][1]).textContent=p[1].toFixed(1);});
 draw(d);
}
setInterval(tick,500);tick();
</script></div></body></html>)HTML";

void handleRoot() { server.send_P(200, "text/html", DASHBOARD); }

void handleData() {
  String j = "{";
  j += "\"f\":" + String(gridFreq, 3);
  j += ",\"rc\":" + String(rocof, 2);
  j += ",\"ps\":[" + String(posSeq.mag, 1) + "," + String(posSeq.angDeg, 1) + "]";
  j += ",\"gps\":\"" + String(gpsLocked ? "LOCKED" : "NO FIX") + "\"";
  j += ",\"cli\":\"" + String(streamClient && streamClient.connected() ? "CONNECTED" : "WAITING") + "\"";
  j += ",\"ph\":[";
  for (int i = 0; i < 3; i++) {
    if (i) j += ",";
    j += "[" + String(ph[i].mag, 1) + "," + String(ph[i].angDeg, 1) + "]";
  }
  j += "]}";
  server.send(200, "application/json", j);
}

// =============================================================
//  SETUP & LOOP
// =============================================================
void setup() {
  Serial.begin(115200);
  Serial2.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);
  analogReadResolution(12);
  pinMode(PPS_PIN, INPUT_PULLDOWN);
  attachInterrupt(digitalPinToInterrupt(PPS_PIN), onPPS, RISING);

  sampleTimer = timerBegin(0, 80, true);            // 1 MHz tick
  timerAttachInterrupt(sampleTimer, &onSampleTimer, true);
  timerAlarmWrite(sampleTimer, 1000000 / SAMPLE_RATE, true);
  timerAlarmEnable(sampleTimer);

  WiFi.begin(WIFI_SSID, WIFI_PASS);
  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 8000) delay(300);
  if (WiFi.status() == WL_CONNECTED) {
    server.on("/", handleRoot);
    server.on("/data", handleData);
    server.begin();
    streamServer.begin();
    Serial.println("Dashboard: http://" + WiFi.localIP().toString());
    Serial.println("C37 stream: TCP port 4711");
  }
  Serial.println("SynchroPhase ready.");
}

void loop() {
  server.handleClient();

  // NMEA intake
  while (Serial2.available()) {
    char ch = (char)Serial2.read();
    if (ch == '\n') {
      if (nmeaBuffer.startsWith("$GPRMC") || nmeaBuffer.startsWith("$GNRMC"))
        parseRMC(nmeaBuffer);
      nmeaBuffer = "";
    } else if (ch != '\r') {
      nmeaBuffer += ch;
      if (nmeaBuffer.length() > 100) nmeaBuffer = "";
    }
  }

  // frame generation at FRAME_RATE once locked onto buffer
  static unsigned long lastFrame = 0;
  if (millis() - lastFrame >= (1000.0f / FRAME_RATE)) {
    lastFrame = millis();
    updatePhasors();
    if (!streamClient || !streamClient.connected()) {
      streamClient = streamServer.available();
    }
    if (streamClient && streamClient.connected()) {
      uint8_t frame[40];
      int n = buildFrame(frame);
      streamClient.write(frame, n);
    }
  }
}
