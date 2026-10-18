// ============================================================
//  gps_time.h - GPS TIME DISCIPLINE FOR SYNCHROPHASE PMU
// ------------------------------------------------------------
//  A phasor is only useful if its time stamp is trustworthy.
//  This module turns a NEO-M8N (or any u-blox) GPS receiver
//  into the PMU's clock:
//
//    * The receiver's 1PPS output drives a hardware interrupt.
//      Every rising edge is, by GPS definition, the exact top
//      of a UTC second (to within tens of nanoseconds).
//    * The NMEA RMC sentences arriving on Serial2 tell us
//      WHICH second each pulse belongs to. Combining the two
//      gives the C37.118 time base: SOC (second of century -
//      here kept as UTC seconds-of-day, exactly as the v1
//      firmware produced it) plus FRACSEC counted in units of
//      the 2^24 time base.
//
//  The PPS interrupt itself does almost nothing - it bumps a
//  counter and latches micros() - because an ISR must never
//  block. All NMEA parsing happens in gpsPoll(), called from
//  the main loop.
//
//  Refinement over v1: v1 re-read socBase from every RMC
//  sentence AND added the free-running PPS count, which made
//  SOC advance twice as fast as real time. v2 latches the
//  RMC time and the PPS count at the same moment, once, and
//  from then on SOC = latched SOC + PPS pulses since latch.
//  The frame layout and scaling are unchanged.
//
//  Public API:
//    gpsBegin()      - start Serial2, arm the PPS interrupt
//    gpsPoll()       - feed NMEA bytes to the parser (loop)
//    gpsLocked()     - true once a valid RMC time was latched
//    gpsSoc()        - current UTC second (SOC field value)
//    gpsFracsec()    - fraction of current second, 2^24 base
//    gpsPpsCount()   - total PPS pulses since boot (diagnostic)
// ============================================================

#ifndef SYNCHROPHASE_GPS_TIME_H
#define SYNCHROPHASE_GPS_TIME_H

#include <Arduino.h>

// ---------------- GPS wiring ---------------------------------
// Serial2 carries NMEA; the PPS line is a separate digital pin
// because NMEA arrival time jitters by tens of milliseconds -
// far too coarse for phasor work - while PPS does not.
#define GPS_RX_PIN 16        // NEO-M8N TX -> ESP32 RX2
#define GPS_TX_PIN 17        // ESP32 TX2 -> NEO-M8N RX
#define PPS_PIN    33        // GPS 1PPS -> interrupt input

// C37.118 FRACSEC time base: one second = 16,777,216 counts.
#define GPS_TIMEBASE 16777216UL

// ---------------- Module state -------------------------------
// Written by the ISR, read everywhere else: volatile.
static volatile uint32_t _ppsCount   = 0;   // pulses since boot
static volatile uint32_t _ppsMicros  = 0;   // micros() at last pulse

static uint32_t _socLatch     = 0;          // UTC sec at latch
static uint32_t _ppsAtLatch   = 0;          // _ppsCount at latch
static bool     _timeLatched  = false;      // valid RMC received
static uint32_t _rmcSeen      = 0;          // RMC sentences parsed
static unsigned long _latchMillis = 0;      // millis() at latch

// Latest complete NMEA sentence being assembled (loop context
// only, never touched by the ISR).
static String _nmeaLine = "";

// =============================================================
//  PPS INTERRUPT
//  Fires on the rising edge of the 1PPS line: the exact UTC
//  second boundary. Keep it to two stores - anything more
//  (printing, parsing, math) would add jitter for no benefit.
// =============================================================
static void IRAM_ATTR _onPPS() {
  _ppsCount++;
  _ppsMicros = micros();
}

// =============================================================
//  NMEA RMC PARSING
//  $GPRMC / $GNRMC layout:
//    $GPRMC,hhmmss.sss,A,lat,N,lon,E,sog,cog,ddmmyy,,,*CS
//  Only the UTC time field is needed; position/validity are
//  ignored beyond requiring a complete, terminated sentence.
//  The checksum is not verified (same behaviour as v1): a
//  corrupt time field shows up as an unlatched clock, and the
//  STAT word in every data frame reports that state honestly.
// =============================================================
static void _handleNmeaSentence(const String& line) {
  if (!line.startsWith("$GPRMC") && !line.startsWith("$GNRMC")) return;

  int p1 = line.indexOf(',');
  int p2 = line.indexOf(',', p1 + 1);
  if (p1 <= 0 || p2 <= p1 + 6) return;      // no time field

  // hhmmss - ignore the fractional part, whole seconds rule
  // the PPS-aligned time base.
  int hh = line.substring(p1 + 1, p1 + 3).toInt();
  int mm = line.substring(p1 + 3, p1 + 5).toInt();
  int ss = line.substring(p1 + 5, p1 + 7).toInt();
  if (hh > 23 || mm > 59 || ss > 60) return;   // implausible

  // Latch RMC time against the PPS counter exactly once.
  // After the latch, PPS pulses are the clock - they are far
  // more precise than re-parsing jittery sentences, and the
  // latch avoids the v1 double-counting of elapsed seconds.
  _rmcSeen++;
  if (!_timeLatched) {
    _socLatch    = hh * 3600UL + mm * 60UL + ss;
    _ppsAtLatch  = _ppsCount;
    _latchMillis = millis();
    _timeLatched = true;
  }
}

// Feed every waiting Serial2 byte to the sentence assembler.
// Call once per main-loop pass; cost is bounded by the GPS's
// 9600 baud output (~1 sentence burst per second).
static void gpsPoll() {
  while (Serial2.available()) {
    char ch = (char)Serial2.read();
    if (ch == '\n') {
      _handleNmeaSentence(_nmeaLine);
      _nmeaLine = "";
    } else if (ch != '\r') {
      _nmeaLine += ch;
      // A runaway line means we lost framing; drop it rather
      // than grow the String without bound.
      if (_nmeaLine.length() > 110) _nmeaLine = "";
    }
  }
}

// =============================================================
//  PUBLIC API
// =============================================================

// Start the NMEA UART and arm the PPS interrupt. Call once in
// setup(), before anything samples time.
static void gpsBegin() {
  Serial2.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  pinMode(PPS_PIN, INPUT_PULLDOWN);
  attachInterrupt(digitalPinToInterrupt(PPS_PIN), _onPPS, RISING);
}

// True once a valid RMC time has been latched. Before that,
// frames still flow but carry STAT bit 0 set (time unlocked).
static bool gpsLocked() {
  return _timeLatched;
}

// Current UTC second for the SOC field: latched RMC second
// plus the PPS pulses counted since the latch. Each PPS pulse
// IS one second, so the sum tracks UTC one-for-one.
static uint32_t gpsSoc() {
  if (!_timeLatched) return 0;
  return _socLatch + (_ppsCount - _ppsAtLatch);
}

// Fraction of the current second in 2^24 time-base counts:
// microseconds since the last PPS edge, rescaled. The PPS
// edge is the second boundary, so this never exceeds one
// second's worth of counts while pulses keep arriving.
static uint32_t gpsFracsec() {
  uint32_t elapsedUs = micros() - _ppsMicros;
  if (elapsedUs >= 1000000UL) elapsedUs = 999999UL;  // missed pulse
  return (uint32_t)(elapsedUs * (GPS_TIMEBASE / 1000000.0));
}

// Diagnostic: raw PPS pulses since boot.
static uint32_t gpsPpsCount() {
  return _ppsCount;
}

// Diagnostic: RMC sentences parsed so far. If this climbs
// while gpsLocked() stays false, the receiver is talking but
// its time field is implausible - an antenna/sky-view issue,
// not a wiring one.
static uint32_t gpsRmcSeen() {
  return _rmcSeen;
}

// Seconds since the time latch (0 while unlatched). A lock
// that keeps re-ageing from zero would mean the latch logic
// is being re-entered - it must happen exactly once.
static uint32_t gpsLockAgeSec() {
  if (!_timeLatched) return 0;
  return (millis() - _latchMillis) / 1000UL;
}

#endif // SYNCHROPHASE_GPS_TIME_H
