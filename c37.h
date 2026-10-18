// ============================================================
//  c37.h - WIRE PROTOCOL FOR SYNCHROPHASE PMU
// ------------------------------------------------------------
//  Everything that leaves the TCP port (4711) is built here.
//  Two frame types, both following IEEE C37.118.2 conventions
//  (big-endian fields, SYNC + FRAMESIZE + IDCODE + SOC +
//  FRACSEC header, CRC-CCITT trailer) with the demo scaling
//  documented in c37-frame-format.md:
//
//  DATA FRAME (SYNC = 0xAA 0x01), 38 bytes, 50 per second:
//    SYNC(2) FRAMESIZE(2) IDCODE(2) SOC(4) FRACSEC(4)
//    STAT(2)  PHASORS 3 x (int16 mag, int16 angle)
//    FREQ(2) DFREQ(2) ANALOG(2) DIGITAL(2) CHK(2)
//    Scaling: magnitude 0.01 V/count, angle 0.01 deg/count,
//    FREQ = int16 milli-Hz deviation from 50 Hz nominal,
//    DFREQ = int16 0.01 Hz/s per count.
//
//  CONFIG FRAME (SYNC = 0xAA 0x02), 74 bytes, on request:
//    This project's own minimal CFG-style frame. A client
//    asks for it by sending the single byte 0x05 on the data
//    connection; the PMU answers with one config frame, then
//    resumes the data stream. Layout:
//    SYNC(2) FRAMESIZE(2) IDCODE(2) SOC(4) FRACSEC(4)
//    TIMEBASE(4)                  2^24 counts per second
//    STATION(16)                  ASCII, space-padded
//    NUM_PHSR(2)                  number of phasor channels
//    per channel: NAME(8) ASCII space-padded,
//                 SCALE(4) uint32 micro-volts per count
//    CHK(2)
//    (Angle scaling is fixed at 0.01 deg/count for all
//    channels and is part of the protocol, not the frame.)
//
//  The builders are pure: they serialize caller-supplied
//  values into a caller-supplied buffer and return the byte
//  count. They own no sockets and no state, which keeps the
//  protocol testable off-target.
// ============================================================

#ifndef SYNCHROPHASE_C37_H
#define SYNCHROPHASE_C37_H

#include <Arduino.h>
#include <string.h>
#include "phasor.h"        // for the Phasor struct only

// ---------------- Protocol constants -------------------------
#define C37_IDCODE            0x0007   // this unit's ID
#define C37_TIMEBASE          16777216UL
#define C37_DATA_FRAME_LEN    38
#define C37_CFG_FRAME_LEN     74
#define C37_MAX_FRAME_LEN     74
#define C37_CMD_REQUEST_CFG   0x05     // client -> PMU byte

// Station identity carried in the config frame.
#define C37_STATION_NAME      "SYNCHROPHASE-01"
#define C37_NUM_CHANNELS      3
static const char* const C37_CHANNEL_NAMES[C37_NUM_CHANNELS] = {
  "VA", "VB", "VC"
};
// Magnitude scaling shared by frame and config frame:
// 0.01 V per count = 10,000 micro-volts per count.
#define C37_MAG_UV_PER_COUNT  10000UL
// Angle scaling (protocol constant): 0.01 degree per count.
#define C37_ANGLE_CENTIDEG    100.0f

// Compile-time frame-size audit: the lengths above are not
// folklore, they are the sums of the field widths. If a field
// is ever resized, these asserts force the constants - and
// the Python client - to be updated in the same commit.
static_assert(C37_DATA_FRAME_LEN ==
              2 + 2 + 2 + 4 + 4 + 2 + 3 * 4 + 2 + 2 + 2 + 2 + 2,
              "data frame length must equal its field sum (38)");
static_assert(C37_CFG_FRAME_LEN ==
              2 + 2 + 2 + 4 + 4 + 4 + 16 + 2 + 3 * (8 + 4) + 2,
              "config frame length must equal its field sum (74)");

// =============================================================
//  CRC-CCITT (seed 0xFFFF, polynomial 0x1021)
//  The bitwise form is used: frames are tiny (<= 74 bytes) and
//  built at most 50 times a second, so a lookup table would
//  buy nothing and cost flash. The Python client implements
//  the identical algorithm and rejects any frame whose CHK
//  does not match - this function is the reference.
// =============================================================
static uint16_t c37Crc(const uint8_t* data, int len) {
  uint16_t crc = 0xFFFF;
  for (int i = 0; i < len; i++) {
    crc ^= (uint16_t)data[i] << 8;
    for (int b = 0; b < 8; b++)
      crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : (crc << 1);
  }
  return crc;
}

// ---------------- Serialization helpers ----------------------
// Big-endian writers advancing a cursor through the buffer,
// mirroring the field order in the frame tables above.
struct _C37Writer {
  uint8_t* buf;
  int n;
  void u8(uint8_t v)   { buf[n++] = v; }
  void u16(uint16_t v) { buf[n++] = v >> 8; buf[n++] = v & 0xFF; }
  void i16(int16_t v)  { u16((uint16_t)v); }
  void u32(uint32_t v) {
    buf[n++] = v >> 24; buf[n++] = (v >> 16) & 0xFF;
    buf[n++] = (v >> 8) & 0xFF; buf[n++] = v & 0xFF;
  }
  // Fixed-width ASCII field, space-padded, no NUL terminator -
  // exactly how C37.118 carries station/channel names.
  void text(const char* s, int width) {
    int len = strlen(s);
    for (int i = 0; i < width; i++)
      buf[n++] = (i < len) ? (uint8_t)s[i] : (uint8_t)' ';
  }
  // Append the running CRC over everything written so far.
  void chk() { u16(c37Crc(buf, n)); }
};

// =============================================================
//  DATA FRAME BUILDER
//  soc/fracsec come from gps_time.h; phasors, freq and rocof
//  from phasor.h. 'timeLocked' drives STAT bit 0 (set = time
//  NOT locked) and the DIGITAL word's GPS bit, matching v1.
//  Returns the frame length (always C37_DATA_FRAME_LEN).
// =============================================================
static int c37BuildDataFrame(uint8_t* out,
                             uint32_t soc, uint32_t fracsec,
                             bool timeLocked,
                             const Phasor phasors[3],
                             float freqHz, float rocofHzPerSec) {
  _C37Writer w = { out, 0 };
  w.u8(0xAA); w.u8(0x01);                     // SYNC: data frame
  w.u16(C37_DATA_FRAME_LEN);                  // FRAMESIZE
  w.u16(C37_IDCODE);                          // IDCODE
  w.u32(soc);                                 // SOC
  w.u32(fracsec);                             // FRACSEC
  w.u16(timeLocked ? 0x0000 : 0x0001);        // STAT
  for (int i = 0; i < 3; i++) {
    w.u16((uint16_t)(phasors[i].mag * 100.0f));       // 0.01 V
    w.i16((int16_t)(phasors[i].angDeg * C37_ANGLE_CENTIDEG));
  }
  w.i16((int16_t)((freqHz - 50.0f) * 1000.0f));       // FREQ, mHz dev
  w.i16((int16_t)(rocofHzPerSec * 100.0f));           // DFREQ
  w.u16(0);                                          // ANALOG
  w.u16(timeLocked ? 1 : 0);                         // DIGITAL
  w.chk();                                           // CHK
  return w.n;
}

// =============================================================
//  CONFIG FRAME BUILDER
//  Serializes the unit's static description: identity, time
//  base and per-channel names + magnitude scaling, so a client
//  can self-configure instead of hard-coding the scaling.
//  Sent once per client request byte 0x05. Returns the frame
//  length (always C37_CFG_FRAME_LEN).
// =============================================================
static int c37BuildConfigFrame(uint8_t* out,
                               uint32_t soc, uint32_t fracsec) {
  _C37Writer w = { out, 0 };
  w.u8(0xAA); w.u8(0x02);                     // SYNC: config frame
  w.u16(C37_CFG_FRAME_LEN);                   // FRAMESIZE
  w.u16(C37_IDCODE);                          // IDCODE
  w.u32(soc);                                 // SOC
  w.u32(fracsec);                             // FRACSEC
  w.u32(C37_TIMEBASE);                        // TIMEBASE
  w.text(C37_STATION_NAME, 16);               // STATION
  w.u16(C37_NUM_CHANNELS);                    // NUM_PHSR
  for (int i = 0; i < C37_NUM_CHANNELS; i++) {
    w.text(C37_CHANNEL_NAMES[i], 8);          // channel name
    w.u32(C37_MAG_UV_PER_COUNT);              // uV per count
  }
  w.chk();                                    // CHK
  return w.n;
}

#endif // SYNCHROPHASE_C37_H
