// ============================================================
//  phasor.h - SAMPLING + PHASOR ESTIMATION FOR SYNCHROPHASE
// ------------------------------------------------------------
//  This module owns everything between the analog pins and a
//  set of UTC-ready phasors:
//
//    1. A hardware timer raises a 4.8 kHz tick (96 samples per
//       nominal 50 Hz cycle). Each tick, the three phase
//       voltages are read and pushed into ring buffers.
//    2. computePhasors() takes the newest two cycles from each
//       buffer and runs a one-cycle DFT - but first resamples
//       the window against the tracked grid frequency, so a
//       grid running at 49.7 Hz does not leak into the phasor
//       the way a fixed-window DFT would.
//    3. The three phase phasors are combined into symmetrical
//       components; the positive sequence is what grid
//       operators watch, and its angle rotation between frames
//       yields frequency and ROCOF.
//
//  Results are published in plain globals (ph, posSeq,
//  gridFreq, rocof, freqSmooth) - the same names the v1
//  single-file firmware used - so the dashboard and the
//  C37.118 frame builder read them without plumbing.
//
//  Public API:
//    phasorBegin()     - configure ADC + start the sample timer
//    phasorService()   - move pending samples into the buffers
//    computePhasors()  - refresh ph / posSeq / gridFreq / rocof
//    phasorReady()     - true once two full cycles are buffered
// ============================================================

#ifndef SYNCHROPHASE_PHASOR_H
#define SYNCHROPHASE_PHASOR_H

#include <Arduino.h>
#include <math.h>

// ---------------- Analog front end ---------------------------
// Three ZMPT101B voltage-sensor modules, one per phase, each
// producing an AC waveform biased at mid-supply (ADC_MID).
#define PH_A_PIN   34        // ZMPT101B phase R
#define PH_B_PIN   35        // ZMPT101B phase Y
#define PH_C_PIN   32        // ZMPT101B phase B

// Per-phase calibration: ADC volts -> line volts. Trim each
// against a reference meter on a known supply; they rarely
// match the nominal value to better than a percent.
static const float PHASOR_V_CAL[3] = {118.0f, 118.0f, 118.0f};
static const float PHASOR_ADC_MID = 1.65f;   // sensor DC bias

// ---------------- Sampling -----------------------------------
// 4800 Hz = 96 samples per 50 Hz cycle. 200 ms of history is
// kept: enough for the two-cycle DFT window plus resampling
// headroom, small enough to stay in fast RAM comfortably.
#define PHASOR_SAMPLE_RATE 4800              // Hz
#define PHASOR_BUF_LEN     960               // 200 ms ring buffer
#define PHASOR_CYCLE       96                // samples per DFT

static float _bufA[PHASOR_BUF_LEN];
static float _bufB[PHASOR_BUF_LEN];
static float _bufC[PHASOR_BUF_LEN];
static volatile int      _bufHead    = 0;    // next write index
static volatile uint32_t _bufSamples = 0;    // total samples ever

static hw_timer_t*       _sampleTimer = NULL;
static volatile bool     _sampleTick  = false;

// ---------------- Published results --------------------------
// A phasor in polar form: RMS magnitude in volts, angle in
// degrees, referenced to the GPS-disciplined sampling clock.
struct Phasor { float mag; float angDeg; };

static Phasor ph[3];              // phases R, Y, B
static Phasor posSeq;             // positive-sequence phasor
static Phasor negSeq;             // negative-sequence phasor
static Phasor zeroSeq;            // zero-sequence phasor
static float  unbalancePct = 0.0f; // |V-| / |V+| * 100
static float  gridFreq   = 50.0f; // latest frame-to-frame value
static float  rocof      = 0.0f;  // rate of change of frequency
static float  freqSmooth = 50.0f; // low-passed, for display

// Frequency the DFT resampler currently trusts. Starts at the
// nominal 50 Hz and is pulled toward the measured frequency
// by computePhasors() - a slow servo, never a jump.
static float _trackFreq = 50.0f;

// =============================================================
//  SAMPLING TIMER
//  The ISR only raises a flag. analogRead() is far too slow to
//  run at interrupt priority; phasorService() in loop context
//  does the actual conversions, at most one tick behind -
//  an acceptable, bounded jitter for a 4.8 kHz / 50 fps unit.
// =============================================================
static void IRAM_ATTR _onSampleTimer() { _sampleTick = true; }

// Convert one phase channel: ADC counts -> volts at the pin,
// remove the sensor's mid-supply bias, apply calibration.
static float _readPhase(int pin, int idx) {
  int raw = analogRead(pin);
  return ((raw / 4095.0f) * 3.3f - PHASOR_ADC_MID) * PHASOR_V_CAL[idx];
}

// Move one sample set (all three phases) into the rings.
static void _takeSamples() {
  _bufA[_bufHead] = _readPhase(PH_A_PIN, 0);
  _bufB[_bufHead] = _readPhase(PH_B_PIN, 1);
  _bufC[_bufHead] = _readPhase(PH_C_PIN, 2);
  _bufHead = (_bufHead + 1) % PHASOR_BUF_LEN;
  _bufSamples++;
}

// =============================================================
//  DFT PHASOR ESTIMATION
//  One nominal cycle (96 points). The tracked frequency sets
//  the resampling step: step = Fs / f_track / 96 raw samples
//  per output point, with linear interpolation between raw
//  samples - the standard low-cost PMU technique for keeping
//  the DFT bin exactly on the fundamental off-nominal.
// =============================================================
static Phasor _dftPhasor(const float* buf, int head) {
  // Gather the last 192 raw samples (two cycles) so the
  // resampler always has a full cycle plus context.
  float win[192];
  for (int i = 0; i < 192; i++) {
    int idx = (head - 192 + i + PHASOR_BUF_LEN * 2) % PHASOR_BUF_LEN;
    win[i] = buf[idx];
  }

  float step = (PHASOR_SAMPLE_RATE / _trackFreq) / (float)PHASOR_CYCLE;
  double re = 0, im = 0;
  for (int n = 0; n < PHASOR_CYCLE; n++) {
    float pos  = n * step;
    int   i0   = (int)pos;
    float frac = pos - i0;
    float x = win[i0] * (1 - frac) + win[min(i0 + 1, 191)] * frac;
    double ang = 2 * PI * n / (double)PHASOR_CYCLE;
    re += x * cos(ang);
    im -= x * sin(ang);
  }

  Phasor p;
  // DFT of a real sinusoid: |X| * 2/N is the peak amplitude.
  p.mag    = (float)(sqrt(re * re + im * im) * 2.0 / PHASOR_CYCLE);
  p.mag   /= sqrt(2.0f);                    // peak -> RMS
  p.angDeg = (float)(atan2(im, re) * 180.0 / PI);
  return p;
}

// Rectangular helper for the sequence-component algebra.
static void _toRect(const Phasor& p, float& x, float& y) {
  float r = p.angDeg * PI / 180.0f;
  x = p.mag * cos(r);
  y = p.mag * sin(r);
}

// Rotate a rectangular phasor by 'deg' degrees.
static void _rotate(float& x, float& y, float deg) {
  float r  = deg * PI / 180.0f;
  float c  = cos(r), s = sin(r);
  float nx = x * c - y * s;
  float ny = x * s + y * c;
  x = nx; y = ny;
}

// Wrap an angle difference into (-180, +180].
static float _wrapDeg(float a) {
  while (a > 180.0f)  a -= 360.0f;
  while (a < -180.0f) a += 360.0f;
  return a;
}

// =============================================================
//  COMPUTE PHASORS + FREQUENCY + ROCOF
//  Intended cadence: once per output frame (every 20 ms for
//  the 50 fps stream). Frequency comes from how far the
//  positive-sequence angle rotated since the previous call,
//  above the nominal 50 Hz rotation that the DFT reference
//  already accounts for.
// =============================================================
static void computePhasors() {
  int head = _bufHead;
  ph[0] = _dftPhasor(_bufA, head);
  ph[1] = _dftPhasor(_bufB, head);
  ph[2] = _dftPhasor(_bufC, head);

  // Symmetrical components from the three phase phasors:
  //   V+ = (A + a*B + a^2*C) / 3      a = 1 at +120 deg
  //   V- = (A + a^2*B + a*C) / 3
  //   V0 = (A + B + C) / 3
  // The originals are kept because each sequence rotates a
  // different copy; three scratch sets are needed.
  float ax, ay, bx, by, cx, cy;
  _toRect(ph[0], ax, ay);
  _toRect(ph[1], bx, by);
  _toRect(ph[2], cx, cy);

  float pbx = bx, pby = by, pcx = cx, pcy = cy;
  _rotate(pbx, pby, 120.0f);
  _rotate(pcx, pcy, 240.0f);
  posSeq.mag    = sqrt(sq((ax + pbx + pcx) / 3.0f) +
                       sq((ay + pby + pcy) / 3.0f));
  posSeq.angDeg = atan2((ay + pby + pcy) / 3.0f,
                        (ax + pbx + pcx) / 3.0f) * 180.0f / PI;

  float nbx = bx, nby = by, ncx = cx, ncy = cy;
  _rotate(nbx, nby, 240.0f);
  _rotate(ncx, ncy, 120.0f);
  negSeq.mag    = sqrt(sq((ax + nbx + ncx) / 3.0f) +
                       sq((ay + nby + ncy) / 3.0f));
  negSeq.angDeg = atan2((ay + nby + ncy) / 3.0f,
                        (ax + nbx + ncx) / 3.0f) * 180.0f / PI;

  zeroSeq.mag    = sqrt(sq((ax + bx + cx) / 3.0f) +
                        sq((ay + by + cy) / 3.0f));
  zeroSeq.angDeg = atan2((ay + by + cy) / 3.0f,
                         (ax + bx + cx) / 3.0f) * 180.0f / PI;

  // Voltage unbalance factor (VUF): the single number grid
  // codes limit (typically to 1-2% at the PCC). Published for
  // the dashboard; the wire frame stays v1-compatible.
  unbalancePct = (posSeq.mag > 0.1f)
      ? negSeq.mag / posSeq.mag * 100.0f : 0.0f;

  // Frequency + ROCOF from the angle step since last frame.
  static float         lastAng = 0;
  static bool          first   = true;
  static unsigned long lastT   = 0;
  unsigned long now = millis();
  if (!first && now > lastT) {
    float dt    = (now - lastT) / 1000.0f;
    float dAng  = _wrapDeg(posSeq.angDeg - lastAng);
    float fMeas = 50.0f + dAng / (360.0f * dt);
    float fPrev = gridFreq;
    gridFreq    = constrain(fMeas, 45.0f, 55.0f);
    rocof       = (gridFreq - fPrev) / dt;
    _trackFreq  = gridFreq;                    // servo the DFT
    freqSmooth  = 0.8f * freqSmooth + 0.2f * gridFreq;
  }
  lastAng = posSeq.angDeg;
  lastT   = now;
  first   = false;
}

// =============================================================
//  PUBLIC API
// =============================================================

// Configure the ADC and start the 4.8 kHz sampling timer.
// Timer 0 at 80 MHz / 80 prescaler = 1 us per tick, so the
// alarm value is simply the sample period in microseconds.
static void phasorBegin() {
  analogReadResolution(12);
  _sampleTimer = timerBegin(0, 80, true);
  timerAttachInterrupt(_sampleTimer, &_onSampleTimer, true);
  timerAlarmWrite(_sampleTimer,
                  1000000UL / PHASOR_SAMPLE_RATE, true);
  timerAlarmEnable(_sampleTimer);
}

// Drain the pending sample tick into the ring buffers. Call
// every main-loop pass.
static void phasorService() {
  if (_sampleTick) {
    _sampleTick = false;
    _takeSamples();
  }
}

// True once at least the two-cycle DFT window (plus margin)
// has been captured; before that the phasors are meaningless.
static bool phasorReady() {
  return _bufSamples >= 200;
}

#endif // SYNCHROPHASE_PHASOR_H
