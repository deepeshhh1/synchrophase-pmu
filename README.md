# SynchroPhase — GPS-Synchronised Phasor Measurement Unit (PMU) 📡⚡

**A working synchrophasor unit: three-phase voltage phasors measured 50 times per second against a GPS time base, streamed in IEEE C37.118-style frames — the measurement technology behind Wide-Area Measurement Systems (WAMS) that grid operators use to watch an entire power system breathe.**

## 🎯 What a PMU is — and why it matters
A SCADA system reports voltages every few seconds, each with its own unsynchronised clock, so measurements from different substations can never be compared instant-by-instant. A PMU time-stamps every phasor with GPS time (microsecond discipline via the PPS signal), so phasors from anywhere on the grid share one clock. That is what lets operators see inter-area oscillations, true phase-angle differences across the network, and frequency dynamics in real time.

## 🧠 What SynchroPhase does
- **GPS time discipline** — NEO-M8N PPS interrupt + NMEA fixes give the sampling clock an absolute time reference
- **Synchronised sampling** — three phase voltages (scaled down safely via dividers) sampled at 4.8 kHz into ring buffers
- **Phasor estimation** — one-cycle DFT per phase at the reporting instant, with **frequency-tracked resampling** so off-nominal frequency doesn't leak into the phasor
- **Symmetrical components** — positive/negative sequence from the three phasors (Fortescue transform)
- **Frequency + ROCOF** — estimated from the positive-sequence angle rotation between frames
- **C37.118-style streaming** — SYNC word, frame size, SOC + FRACSEC time stamp, phasors, FREQ/DFREQ and a CRC-CCITT check, over TCP at 50 frames/s

## 📂 In this repo

- `SynchroPhase_PMU.ino` — complete firmware (GPS time, sampling, phasor engine, streaming, dashboard)
- `components.md` — component list with the role of each part
- `c37-frame-format.md` — the exact frame layout this firmware emits, field by field

## 🖥️ Dashboard

Live phasor diagram (three vectors + positive sequence), magnitudes, angles, frequency, ROCOF, GPS lock state and client status — served by the ESP32.

## ⚠️ Safety

All voltage inputs are **scaled-down low-voltage signals** from divider networks on a bench setup. Never connect microcontroller ADC inputs to mains or unscaled VT secondaries; use proper instrument transformers and isolation for any real installation.

## 👤 Author

**Deepesh Chauhan** — B.Tech Electrical Engineering, RGGEC Kangra
[LinkedIn](https://www.linkedin.com/in/deepeshhh1)
