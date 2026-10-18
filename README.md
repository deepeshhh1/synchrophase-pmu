# SynchroPhase — GPS-Synchronised Phasor Measurement Unit (PMU) 📡⚡

**A working synchrophasor unit: three-phase voltage phasors measured 50 times per second against a GPS time base, streamed in IEEE C37.118-style frames — the measurement technology behind Wide-Area Measurement Systems (WAMS) that grid operators use to watch an entire power system breathe.**

## 🎯 What a PMU is — and why it matters
A SCADA system reports voltages every few seconds, each with its own unsynchronised clock, so measurements from different substations can never be compared instant-by-instant. A PMU time-stamps every phasor with GPS time (microsecond discipline via the PPS signal), so phasors from anywhere on the grid share one clock. That is what lets operators see inter-area oscillations, true phase-angle differences across the network, and frequency dynamics in real time.

## 🧠 What SynchroPhase does
- **GPS time discipline** — NEO-M8N PPS interrupt + NMEA fixes give the sampling clock an absolute time reference
- **Synchronised sampling** — three phase voltages (scaled down safely via dividers) sampled at 4.8 kHz into ring buffers
- **Phasor estimation** — one-cycle DFT per phase at the reporting instant, with **frequency-tracked resampling** so off-nominal frequency doesn't leak into the phasor
- **Symmetrical components** — positive/negative/zero sequence from the three phasors (Fortescue transform), with voltage-unbalance % on the dashboard
- **Frequency + ROCOF** — estimated from the positive-sequence angle rotation between frames
- **C37.118-style streaming** — SYNC word, frame size, SOC + FRACSEC time stamp, phasors, FREQ/DFREQ and a CRC-CCITT check, over TCP at 50 frames/s — plus a configuration frame a client can request
- **Python client included** — `pmu_client.py` connects, resynchronises, CRC-validates and parses the stream live (protocol-tested against the firmware's frame builder)

## 📂 In this repo (multi-file firmware + client)

- `SynchroPhase_PMU.ino` — main sketch: Wi-Fi, dashboard, stream-client management
- `gps_time.h` — PPS discipline + NMEA RMC time base
- `phasor.h` — 4.8 kHz sampling, DFT phasor engine, sequence components, frequency/ROCOF
- `c37.h` — data-frame + configuration-frame builders, CRC-CCITT
- `pmu_client.py` — standalone Python 3 client for the TCP stream
- `components.md` — component list with the role of each part
- `c37-frame-format.md` — the exact frame layout this firmware emits, field by field

## 🖥️ Dashboard

Live phasor diagram (three vectors + positive sequence), magnitudes, angles, frequency, ROCOF, voltage unbalance, GPS lock state and client status — served by the ESP32.

## 📚 Required libraries

Board support (once): **esp32 by Espressif Systems** — Arduino IDE → Boards Manager.

**No external libraries to install.** The firmware uses only ESP32-core built-ins (`WiFi`, `WebServer`) — NMEA parsing and the C37.118-style framing are implemented in this repo. The PC client `pmu_client.py` needs only **Python 3 standard library** (socket/struct) — no pip installs.

## 🔌 Wiring — pin connections (ESP32)

| ESP32 pin | Connects to | Notes |
| --- | --- | --- |
| GPIO16 (RX2) | NEO-M8N GPS — TX | NMEA input, 9600 baud |
| GPIO17 (TX2) | NEO-M8N GPS — RX | GPS configuration output |
| GPIO33 | NEO-M8N 1PPS (TIMEPULSE) | Interrupt input — the microsecond time discipline comes from this pin |
| GPIO34 | ZMPT101B output — phase R | Scaled-down phase voltage, ADC1 |
| GPIO35 | ZMPT101B output — phase Y | Scaled-down phase voltage, ADC1 |
| GPIO32 | ZMPT101B output — phase B | Scaled-down phase voltage |
| GND | GPS / sensor ground | Common ground throughout |

## ⚠️ Safety

All voltage inputs are **scaled-down low-voltage signals** from divider networks on a bench setup. Never connect microcontroller ADC inputs to mains or unscaled VT secondaries; use proper instrument transformers and isolation for any real installation.

## 👤 Author

**Deepesh Chauhan** — B.Tech Electrical Engineering, RGGEC Kangra
[LinkedIn](https://www.linkedin.com/in/deepeshhh1)
