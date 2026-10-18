# Components — SynchroPhase PMU

| S.No. | Component | Purpose | Qty |
| --- | --- | --- | --- |
| 1 | ESP32 DevKit V1 (30-pin) | Phasor engine, streaming server, dashboard | 1 |
| 2 | u-blox NEO-M8N GPS Module (with PPS output) | Absolute time: PPS pulse + NMEA sentences | 1 |
| 3 | Voltage Divider Networks (precision resistors) | Scale the three phase voltages into the 0–3.3 V ADC window (with mid-rail bias) | 3 sets |
| 4 | Low-Voltage 3-Phase Source / 3× Signal Sources | Safe bench three-phase signals to measure | 1 set |
| 5 | 5 V 2 A Power Adapter | Power for ESP32 and GPS | 1 |
| 6 | GPS Antenna (active, with sky view) | Reliable fix + PPS during demonstrations | 1 |

**Time is the instrument here:** the GPS must hold a fix for the PPS discipline to be meaningful; the dashboard's GPS card shows lock state so an unlocked demo is never mistaken for synchrophasor data.
