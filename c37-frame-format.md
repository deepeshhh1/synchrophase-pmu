# SynchroPhase Frame Format (C37.118-Style)

SynchroPhase emits a compact data frame modelled on the IEEE C37.118.2 synchrophasor frame. It is a **teaching implementation**: the layout and semantics follow the standard's data frame so the concepts transfer, but it is not a certified C37.118 implementation (no configuration frames, no PDC conformance testing).

## Data frame layout (38 bytes, big-endian)

| Offset | Size | Field | Value / encoding |
| --- | --- | --- | --- |
| 0 | 2 | SYNC | `0xAA 0x01` (data frame) |
| 2 | 2 | FRAMESIZE | 38 |
| 4 | 2 | IDCODE | 1 (station id) |
| 6 | 4 | SOC | Seconds since 1 Jan 1970 (GPS time) |
| 10 | 4 | FRACSEC | Fraction of second on a 2^24 time base; top byte carries quality flags (bit 0 set = time NOT locked) |
| 14 | 2 | STAT | Status word; bit 0 = time unlocked, bit 1 = GPS fix absent |
| 16 | 12 | PHASORS | VA, VB, VC — each: magnitude (uint16, 0.01 V units) + angle (int16, 0.01 degree units, −180…+180) |
| 28 | 2 | FREQ | Frequency deviation from 50 Hz, in mHz (int16) |
| 30 | 2 | DFREQ | ROCOF in 0.01 Hz/s units (int16) |
| 32 | 2 | ANALOG | Reserved (0) |
| 34 | 2 | DIGITAL | Digital status word (0 in this build) |
| 36 | 2 | CHK | CRC-CCITT (poly 0x1021, init 0xFFFF) over bytes 0–35 |

## Reading the stream
1. Connect TCP to the ESP32 on **port 4711**.
2. Resynchronise on the byte pair `AA 01`, then read FRAMESIZE bytes.
3. Verify CHK before trusting a frame; drop and resync on mismatch.
4. Convert phasors: volts = raw / 100; angle° = raw / 100.
5. Frequency = 50 + FREQ/1000 Hz; ROCOF = DFREQ/100 Hz/s.

The web dashboard on port 80 shows the same measurements as a live phasor diagram for demonstrations where no PDC client is available.
