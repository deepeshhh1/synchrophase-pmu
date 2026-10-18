#!/usr/bin/env python3
"""
pmu_client.py - Desktop client for the SynchroPhase PMU.

Connects to the PMU's TCP stream (port 4711), requests the
config frame (command byte 0x05), then decodes the 50 fps
C37.118-style data stream:

  * resynchronises on the 0xAA 0x01 / 0xAA 0x02 SYNC words,
  * validates FRAMESIZE and the CRC-CCITT trailer of every
    frame (frames that fail are dropped and counted),
  * parses SOC / FRACSEC, the three phase phasors
    (0.01 V and 0.01 degree per count), FREQ (mHz deviation
    from 50 Hz) and DFREQ (0.01 Hz/s per count),
  * prints a live one-line status per frame,
  * tracks the angle difference between phase A and the
    positive-sequence phasor (computed here, client-side,
    by symmetrical-component math - the wire carries only
    the three phase phasors, as in the real standard).

SOC as produced by this firmware is UTC seconds-of-day
(lat/lon are irrelevant to a phasor), so the clock shown is
the GPS time-of-day plus the FRACSEC fraction.

Usage:
    python3 pmu_client.py [host] [port]
Defaults: host 192.168.1.50, port 4711. Standard library
only; if matplotlib happens to be installed, a frequency
history plot is saved to pmu_frequency.png on exit.

Author: Deepesh Chauhan, B.Tech Electrical Engineering,
        RGGEC Kangra
"""

import cmath
import socket
import struct
import sys
import time

try:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    HAVE_PLOT = True
except ImportError:
    HAVE_PLOT = False

# --- Protocol constants (mirror of c37.h - keep in sync) ----
SYNC_DATA = b"\xAA\x01"
SYNC_CFG = b"\xAA\x02"
CMD_REQUEST_CFG = b"\x05"
DATA_FRAME_LEN = 38
NOMINAL_FREQ = 50.0
MAG_V_PER_COUNT = 0.01        # volts per magnitude count
ANG_DEG_PER_COUNT = 0.01      # degrees per angle count
TIMEBASE = 16777216           # FRACSEC counts per second


def crc_ccitt(data: bytes) -> int:
    """CRC-CCITT, seed 0xFFFF, poly 0x1021 - identical to the
    firmware's c37Crc(); the reference implementation there
    is bitwise, and so is this one (frames are tiny)."""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 \
                else (crc << 1) & 0xFFFF
    return crc


def wrap_deg(angle: float) -> float:
    """Wrap an angle difference into (-180, +180]."""
    while angle > 180.0:
        angle -= 360.0
    while angle <= -180.0:
        angle += 360.0
    return angle


def parse_config(frame: bytes) -> dict:
    """Parse the 74-byte CFG-style frame (SYNC 0xAA 0x02).
    Layout: header 14, TIMEBASE(4), STATION(16), NUM(2),
    then per channel NAME(8) + SCALE u32 (uV per count)."""
    timebase, = struct.unpack_from(">I", frame, 14)
    station = frame[18:34].decode("ascii", "replace").strip()
    num, = struct.unpack_from(">H", frame, 34)
    channels = []
    off = 36
    for _ in range(num):
        name = frame[off:off + 8].decode("ascii", "replace").strip()
        scale_uv, = struct.unpack_from(">I", frame, off + 8)
        channels.append((name, scale_uv / 1e6))   # -> V per count
        off += 12
    return {"timebase": timebase, "station": station,
            "channels": channels}


def parse_data(frame: bytes) -> dict:
    """Parse a 38-byte data frame (SYNC 0xAA 0x01); offsets
    follow c37-frame-format.md exactly."""
    soc, fracsec = struct.unpack_from(">II", frame, 6)
    stat, = struct.unpack_from(">H", frame, 14)
    phasors = []
    for k in range(3):
        mag_raw, ang_raw = struct.unpack_from(">hh", frame, 16 + 4 * k)
        phasors.append((mag_raw * MAG_V_PER_COUNT,
                        ang_raw * ANG_DEG_PER_COUNT))
    freq_dev_mhz, dfreq_raw = struct.unpack_from(">hh", frame, 28)
    digital, = struct.unpack_from(">H", frame, 34)
    return {
        "soc": soc,
        "frac": (fracsec & 0xFFFFFF) / TIMEBASE,
        "locked": not (stat & 0x0001),
        "phasors": phasors,
        "freq": NOMINAL_FREQ + freq_dev_mhz / 1000.0,
        "rocof": dfreq_raw * 0.01,
        "gps_digital": bool(digital & 0x0001),
    }


def positive_sequence(phasors):
    """(A + aB + a^2 C) / 3 with a = exp(j*120 deg), computed
    from the three phase phasors; returns (mag, angle_deg)."""
    z = []
    for mag, ang in phasors:
        z.append(cmath.rect(mag, cmath.pi * ang / 180.0))
    a = cmath.exp(2j * cmath.pi / 3.0)
    pos = (z[0] + a * z[1] + a * a * z[2]) / 3.0
    return abs(pos), cmath.phase(pos) * 180.0 / cmath.pi


def format_clock(soc: int, frac: float) -> str:
    """SOC is UTC seconds-of-day on this firmware."""
    soc %= 86400
    hh, rem = divmod(soc, 3600)
    mm, ss = divmod(rem, 60)
    return f"{hh:02d}:{mm:02d}:{ss:02d}.{int(frac * 1000):03d}"


def find_frame(buf: bytes):
    """Return (frame, rest) for the first complete, CRC-valid
    frame in buf, resynchronising past any garbage; frames
    with a bad CRC are skipped and counted by the caller via
    the returned 'bad' tally. Returns (None, buf) if more
    bytes are needed."""
    bad = 0
    while True:
        i_data = buf.find(SYNC_DATA)
        i_cfg = buf.find(SYNC_CFG)
        starts = [i for i in (i_data, i_cfg) if i >= 0]
        if not starts:
            return None, buf[-1:], bad       # keep a trailing 0xAA
        i = min(starts)
        if len(buf) < i + 4:
            return None, buf[i:], bad
        size, = struct.unpack_from(">H", buf, i + 2)
        if size < 20 or size > 512:          # implausible: resync
            buf = buf[i + 1:]
            bad += 1
            continue
        if len(buf) < i + size:
            return None, buf[i:], bad
        frame = buf[i:i + size]
        chk_rx, = struct.unpack_from(">H", frame, size - 2)
        if crc_ccitt(frame[:-2]) == chk_rx:
            return frame, buf[i + size:], bad
        buf = buf[i + 1:]                    # bad CRC: hunt on
        bad += 1


def main(argv):
    host = argv[1] if len(argv) > 1 else "192.168.1.50"
    port = int(argv[2]) if len(argv) > 2 else 4711

    print(f"Connecting to SynchroPhase at {host}:{port} ...")
    sock = socket.create_connection((host, port), timeout=10)
    sock.settimeout(5)
    sock.sendall(CMD_REQUEST_CFG)            # ask for the CFG frame

    buf = b""
    frames = errors = 0
    diff_sum = 0.0
    freq_history = []
    started = time.time()
    try:
        while True:
            chunk = sock.recv(4096)
            if not chunk:
                print("\nPMU closed the connection.")
                break
            buf += chunk
            while True:
                frame, buf, bad = find_frame(buf)
                errors += bad
                if frame is None:
                    break
                if frame[1] == 0x02:         # config frame
                    cfg = parse_config(frame)
                    chans = ", ".join(
                        f"{n} ({s:g} V/cnt)" for n, s in cfg["channels"])
                    print(f"Config: station={cfg['station']} "
                          f"timebase={cfg['timebase']} channels: {chans}")
                    continue
                d = parse_data(frame)
                frames += 1
                (ma, aa), (mb, ab), (mc, ac) = d["phasors"]
                _, pos_ang = positive_sequence(d["phasors"])
                diff = wrap_deg(aa - pos_ang)
                diff_sum += diff
                freq_history.append(d["freq"])
                line = (
                    f"{format_clock(d['soc'], d['frac'])}Z "
                    f"|VA| {ma:6.1f}V {aa:7.2f}°  "
                    f"|VB| {mb:6.1f}V {ab:7.2f}°  "
                    f"|VC| {mc:6.1f}V {ac:7.2f}°  "
                    f"f {d['freq']:.3f} Hz  ROCOF {d['rocof']:+.2f}  "
                    f"A-pos {diff:+6.2f}° (avg {diff_sum / frames:+6.2f}°)  "
                    f"{'GPS' if d['locked'] else 'NOLOCK'}"
                )
                print("\r" + line, end="", flush=True)
    except KeyboardInterrupt:
        print("\nStopped by user.")
    finally:
        sock.close()
        rate = frames / max(time.time() - started, 0.001)
        print(f"Frames: {frames} ({rate:.1f}/s), CRC/resync drops: {errors}")
        if HAVE_PLOT and freq_history:
            plt.figure(figsize=(10, 3))
            plt.plot(freq_history)
            plt.ylabel("Frequency (Hz)")
            plt.xlabel("Frame")
            plt.title("SynchroPhase frequency history")
            plt.tight_layout()
            plt.savefig("pmu_frequency.png")
            print("Saved pmu_frequency.png")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
