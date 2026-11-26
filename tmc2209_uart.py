"""TMC2209 single-wire UART bus + AS5600 output-shaft encoders (samwise rig).

Adapted 2026-11-25 from ~/backupsting (motor_ui.py, test_hardware.py), the
code that was tested on this rig. Kept self-contained so the repo does not
depend on that folder. Hardware (see the samwise rig notes):

  * Both TMC2209s on /dev/serial0 @115200: addr 0 = pan, addr 1 = tilt.
    No STEP/DIR wiring; motion is the driver's internal step generator
    (VACTUAL). EN is tied to GND, so the ONLY way to stop a motor is over the
    UART (VACTUAL = 0 and/or run current = 0).
  * 1/8 microstepping set over UART (GCONF.mstep_reg_select + CHOPCONF.MRES=5),
    volatile: re-applied by setup() before every move.
  * AS5600 encoders (0x36) on the OUTPUT shafts (~80:1 gear): pan on I2C bus 1,
    tilt on bus 2. 4096 counts / rev = 0.088 deg.

Safety lessons kept from the rig (2026-11-22 runaway):
  * zero_now() sends VACTUAL=0 and current=0 WITHOUT any read first, so a stop
    still goes out when the drivers stop answering.
  * Verified writes give up after a deadline instead of retrying for seconds.
  * The encoder glitch filter caps its hold time from the FIRST suspicious
    sample (the old one could freeze the reading above ~17 deg/s).
"""

import collections
import threading
import time

PORT_DEV, BAUD = "/dev/serial0", 115200
ADDR = {"pan": 0, "tilt": 1}

TMC_SYNC = 0x05
REG_GCONF, REG_GSTAT, REG_IFCNT = 0x00, 0x01, 0x02
REG_IHOLD_IRUN = 0x10
REG_VACTUAL = 0x22
REG_CHOPCONF, REG_DRV_STATUS = 0x6C, 0x6F
TMC_FCLK = 12_000_000                    # internal oscillator (~12 MHz)

MRES = 5                                 # 1/8 microstep
USTEPS_PER_REV = 200 * (256 >> MRES)     # 1600
VACT_PER_RPM = (USTEPS_PER_REV / 60.0) / (TMC_FCLK / (1 << 24))   # ~37.3 VACTUAL units per motor rpm
GEAR = 80.0                              # motor revs per output rev (approximate)
ENC_SIGN = {"pan": -1, "tilt": +1}       # encoder-angle change for +VACTUAL (measured on the rig)
W_DEADLINE_S = 0.4

ENC_BUS = {"pan": 1, "tilt": 2}
ENC_ADDR, ENC_REG_STATUS, ENC_REG_ANGLE = 0x36, 0x0B, 0x0E
GLITCH_COUNTS = 40                       # ~3.5 deg between 5 ms samples is not physical
GLITCH_HOLD_S = 0.10


def out_dps_to_vactual(axis, dps):
    """Output-shaft deg/s (encoder sign convention) -> signed VACTUAL."""
    return ENC_SIGN[axis] * dps * GEAR / 6.0 * VACT_PER_RPM


def vactual_to_out_dps(axis, v):
    return ENC_SIGN[axis] * v / VACT_PER_RPM * 6.0 / GEAR


# ---- datagrams ------------------------------------------------------------
def tmc_crc(data):
    crc = 0
    for byte in data:
        b = byte
        for _ in range(8):
            if ((crc >> 7) ^ (b & 0x01)) & 0x01:
                crc = ((crc << 1) ^ 0x07) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
            b >>= 1
    return crc


def tmc_read_reg(ser, node, reg):
    req = bytes([TMC_SYNC, node & 0xFF, reg & 0x7F])
    req += bytes([tmc_crc(req)])
    ser.reset_input_buffer()
    ser.write(req)
    ser.flush()
    expected = len(req) + 8                      # own echo (single wire) + reply
    deadline = time.time() + (ser.timeout or 0.5)
    buf = bytearray()
    while len(buf) < expected and time.time() < deadline:
        chunk = ser.read(expected - len(buf))
        if not chunk:
            break
        buf += chunk
    data = bytes(buf)
    if len(data) < 8:
        raise IOError(f"no/short reply ({len(data)} bytes)")
    start_from = len(req) if data[:len(req)] == req else 0
    for i in range(start_from, len(data) - 7):
        if data[i] == TMC_SYNC and data[i + 1] == 0xFF and data[i + 2] == (reg & 0x7F):
            reply = data[i:i + 8]
            if tmc_crc(reply[:7]) != reply[7]:
                raise IOError("CRC mismatch")
            return (reply[3] << 24) | (reply[4] << 16) | (reply[5] << 8) | reply[6]
    raise IOError("reply sync not found")


def tmc_write_reg(ser, node, reg, value):
    value &= 0xFFFFFFFF                          # signed VACTUAL -> two's complement
    data = bytes([TMC_SYNC, node & 0xFF, (reg & 0x7F) | 0x80,
                  (value >> 24) & 0xFF, (value >> 16) & 0xFF, (value >> 8) & 0xFF, value & 0xFF])
    data += bytes([tmc_crc(data)])
    ser.reset_input_buffer()
    ser.write(data)
    ser.flush()
    time.sleep(0.01)                             # tested timing; the faster variant was set aside as unsafe


def tmc_write_datagram(node, reg, value):
    value &= 0xFFFFFFFF
    data = bytes([TMC_SYNC, node & 0xFF, (reg & 0x7F) | 0x80,
                  (value >> 24) & 0xFF, (value >> 16) & 0xFF, (value >> 8) & 0xFF, value & 0xFF])
    return data + bytes([tmc_crc(data)])


def tmc_write_many_fast(ser, writes):
    """Send several write datagrams [(addr, reg, value), ...] in one serial write
    and read back our own echo (single wire). The echo arriving means the bytes
    are on the bus, so no flush() (~7 ms tcdrain on the Pi 5) and no settle
    sleep are needed: ~1.5 ms for pan+tilt. Reading the echo keeps it out of the
    next read. Unverified like any write (the TMC2209 doesn't reply to writes).
    Returns True if the full echo came back intact."""
    data = b"".join(tmc_write_datagram(a, r, v) for a, r, v in writes)
    ser.reset_input_buffer()
    ser.write(data)
    echo = bytearray()
    deadline = time.monotonic() + 0.02
    while len(echo) < len(data) and time.monotonic() < deadline:
        chunk = ser.read(len(data) - len(echo))
        if not chunk:
            break
        echo += chunk
    return bytes(echo) == data


def current_to_cs(current_ma, rsense=0.11):
    irms = current_ma / 1000.0
    for vsense, vfs in ((1, 0.180), (0, 0.325)):
        cs = round(irms * 1.41421 * 32 * (rsense + 0.02) / vfs) - 1
        if 0 <= cs <= 31:
            return cs, vsense
    return (31, 0) if irms > 0.5 else (0, 1)


def cs_to_current(cs, vsense, rsense=0.11):
    vfs = 0.180 if vsense else 0.325
    return (cs + 1) / 32.0 * vfs / (rsense + 0.02) / 1.41421 * 1000.0


# ---- bus ------------------------------------------------------------------
class TMC2209Bus:
    """Owns /dev/serial0. One instance shared by both axes; a lock serialises traffic."""

    def __init__(self, ser=None, port=PORT_DEV, baud=BAUD):
        if ser is None:
            import serial
            ser = serial.Serial(port, baud, timeout=0.05)
        self.ser = ser
        self.lock = threading.RLock()

    def r(self, addr, reg, tries=5):
        last = None
        with self.lock:
            for _ in range(tries):
                try:
                    return tmc_read_reg(self.ser, addr, reg)
                except IOError as e:
                    last = e
                    try:
                        self.ser.reset_input_buffer()
                    except Exception:
                        pass
                    time.sleep(0.002)
        raise last

    def w(self, addr, reg, val, deadline_s=W_DEADLINE_S):
        """Verified write (IFCNT must advance by 1), bounded in time."""
        end = time.monotonic() + deadline_s
        with self.lock:
            while time.monotonic() < end:
                try:
                    c0 = tmc_read_reg(self.ser, addr, REG_IFCNT)
                    tmc_write_reg(self.ser, addr, reg, val)
                    c1 = tmc_read_reg(self.ser, addr, REG_IFCNT)
                except IOError:
                    try:
                        self.ser.reset_input_buffer()
                    except Exception:
                        pass
                    continue
                if ((c1 - c0) & 0xFF) == 1:
                    return True
                time.sleep(0.003)
        return False

    def w_fast(self, addr, reg, val):
        """Unverified write. Only for values re-sent every control cycle."""
        with self.lock:
            tmc_write_reg(self.ser, addr, reg, val)

    def w_many_fast(self, writes):
        """Fast batched write for control loops that re-send every cycle
        (tmc_write_many_fast). Stops still use zero_now() + verified writes."""
        with self.lock:
            return tmc_write_many_fast(self.ser, writes)

    def zero_now(self, addrs=None, reps=3):
        """SAFETY: unverified VACTUAL=0 + run current 0. No reads, so it works on a sick bus."""
        addrs = list(ADDR.values()) if addrs is None else list(addrs)
        with self.lock:
            for _ in range(reps):
                for a in addrs:
                    for reg in (REG_VACTUAL, REG_IHOLD_IRUN):
                        try:
                            tmc_write_reg(self.ser, a, reg, 0)
                        except Exception:
                            pass

    def setup(self, addr, ma):
        """Idempotent: UART current + 1/8 microsteps, stealthChop, driver stage on. Returns actual mA."""
        g = self.r(addr, REG_GCONF)
        g = (g | (1 << 6) | (1 << 7)) & ~((1 << 0) | (1 << 2))   # pdn_disable, mstep_reg_select; no analog I; stealthChop
        c = self.r(addr, REG_CHOPCONF)
        c = (c & ~(0xF << 24)) | (MRES << 24) | (1 << 28)         # MRES, intpol
        if (c & 0xF) == 0:
            c = (c & ~0xF) | 3                                    # TOFF so the stage is on
        cs, vsense = current_to_cs(ma)
        c = (c | (1 << 17)) if vsense else (c & ~(1 << 17))
        ok = (self.w(addr, REG_GSTAT, 0x07) and self.w(addr, REG_GCONF, g) and
              self.w(addr, REG_CHOPCONF, c) and
              self.w(addr, REG_IHOLD_IRUN, (cs // 2) | (cs << 8) | (6 << 16)))
        if not ok:
            raise RuntimeError("TMC2209 write not acknowledged (check UART / motor power)")
        return round(cs_to_current(cs, vsense))

    def stop_verified(self, addr, release=True):
        """VACTUAL=0 confirmed (after an unverified blast), then optionally current 0.
        Raises if the stop can't be confirmed."""
        self.zero_now([addr]) if release else self._vact0_blast(addr)
        ok = any(self.w(addr, REG_VACTUAL, 0) for _ in range(2))
        if release:
            self.w(addr, REG_IHOLD_IRUN, 0)
        if not ok:
            raise RuntimeError("could not confirm the motor stopped (driver not answering) -- CUT MOTOR POWER")

    def _vact0_blast(self, addr):
        with self.lock:
            for _ in range(3):
                try:
                    tmc_write_reg(self.ser, addr, REG_VACTUAL, 0)
                except Exception:
                    pass

    def health(self, addr, name):
        """Raise on over-temperature, short, or latched driver error. IOError if it doesn't answer."""
        drv = self.r(addr, REG_DRV_STATUS, tries=1)
        gs = self.r(addr, REG_GSTAT, tries=1)
        fix = "let it cool, add a heatsink/fan, or lower the current"
        if drv & 0x2:
            raise RuntimeError(f"{name} driver OVER-TEMPERATURE shutdown; {fix}")
        if drv & 0x1:
            raise RuntimeError(f"{name} driver over-temperature warning (~120 C); {fix}")
        if drv & 0x3C:
            raise RuntimeError(f"{name} driver short-circuit flag (0x{drv & 0x3C:02x}); check motor wiring")
        if gs & 0x2:
            raise RuntimeError(f"{name} driver error latched (over-temperature or short); {fix}")


# ---- encoders -------------------------------------------------------------
class AS5600Encoders:
    """Polls both AS5600s (~150 Hz) in a thread. pos_deg() = unwrapped output angle (deg) since start."""

    def __init__(self, buses=None, period=0.005, zero_raw=None):
        self.buses = dict(ENC_BUS if buses is None else buses)
        self.zero_raw = dict(zero_raw or {})        # raw count that abs_deg() calls 0
        self.first_raw = {m: None for m in self.buses}
        self.period = period
        self.lock = threading.Lock()
        self.unw = {m: None for m in self.buses}
        self.last = {m: None for m in self.buses}
        self.cand = {m: None for m in self.buses}
        self.recent = {m: collections.deque(maxlen=32) for m in self.buses}
        self.glitches = {m: 0 for m in self.buses}
        self.magnet = {m: None for m in self.buses}
        self._stop = threading.Event()
        threading.Thread(target=self._loop, daemon=True, name="AS5600").start()

    def close(self):
        self._stop.set()

    def _loop(self):
        from smbus2 import SMBus
        handles = {}
        while not self._stop.is_set():
            for m, b in self.buses.items():
                try:
                    if m not in handles:
                        handles[m] = SMBus(b)
                    st = handles[m].read_byte_data(ENC_ADDR, ENC_REG_STATUS)
                    hi, lo = handles[m].read_i2c_block_data(ENC_ADDR, ENC_REG_ANGLE, 2)
                    self.feed(m, ((hi << 8) | lo) & 0xFFF, time.monotonic(), magnet=bool(st & 0x20))
                except Exception:
                    with self.lock:
                        self.magnet[m] = None
                    try:
                        handles.pop(m).close()
                    except Exception:
                        pass
            time.sleep(self.period)

    def feed(self, m, raw, now, magnet=True):
        """One raw sample (0..4095). Public so tests can drive it."""
        with self.lock:
            raw = self._gate(m, raw, now)
            self.magnet[m] = magnet
            if self.unw[m] is None:
                self.unw[m] = 0
                self.first_raw[m] = raw
            else:
                self.unw[m] += ((raw - self.last[m] + 2048) % 4096) - 2048
            self.last[m] = raw
            if self.cand[m] is None:          # a held (suspect) sample is not fresh data
                self.recent[m].append((now, self.unw[m]))

    def _gate(self, m, raw, now):
        last = self.last[m]
        if last is None:
            return raw
        wrap = lambda a, b: ((a - b + 2048) % 4096) - 2048
        if abs(wrap(raw, last)) <= GLITCH_COUNTS:
            if self.cand[m] is not None:
                self.glitches[m] += 1
                self.cand[m] = None
            return raw
        c = self.cand[m]
        if c is None:
            self.cand[m] = (raw, now)
            return last
        if now - c[1] >= GLITCH_HOLD_S:       # SAFETY: hold capped from the FIRST suspect sample
            self.cand[m] = None
            return raw
        if abs(wrap(raw, c[0])) > GLITCH_COUNTS // 2:
            self.glitches[m] += 1
            self.cand[m] = (raw, c[1])
        return last

    def pos_deg(self, m, n=7, max_age=0.15):
        """Averaged unwrapped output angle in deg; None if the encoder is stale/missing."""
        with self.lock:
            items = list(self.recent[m])[-n:]
        if not items or time.monotonic() - items[-1][0] > max_age:
            return None
        return sum(c for _, c in items) / len(items) * 360.0 / 4096.0

    def abs_deg(self, m, n=7, max_age=0.15):
        """Absolute output angle (deg) from zero_raw: the AS5600 is absolute within
        one output turn, so this survives restarts. Continuous (unwrapped) after
        the first sample. None if stale or no zero is set for this axis."""
        z, f = self.zero_raw.get(m), self.first_raw.get(m)
        p = self.pos_deg(m, n=n, max_age=max_age)
        if z is None or f is None or p is None:
            return None
        return (((f - z + 2048) % 4096) - 2048) * 360.0 / 4096.0 + p
