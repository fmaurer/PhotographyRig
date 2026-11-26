"""Simulated samwise rig for off-rig tests: fake TMC2209 UART (real datagrams/CRC,
single-wire echo, IFCNT) + gear/encoder plant (lash, slip, noise, freeze, dropout).
No serial port, GPIO or I2C is touched."""
import math, random, sys, threading, time
import os; sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import tmc2209_uart as tu
from TMC2209Driver import TMC2209Driver


class FakeTMC:
    """Emulates two TMC2209s on a single wire: echo + 8-byte replies, IFCNT, CRC check."""
    def __init__(self):
        self.timeout = 0.05
        self.regs = {a: {tu.REG_GCONF: 0x1C1, tu.REG_GSTAT: 0x1, tu.REG_IFCNT: 0, tu.REG_CHOPCONF: 0x10000053,
                         tu.REG_DRV_STATUS: 0, tu.REG_VACTUAL: 0, tu.REG_IHOLD_IRUN: 0} for a in (0, 1)}
        self.rx = bytearray()
        self.dead = set()          # addrs that don't answer or accept
        self.lock = threading.Lock()
        self.writes = []

    def reset_input_buffer(self):
        self.rx.clear()

    def flush(self):
        pass

    def write(self, data):
        data = bytes(data)
        if len(data) > 8 and len(data) % 8 == 0:        # batched write datagrams (tmc_write_many_fast)
            for i in range(0, len(data), 8):
                self.write(data[i:i + 8])
            return
        with self.lock:
            self.rx += data                                  # single-wire echo
            if tu.tmc_crc(data[:-1]) != data[-1]:
                return
            addr, reg = data[1], data[2]
            if addr in self.dead:
                return
            if len(data) == 8 and reg & 0x80:
                r = reg & 0x7F
                v = int.from_bytes(data[3:7], "big")
                if r == tu.REG_GSTAT:
                    self.regs[addr][r] &= ~v
                else:
                    if r == tu.REG_VACTUAL and v & 0x800000:
                        v = v - (1 << 32) if v & 0x80000000 else v - (1 << 24)
                    self.regs[addr][r] = v
                self.regs[addr][tu.REG_IFCNT] = (self.regs[addr][tu.REG_IFCNT] + 1) & 0xFF
                self.writes.append((time.monotonic(), addr, r, v))
            elif len(data) == 4:
                v = self.regs[addr].get(reg, 0) & 0xFFFFFFFF
                rep = bytes([0x05, 0xFF, reg]) + v.to_bytes(4, "big")
                self.rx += rep + bytes([tu.tmc_crc(rep)])

    def read(self, n):
        with self.lock:
            out = bytes(self.rx[:n]); del self.rx[:n]
        return out

    def vact(self, a):
        v = self.regs[a][tu.REG_VACTUAL]
        return v if v < (1 << 31) else v - (1 << 32)

    def irun(self, a):
        return (self.regs[a][tu.REG_IHOLD_IRUN] >> 8) & 0x1F


class Plant(threading.Thread):
    """Motor integrates VACTUAL; output follows through a lash band, times a slip ratio; feeds encoders."""
    def __init__(self, fake, enc, lash=None, slip=None, start=None):
        super().__init__(daemon=True)
        self.fake, self.enc = fake, enc
        self.lash = lash or {"pan": 0.0, "tilt": 0.0}
        self.slip = slip or {"pan": 1.0, "tilt": 1.0}
        self.motor = dict(start or {"pan": 10.0, "tilt": -5.0})   # motor position in output deg (no lash)
        self.out = dict(self.motor)
        self.freeze = set(); self.drop = set(); self.stop = False
        self.peak_speed = {"pan": 0.0, "tilt": 0.0}
        self.speed_log = {"pan": [], "tilt": []}

    def run(self):
        last = time.monotonic()
        while not self.stop:
            now = time.monotonic(); dt = now - last; last = now
            for ax, a in tu.ADDR.items():
                v = self.fake.vact(a)
                if self.fake.irun(a) == 0:
                    v = 0                                   # no current, no motion
                dps = tu.vactual_to_out_dps(ax, v) * self.slip[ax]
                self.peak_speed[ax] = max(self.peak_speed[ax], abs(dps))
                self.speed_log[ax].append((now, dps))
                self.motor[ax] += dps * dt
                h = self.lash[ax] / 2
                if self.motor[ax] - self.out[ax] > h: self.out[ax] = self.motor[ax] - h
                if self.out[ax] - self.motor[ax] > h: self.out[ax] = self.motor[ax] + h
                if ax in self.drop:
                    continue
                if ax in self.freeze:
                    raw = self.frozen_raw[ax]
                else:
                    raw = int(round(self.out[ax] * 4096 / 360 + random.choice((-1, 0, 0, 1)))) % 4096
                    self.frozen_raw = getattr(self, "frozen_raw", {}); self.frozen_raw[ax] = raw
                self.enc.feed(ax, raw, now)
            time.sleep(0.005)


class SimEncoders(tu.AS5600Encoders):
    def _loop(self):           # never touch I2C
        return



def rig(limits=None, **kw):
    fake = FakeTMC()
    bus = tu.TMC2209Bus(ser=fake)
    enc = SimEncoders(zero_raw={"pan": 0, "tilt": 0})
    plant = Plant(fake, enc, **kw); plant.start()
    time.sleep(0.1)
    limits = limits or {}
    pan = TMC2209Driver(bus, enc, "pan", 0.00008, 2146, current_ma=1000, travel_limits=limits.get("pan"))
    tilt = TMC2209Driver(bus, enc, "tilt", 0.00035, 805, current_ma=1400, travel_limits=limits.get("tilt"))
    return fake, bus, enc, plant, pan, tilt


def stopped(fake, a):
    return fake.vact(a) == 0 and fake.irun(a) == 0
