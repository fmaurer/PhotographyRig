#!/usr/bin/env python3
"""Emergency stop for samwise: VACTUAL=0 + run current 0 to both TMC2209s.

Works while the app is running (opens /dev/serial0 a second time; the
unverified writes need no replies) and without it. Leaves the app running.
If it can't confirm the stop: CUT MOTOR POWER.

    python3 tools/estop.py
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import tmc2209_uart as tu  # noqa: E402

bus = tu.TMC2209Bus()
bus.zero_now(reps=5)
ok = True
for name, addr in tu.ADDR.items():
    confirmed = bus.w(addr, tu.REG_VACTUAL, 0)
    ok &= confirmed
    print(f"{name}: {'stop confirmed' if confirmed else 'NOT CONFIRMED -- CUT MOTOR POWER'}")
sys.exit(0 if ok else 1)
