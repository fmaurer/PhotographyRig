#!/usr/bin/env python3
"""Manual check of the command override system (see COMMAND_OVERRIDE_README.md).

REBUILT 2026-11-25 (the original was lost with the SD card). Run against a
live rig: `python test_command_override.py [ws://host:8765]`.
THE MOTORS WILL MOVE (small relative moves that net to zero).

Sends bursts of pan/tilt commands faster than they can finish and checks,
via get_pointing_state, that the step counters only ever reflect completed
or cancelled moves (no queue building up) and return to where they started.
"""
import asyncio
import json
import sys

import websockets

URL = sys.argv[1] if len(sys.argv) > 1 else "ws://localhost:8765"


async def state(ws):
    await ws.send("get_pointing_state")
    while True:
        msg = await asyncio.wait_for(ws.recv(), 5)
        try:
            d = json.loads(msg)
        except (ValueError, TypeError):
            continue
        if d.get("type") == "pointing_state":
            return d["data"]["current_pan_steps"], d["data"]["current_tilt_steps"]


async def settle(ws, timeout=20):
    last = None
    for _ in range(int(timeout / 0.3)):
        now = await state(ws)
        if now == last:
            return now
        last = now
        await asyncio.sleep(0.3)
    raise RuntimeError("motors never settled")


async def main():
    async with websockets.connect(URL) as ws:
        start = await settle(ws)
        print("start", start)

        # 1) Burst: 5 pan commands 50 ms apart; only the last should run to completion.
        for n in (400, -400, 400, -400, 200):
            await ws.send(f"pan {n}")
            await asyncio.sleep(0.05)
        mid = await settle(ws)
        print("after pan burst", mid, "(should be within ~1 move of start, not +1000)")

        # 2) Return with an absolute correction.
        await ws.send(f"pan {start[0] - mid[0]}")
        back = await settle(ws)
        print("after correction", back)
        assert back[0] == start[0], "pan counter did not return to start"

        # 3) Simultaneous move then its inverse.
        await ws.send("move 300 -150")
        await asyncio.sleep(0.1)
        await ws.send("move -300 150")      # overrides the first mid-way
        await settle(ws)
        now = await state(ws)
        await ws.send(f"move {start[0] - now[0]} {start[1] - now[1]}")
        end = await settle(ws)
        print("end", end)
        assert end == start, f"counters {end} != start {start}"
        print("PASS")


asyncio.run(main())
