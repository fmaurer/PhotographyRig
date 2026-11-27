"""Aircraft overlay: the publisher side of ws://<host>:8765/overlay.

The tracker PC's publisher (tools/ui_feed.py on that PC) connects here as a
client and streams {"type": "aircraft", ...} frames at ~10 Hz; operator
commands go back to it as {"type": "command", ...} and come back as
{"type": "command_reply", ...}. See the interface doc "Aircraft overlay
server - interface the tracker expects".

Browser side (server's choice): the rig UI's normal socket (path "/") gets
  {"type": "aircraft", ...}           every valid publisher frame, as sent
  {"type": "overlay_status", "connected": bool, "stale": bool, "message": str}
                                      on connect/disconnect and when frames stop
                                      for > STALE_S (hide overlays then)
  {"type": "overlay_reply", "req_id", "ok", "message", "late": bool}
                                      the publisher's reply ("ok": null when it
                                      did not answer within REPLY_WAIT_S)
and sends commands as the text line
  overlay_command {"cmd": "follow", "target": "a7f4ec"}
  overlay_command {"cmd": "follow_px", "px": [x, y], "t_video": t, "box": null, "source": "click"}
"""
import asyncio
import itertools
import json
import time

import websockets

PUBLISHER_PATH = "/overlay"
STALE_S = 1.0             # no frame for this long -> tell browsers to hide overlays
REPLY_WAIT_S = 5.0        # then show "no reply"
LATE_REPLY_KEEP_S = 30.0  # keep matching late replies this long
REPLACED_CODE = 4000

COMMANDS = {"follow", "follow_px", "stop", "hold", "release", "recenter"}
_COMMAND_EXTRA = ("px", "t_video", "box", "source")


def _num(v):
    return isinstance(v, (int, float)) and not isinstance(v, bool)


def _pt(v, allow_null=False):
    if v is None:
        return allow_null
    return isinstance(v, (list, tuple)) and len(v) == 2 and all(_num(c) for c in v)


def validate_aircraft(m):
    """Return None if the frame is usable, else the reason (sent back as error).
    Extra/unknown fields are always allowed."""
    if not _num(m.get("t_video")):
        return "t_video missing or not a number"
    if not isinstance(m.get("follow"), dict):
        return "follow missing or not an object"
    if not _pt(m.get("zoom_spot_px"), allow_null=True):
        return "zoom_spot_px must be [x, y] or null"
    ac = m.get("aircraft")
    if not isinstance(ac, list):
        return "aircraft missing or not a list"
    for i, a in enumerate(ac):
        if not isinstance(a, dict) or not isinstance(a.get("id"), str):
            return f"aircraft[{i}] has no string id"
        if not _pt(a.get("live_px"), allow_null=True):
            return f"aircraft[{i}].live_px must be [x, y] or null"
        p = a.get("path_px")
        if p is not None and not (isinstance(p, list) and all(_pt(q) for q in p)):
            return f"aircraft[{i}].path_px must be a list of [x, y] or null"
    if "detections" in m:
        d = m["detections"]
        if not isinstance(d, dict) or not isinstance(d.get("items"), list):
            return "detections must be an object with an items list"
        for i, it in enumerate(d["items"]):
            if not isinstance(it, dict) or not _pt(it.get("px")):
                return f"detections.items[{i}].px must be [x, y]"
    return None


class OverlayHub:
    """Owns the single publisher link. `broadcast(text)` is an async callable
    that sends a text frame to every browser."""

    def __init__(self, broadcast):
        self._broadcast = broadcast
        self.pub = None
        self.pub_addr = None
        self.last = None            # last valid aircraft frame (dict)
        self.last_raw = None
        self.last_rx = 0.0          # monotonic time of last valid frame
        self.stale = True
        self.pending = {}           # req_id -> future
        self.recent = {}            # req_id -> monotonic time sent (for late replies)
        self._ids = itertools.count(1)
        self._send_lock = asyncio.Lock()
        self._watch_task = None
        self.frames = 0

    # ---- status ---------------------------------------------------------
    def status_payload(self, message=""):
        return {"type": "overlay_status", "connected": self.pub is not None,
                "stale": self.stale, "message": message}

    async def _announce(self, message):
        await self._broadcast(json.dumps(self.status_payload(message)))

    async def on_browser_connect(self, ws):
        """New UI page: current status, plus the last frame if it is fresh."""
        try:
            await ws.send(json.dumps(self.status_payload("")))
            if self.last_raw is not None and not self.stale and time.monotonic() - self.last_rx < STALE_S:
                await ws.send(self.last_raw)
        except Exception:
            pass

    async def _watch(self):
        """Mark the feed stale when frames stop; forget old late-reply ids."""
        while True:
            await asyncio.sleep(0.25)
            now = time.monotonic()
            if not self.stale and now - self.last_rx > STALE_S:
                self.stale = True
                why = "no aircraft frame for %.0f s" % STALE_S if self.pub else "tracker disconnected"
                print(f"overlay: stale ({why})", flush=True)
                await self._announce(why)
            for rid, t in list(self.recent.items()):
                if now - t > LATE_REPLY_KEEP_S:
                    self.recent.pop(rid, None)

    def start(self):
        if self._watch_task is None:
            self._watch_task = asyncio.get_running_loop().create_task(self._watch())

    # ---- publisher link -------------------------------------------------
    async def _send_pub(self, ws, obj):
        async with self._send_lock:
            await ws.send(json.dumps(obj))

    async def publisher(self, ws):
        self.start()
        addr = getattr(ws, "remote_address", None)
        old, old_addr = self.pub, self.pub_addr
        self.pub, self.pub_addr = ws, addr
        if old is not None:
            print(f"overlay: publisher {addr} replaces {old_addr}", flush=True)
            try:
                await old.close(code=REPLACED_CODE, reason="replaced by a new publisher")
            except Exception:
                pass
        print(f"overlay: publisher connected from {addr}", flush=True)
        await self._announce("tracker connected")
        try:
            async for raw in ws:
                if not isinstance(raw, str):
                    await self._send_pub(ws, {"type": "error", "message": "binary frames are not accepted"})
                    continue
                try:
                    m = json.loads(raw)
                except ValueError as e:
                    await self._send_pub(ws, {"type": "error", "message": f"not JSON: {e}"})
                    continue
                if not isinstance(m, dict) or not isinstance(m.get("type"), str):
                    await self._send_pub(ws, {"type": "error", "message": "missing type"})
                    continue
                t = m["type"]
                if t == "command_reply":
                    await self._on_reply(m)
                elif t == "aircraft":
                    why = validate_aircraft(m)
                    if why:
                        await self._send_pub(ws, {"type": "error", "message": f"bad aircraft frame: {why}"})
                        continue
                    if ws is not self.pub:          # replaced while this frame was in flight
                        continue
                    self.last, self.last_raw, self.last_rx = m, raw, time.monotonic()
                    self.frames += 1
                    if self.stale:
                        self.stale = False
                        await self._announce("receiving")
                    await self._broadcast(raw)      # forwarded unchanged (keeps extra fields)
                else:
                    await self._send_pub(ws, {"type": "error", "message": f"unknown type '{t[:40]}'"})
        except websockets.exceptions.ConnectionClosed:
            pass
        finally:
            if self.pub is ws:
                self.pub = None
                self.stale = True
                for fut in self.pending.values():
                    if not fut.done():
                        fut.set_result(None)
                print(f"overlay: publisher {addr} disconnected ({self.frames} frames)", flush=True)
                await self._announce("tracker disconnected")

    async def _on_reply(self, m):
        rid = str(m.get("req_id"))
        reply = {"type": "overlay_reply", "req_id": rid, "ok": m.get("ok"),
                 "message": str(m.get("message", ""))}
        fut = self.pending.pop(rid, None)
        if fut is not None and not fut.done():
            fut.set_result(m)
            return                                  # send_command() reports it
        if rid in self.recent:
            self.recent.pop(rid, None)
            print(f"overlay: late reply {rid}: {reply['ok']} {reply['message']}", flush=True)
            await self._broadcast(json.dumps({**reply, "late": True}))
        else:
            print(f"overlay: reply for unknown req_id {rid!r} ignored", flush=True)

    # ---- commands -------------------------------------------------------
    async def send_command(self, cmd, target=None, **extra):
        """Relay one operator command. Returns the overlay_reply dict for the UI."""
        rid = "r%d" % next(self._ids)
        base = {"type": "overlay_reply", "req_id": rid, "late": False}
        ws = self.pub
        if ws is None:
            return {**base, "ok": False, "message": "the tracker is not connected"}
        if cmd not in COMMANDS:
            return {**base, "ok": False, "message": f"unknown command '{cmd}'"}
        msg = {"type": "command", "req_id": rid, "cmd": cmd, "target": target}
        msg.update({k: extra[k] for k in _COMMAND_EXTRA if k in extra})
        fut = asyncio.get_running_loop().create_future()
        self.pending[rid] = fut
        self.recent[rid] = time.monotonic()
        print(f"overlay: command {rid} {cmd} {target if target is not None else ''}"
              f"{' ' + json.dumps({k: msg[k] for k in _COMMAND_EXTRA if k in msg}) if extra else ''}", flush=True)
        try:
            await self._send_pub(ws, msg)
        except Exception as e:
            self.pending.pop(rid, None)
            self.recent.pop(rid, None)
            return {**base, "ok": False, "message": f"could not send to the tracker: {e}"}
        try:
            m = await asyncio.wait_for(asyncio.shield(fut), REPLY_WAIT_S)
        except asyncio.TimeoutError:
            self.pending.pop(rid, None)             # a late reply goes via self.recent
            return {**base, "ok": None, "message": "no reply from the tracker yet"}
        if m is None:
            self.recent.pop(rid, None)
            return {**base, "ok": False, "message": "the tracker disconnected before replying"}
        self.recent.pop(rid, None)
        return {**base, "ok": m.get("ok"), "message": str(m.get("message", ""))}

    async def browser_command(self, ws, arg_text):
        """Handle the UI line `overlay_command <json>`; replies to that browser only.
        Runs as its own task so a 5 s wait does not stall the UI socket."""
        try:
            c = json.loads(arg_text)
            if not isinstance(c, dict) or not isinstance(c.get("cmd"), str):
                raise ValueError("needs a JSON object with cmd")
        except ValueError as e:
            await ws.send(json.dumps({"type": "overlay_reply", "req_id": None, "ok": False,
                                      "late": False, "message": f"bad overlay_command: {e}"}))
            return
        extra = {k: c[k] for k in _COMMAND_EXTRA if k in c}
        reply = await self.send_command(c["cmd"], c.get("target"), **extra)
        if "ui_id" in c:                            # lets the page match its own request
            reply["ui_id"] = c["ui_id"]
        try:
            await ws.send(json.dumps(reply))
        except Exception:
            pass
