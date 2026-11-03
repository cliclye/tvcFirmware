#!/usr/bin/env python3
"""
A fake rocket on the ground link, for trying Studio's Live telemetry page without hardware.

Flies the rocket from Studio's Rocket setup in the 3D sim (the real flight software in the loop),
then streams that flight as E1 frames over UDP, the way DroneBridge forwards the board's USART2:

    python3 scripts/fake_link.py                    # to Studio on 127.0.0.1:14550, real time
    python3 scripts/fake_link.py --loss 0.1 --speed 2 --corrupt 0.01
    python3 scripts/fake_link.py --idle --session "~/Documents/EasyTVC Flights/session_7.bin"   # board tools

In Studio: Board → Live data → UDP, bridge address 127.0.0.1, port 14551 (the fake board's port),
listen on 14550. It answers HELLO / PING / STATUS / CFG / LOGINFO / LOGREAD like the firmware, and,
like the firmware, ignores everything but HELLO / PING once the "rocket" has left IDLE.
"""

from __future__ import annotations

import argparse
import base64
import math
from pathlib import Path
import random
import select
import socket
import sys
import time
import zlib

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "studio"))
import groundlink  # noqa: E402


def _r(value: float) -> int:
    """C lroundf: round half away from zero."""
    return int(math.copysign(math.floor(abs(value) + 0.5), value)) if math.isfinite(value) else 0


def format_tlm(t: dict) -> str:
    """Same text as EasyTVC_LinkFormatTlm (Core/Src/link.c); the tests check they match."""
    v = [t["ms"], t["state"], t["since_launch_ms"]]
    v += [_r(x * 1e4) for x in t["q"]] + [_r(x * 1e3) for x in t["gyro_rad_s"]]
    v += [_r(x * 1000 / 9.80665) for x in t["accel_m_s2"]]
    v += [_r(t["altitude_m"] * 100), _r(t["vertical_speed_m_s"] * 100)]
    v += [_r(x * 5729.578) for x in t["tilt_rad"]] + [_r(x * 1000) for x in t["command"]]
    v += list(t["pulse_us"]) + [t["flags"], t.get("deploy_eta_ms", -1), t.get("pressure_pa", 0), t.get("baro_temp_cdeg", 0)]
    return ",".join(str(int(x)) for x in v)


def servo_fraction(config: dict, servo: str, pulse_us: float) -> float:
    """Inverse of EasyTVC_ServoPulse (Core/Src/settings.c): pulse -> command, -1..1."""
    k = servo.upper()
    center = config[f"SERVO_{k}_CENTER_US"]
    span = config[f"SERVO_{k}_MAX_US"] - center if pulse_us >= center else center - config[f"SERVO_{k}_MIN_US"]
    return config[f"SERVO_{k}_DIRECTION"] * (pulse_us - center) / max(1, span)


def sim_telemetry(native=None):
    """(time_s, TLM dict, state) at 20 Hz from one simulated flight of the saved rocket."""
    import studio
    import launch
    native = native or studio.NativeController()
    config = launch.read_config(launch.find_project())
    vehicle = launch.upgrade_vehicle(launch.load_state().get("vehicle") or {})
    result = launch.run_sim3d(native, config, vehicle, launch.sim_defaults())
    col = {c: i for i, c in enumerate(result["columns"])}
    out, launch_t, next_t, last_state = [], None, 0.0, None
    for r in result["rows"]:
        state = int(r[col["state"]])
        if r[0] < next_t and state == last_state:
            continue                     # the sim writes 50 Hz rows; the board sends TLM at 20 Hz
        next_t, last_state = r[0] + 0.05, state   # (and an event on every state change)
        if state >= 2 and launch_t is None:
            launch_t = r[0]
        cmd = [servo_fraction(config, k, r[col[f"pulse_{k}"]]) for k in ("a", "b")]
        q = [r[col["qw"]], r[col["qx"]], r[col["qy"]], r[col["qz"]]]
        since = None if launch_t is None else r[0] - launch_t
        chute = bool(r[col["chute"]])
        eta = -1 if since is None else 0 if chute else max(0, int(config["CHUTE_BACKUP_MS"] - since * 1000))
        pressure = int(101325 * (1 - 2.25577e-5 * (r[col["z"]] + 120)) ** 5.25588)   # pad at 120 m
        tlm = {"ms": int(r[0] * 1000), "state": state, "deploy_eta_ms": eta, "pressure_pa": pressure, "baro_temp_cdeg": 2150,
               "since_launch_ms": -1 if launch_t is None else int((r[0] - launch_t) * 1000), "q": q,
               "gyro_rad_s": [0.0, 0.0, math.radians(r[col["roll_rate"]])],
               "accel_m_s2": [0.0, 0.0, 9.80665 if state < 2 else r[col["thrust"]] / 0.98 + 0.5],
               "altitude_m": r[col["z"]], "vertical_speed_m_s": r[col["vz"]],
               "tilt_rad": [math.radians(r[col["est_tilt_x"]]), math.radians(r[col["est_tilt_y"]])],
               "command": cmd, "pulse_us": [int(r[col["pulse_a"]]), int(r[col["pulse_b"]]),
                                            2000 if r[col["chute"]] else 1500],
               "flags": 0x7 | (0x20 if r[col["tvc"]] else 0) | (0x80 if r[col["chute"]] else 0) |
                        (0x100 if r[col["armed"]] else 0)}
        if state == 5 and out and out[-1][2] == 3:
            out.append((r[0], {**tlm, "state": 4}, 4))   # APOGEE lasts one 2 ms tick: the sim rows skip it
        out.append((r[0], tlm, state))
    return out, config


EVENTS = {0: "IDLE", 1: "ARMED", 2: "LAUNCH_DETECTED", 3: "BURNOUT", 4: "APOGEE", 5: "DESCENT", 6: "LANDED", 7: "FAULT"}


class FakeBoard:
    """Answers requests like Core/Src/main.c link_handle, with the same IDLE-only gate."""

    def __init__(self, config: dict, session_bytes: bytes | None = None):
        self.config, self.seq, self.state = config, 0, 0
        self.session = session_bytes or b""
        self.gimbal = {"mode": 0, "cmd": [0.0, 0.0], "trim": [0, 0], "since": 0.0}   # like servo_test.c
        self.compiled = dict(config)                  # vehicle_config.h; settings over the link override
        self.stored = None                            # what SETCFG put "on the card"
        self.pyro = {"channel": 0, "code": 0, "armed_at": 0.0}
        self.beeps = 0

    def servo_pulses(self, now: float) -> list[int]:
        """What Core/Src/servo_test.c would output on the pad right now (centred pad output + test)."""
        g, c = self.gimbal, self.config
        if self.state != 0:
            self.gimbal = {"mode": 0, "cmd": [0.0, 0.0], "trim": [0, 0], "since": 0.0}
            g = self.gimbal
        cmd = [0.0, 0.0]
        if g["mode"] == 1:
            if now - g["since"] > 1.0:
                g["mode"] = 0
            else:
                cmd = g["cmd"]
        elif g["mode"] == 2:
            step = int((now - g["since"]) / 0.3)
            if step >= 8:
                g["mode"] = 0
            else:
                cmd[step // 4] = (0.0, 1.0, -1.0, 0.0)[step % 4]
        out = []
        for i, k in enumerate("AB"):
            centre = c[f"SERVO_{k}_CENTER_US"] + g["trim"][i]
            lo, hi, d = c[f"SERVO_{k}_MIN_US"], c[f"SERVO_{k}_MAX_US"], c[f"SERVO_{k}_DIRECTION"]
            x = max(-1.0, min(1.0, cmd[i])) * d
            out.append(int(centre + (hi - centre if x >= 0 else centre - lo) * x + 0.5))
        return out

    def frame(self, ident: str, payload: str) -> bytes:
        f = groundlink.encode_frame(self.seq, ident, payload)
        self.seq += 1
        return f

    def answer(self, line: bytes) -> bytes | None:
        req = groundlink.decode_frame(line)
        if not req:
            return None
        seq, cmd, payload = req
        if cmd not in groundlink.ALWAYS_ALLOWED and self.state != 0:
            return None                      # in flight: ignored, like the firmware
        c = self.config
        if cmd == "HELLO":
            reply = "EASYTVC,2,fake,0.3.0,sim"
        elif cmd == "PING":
            reply = f"PONG,{int(time.time() * 1000) & 0x7fffffff}"
        elif cmd == "STATUS":
            reply = f"{self.state},0,0,1,1,1"
        elif cmd == "CFG":
            vals = []
            for name, kind in groundlink.CFG_FIELDS:
                v = int(self.stored is not None) if name == "RADIO_SETTINGS" else c.get(name, 0)   # lock: 0, like the board
                vals.append(groundlink.AXES.index(v) + 1 if kind == "axis" else _r(v * 1000) if kind == "m" else int(v))
            reply = ",".join(map(str, vals))
        elif cmd in ("SERVO", "SWEEP", "CENTER", "TRIM"):
            g = self.gimbal
            try:
                a, b = (int(x) for x in payload.split(",")) if cmd in ("SERVO", "TRIM") else (0, 0)
            except ValueError:
                return groundlink.encode_frame(seq, "ERR", "PAYLOAD")
            if cmd == "SERVO":
                if max(abs(a), abs(b)) > 1000:
                    return groundlink.encode_frame(seq, "ERR", "PAYLOAD")
                g.update(mode=1, cmd=[a / 1000, b / 1000], since=time.monotonic())
            elif cmd == "SWEEP":
                g.update(mode=2, since=time.monotonic())
            elif cmd == "CENTER":
                g["mode"] = 0
            else:
                if max(abs(a), abs(b)) > 100:
                    return groundlink.encode_frame(seq, "ERR", "PAYLOAD")
                g["trim"] = [a, b]
            reply = f"{g['mode']},{g['trim'][0]},{g['trim'][1]}"
        elif cmd == "SETCFG":
            try:
                wire = [int(x) for x in payload.split(",")]
            except ValueError:
                return groundlink.encode_frame(seq, "ERR", "PAYLOAD")
            if len(wire) != len(groundlink.TUNABLE_FIELDS):
                return groundlink.encode_frame(seq, "ERR", "PAYLOAD")
            self.stored = wire
            for (name, scale), x in zip(groundlink.TUNABLE_FIELDS, wire):
                self.config[name] = x / scale
            reply = ",".join(map(str, wire))
        elif cmd == "CFGRESET":
            self.stored = None
            self.config.update(self.compiled)
            reply = ",".join(str(round(self.config.get(n, 0) * k)) for n, k in groundlink.TUNABLE_FIELDS)
        elif cmd == "PYROARM":
            if payload not in ("1", "3"):
                return groundlink.encode_frame(seq, "ERR", "CHANNEL")
            self.pyro = {"channel": int(payload), "code": random.randint(1000, 9999), "armed_at": time.monotonic()}
            reply = f"{payload},{self.pyro['code']},1,0"
        elif cmd == "PYROFIRE":
            try:
                ch, code, ms = (int(x) for x in payload.split(","))
            except ValueError:
                return groundlink.encode_frame(seq, "ERR", "PAYLOAD")
            p, self.pyro = self.pyro, {"channel": 0, "code": 0, "armed_at": 0.0}
            if p["channel"] != ch:
                return groundlink.encode_frame(seq, "ERR", "NOT_ARMED_FOR_TEST")
            if code != p["code"]:
                return groundlink.encode_frame(seq, "ERR", "CODE")
            reply = f"{ch},{ms},1,0"                  # locked, like the real board until it is unlocked
        elif cmd == "BEEP":
            self.beeps += 1
            reply = "2000"
        elif cmd == "LOGINFO":
            number = int.from_bytes(self.session[8:12], "little") if self.session else 0
            reply = f"{1 if self.session else 0},{number},32,16384"
        elif cmd == "LOGREAD":
            session, offset = (int(x) for x in payload.split(","))
            chunk = self.session[offset:offset + 128].ljust(128, b"\0")
            reply = f"{session},{offset},128,{zlib.crc32(chunk):08x},{base64.b64encode(chunk).decode()}"
        else:
            return groundlink.encode_frame(seq, "ERR", "COMMAND")
        return groundlink.encode_frame(seq, "OK", reply)


def stream_idle(board, tlm, sock, target, args, rng):
    """IDLE on the pad: TLM at 10 Hz and SYS at 1 Hz, answering requests, until Ctrl-C."""
    records = max(0, (len(board.session) - 512) // 32) if board.session else 0
    number = int.from_bytes(board.session[8:12], "little") if board.session else 0
    print(f"On the pad (IDLE), streaming to {args.to}; session {number} with {records} records. Ctrl-C to stop.")
    t0, next_tlm, next_sys = time.monotonic(), 0.0, 0.0
    try:
        while True:
            now = time.monotonic() - t0
            if select.select([sock], [], [], 0.002)[0]:
                data, sender = sock.recvfrom(4096)
                for line in data.split(b"\n"):
                    reply = board.answer(line + b"\n") if line else None
                    if reply and rng.random() >= args.loss:
                        sock.sendto(reply, sender)
            frames = []
            if now >= next_tlm:
                next_tlm += 0.1
                pa, pb = board.servo_pulses(time.monotonic())
                g = board.gimbal
                busy = g["mode"] != 0 or any(g["trim"])
                frames.append(board.frame("TLM", format_tlm({**tlm, "ms": int(now * 1000), "pulse_us": [pa, pb, tlm["pulse_us"][2]],
                    "command": [servo_fraction(board.config, "a", pa), servo_fraction(board.config, "b", pb)],
                    "flags": tlm["flags"] | (0x400 if busy else 0)})))
            if now >= next_sys:
                next_sys += 1.0
                frames.append(board.frame("SYS", f"{int(now * 1000)},0,0,0,0,2010,3300,1,0,0,{int(bool(records))},{number},"
                                                 f"{records},0,0,7000,1800,0,0,0,0,0,0"))
            for f in frames:
                if rng.random() >= args.loss:
                    sock.sendto(f, target)
    except KeyboardInterrupt:
        print("Done.")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument("--to", default="127.0.0.1:14550", help="where Studio listens (host:port)")
    ap.add_argument("--port", type=int, default=14551, help="the fake board's own UDP port")
    ap.add_argument("--loss", type=float, default=0.05, help="fraction of frames to drop")
    ap.add_argument("--corrupt", type=float, default=0.005, help="fraction of frames to damage")
    ap.add_argument("--speed", type=float, default=1.0)
    ap.add_argument("--pad", type=float, default=4.0, help="seconds of pad time before ignition to send")
    ap.add_argument("--idle", action="store_true", help="stay on the pad (IDLE) until Ctrl-C, for the board tools")
    ap.add_argument("--session", type=Path, help="an SD session file (session_N.bin) for LOGREAD to serve")
    args = ap.parse_args()
    host, port = args.to.rsplit(":", 1)
    target = (host, int(port))
    flight, config = sim_telemetry()
    session = args.session.read_bytes().rstrip(b"\0") if args.session else None
    board = FakeBoard(config, session)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("0.0.0.0", args.port))
    rng = random.Random(1)
    if args.idle:
        return stream_idle(board, flight[0][1], sock, target, args, rng)
    start_t = max(flight[0][0], next(t for t, _, s in flight if s >= 2) - args.pad)
    rows = [r for r in flight if r[0] >= start_t]
    print(f"Streaming {len(rows)} TLM frames ({rows[-1][0] - rows[0][0]:.0f} s) to {args.to}, loss {args.loss:.0%}")
    t0, last_sys, last_state = time.monotonic(), -1.0, None
    for t, tlm, state in rows:
        while time.monotonic() - t0 < (t - start_t) / args.speed:
            if select.select([sock], [], [], 0.005)[0]:
                data, sender = sock.recvfrom(4096)
                for line in data.split(b"\n"):
                    reply = board.answer(line + b"\n") if line else None
                    if reply:
                        sock.sendto(reply, sender)
        board.state = state
        frames = []
        if state != last_state:
            frames += [board.frame("EVT", f"{tlm['ms']},{EVENTS[state]}")] * 1
            last_state = state
        frames.append(board.frame("TLM", format_tlm(tlm)))
        if t - last_sys >= 1.0:
            last_sys = t
            frames.append(board.frame("SYS", f"{tlm['ms']},0,{state},{int(state == 1)},{int(bool(tlm['flags'] & 0x80))},"
                                             "1234,3300,0,0,0,1,7,1000,0,0,7000,1800,0,0,0,0,0,0"))
        for f in frames:
            if rng.random() < args.loss:
                continue
            if rng.random() < args.corrupt:
                f = f[:10] + b"#" + f[11:]
            sock.sendto(f, target)
    print("Done.")


if __name__ == "__main__":
    main()
