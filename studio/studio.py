#!/usr/bin/env python3
"""Local EasyTVC tuning UI. Python standard library; no cloud or pip packages."""
from __future__ import annotations

import argparse
from collections import deque
import ctypes
import glob
import json
import math
import os
from pathlib import Path
import secrets
import select
import struct
import sys
import termios
import threading
import time
import tty
import webbrowser
import zlib
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

ROOT = Path(__file__).resolve().parent
AXIS_FIELDS = ("kp", "ki", "kd", "integrator_limit", "output_limit", "output_rate_limit", "derivative_cutoff_hz")
PLANT_FIELDS = ("inertia_x", "inertia_y", "thrust_n", "thrust_arm_m", "gimbal_deg", "servo_tau_s",
                "wind_m_s", "gust_m_s", "temperature_c", "area_m2", "drag_coefficient", "cp_arm_m", "initial_tilt_deg")
PLANT_BOUNDS = ((.0001, 10), (.0001, 10), (.1, 1000), (.01, 2), (.1, 15), (.005, 1),
                (0, 30), (0, 40), (-30, 60), (.0001, 1), (.01, 3), (.001, 2), (-20, 20))
STATE_NAMES = ("IDLE", "ARMED", "BOOST", "COAST", "APOGEE", "DESCENT", "LANDED", "FAULT")


def defaults():
    return {
        "schema": 1, "name": "Bench starting point — unvalidated",
        "axes": [dict(zip(AXIS_FIELDS, (1.2, 0, .2, .15, 1, 4, 20))) for _ in range(2)],
        "servos": [dict(minimum_us=1250, center_us=1500, maximum_us=1750, channel=i+1, direction=1) for i in range(2)],
        "maximum_tilt_deg": 25,
        "plant": dict(zip(PLANT_FIELDS, (.02, .02, 15, .15, 5, .08, 2, 4, 20, .008, .7, .1, 3))),
        "weather": {"maximum_gust_m_s": 5, "minimum_temperature_c": 5, "maximum_temperature_c": 35,
                    "wet": False, "limits_validated": False},
    }


def number(value, low, high, label):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or not low <= value <= high:
        raise ValueError(f"{label} must be a finite number between {low:g} and {high:g}.")
    return value


def integer(value, low, high, label):
    n = number(value, low, high, label)
    if int(n) != n:
        raise ValueError(f"{label} must be a whole number.")
    return int(n)


def exact_keys(obj, keys, label):
    if not isinstance(obj, dict) or set(obj) != set(keys):
        raise ValueError(f"{label} has missing or unsupported fields.")


def encode_settings(profile):
    """Wire encoding is explicit little-endian; never serialize C struct padding."""
    data = bytearray(b"ETC1" + struct.pack("<I", 1))
    for axis in profile["axes"]:
        data.extend(struct.pack("<7f", *(axis[k] for k in AXIS_FIELDS)))
    for servo in profile["servos"]:
        data.extend(struct.pack("<HHHBb", *(servo[k] for k in ("minimum_us", "center_us", "maximum_us", "channel", "direction"))))
    data.extend(struct.pack("<f", profile["maximum_tilt_deg"]))
    data.extend(struct.pack("<I", zlib.crc32(data)))
    return bytes(data)


def decode_settings(data):
    if len(data) != 88 or data[:8] != b"ETC1\x01\0\0\0" or struct.unpack_from("<I", data, 84)[0] != zlib.crc32(data[:84]):
        raise ValueError("The device returned an invalid settings packet.")
    return {
        "axes": [dict(zip(AXIS_FIELDS, struct.unpack_from("<7f", data, 8 + i*28))) for i in range(2)],
        "servos": [dict(zip(("minimum_us", "center_us", "maximum_us", "channel", "direction"),
                            struct.unpack_from("<HHHBb", data, 64 + i*8))) for i in range(2)],
        "maximum_tilt_deg": struct.unpack_from("<f", data, 80)[0],
    }


def validate_profile(p):
    exact_keys(p, defaults(), "Profile")
    if type(p["schema"]) is not int or p["schema"] != 1:
        raise ValueError("Unsupported profile schema.")
    if not isinstance(p["name"], str) or not 1 <= len(p["name"]) <= 80 or any(ord(c) < 32 for c in p["name"]):
        raise ValueError("Profile name must contain 1–80 printable characters.")
    if not isinstance(p["axes"], list) or len(p["axes"]) != 2 or not isinstance(p["servos"], list) or len(p["servos"]) != 2:
        raise ValueError("Exactly two axes and servos are required.")
    for i, a in enumerate(p["axes"]):
        exact_keys(a, AXIS_FIELDS, f"Axis {i+1}")
        for key in AXIS_FIELDS:
            lo, hi = (0, 100)
            if key == "output_limit": lo, hi = .01, 1
            if key == "output_rate_limit": lo = .01
            if key == "integrator_limit": hi = 1
            number(a[key], lo, hi, f"Axis {i+1}: {key}")
    for i, s in enumerate(p["servos"]):
        exact_keys(s, ("minimum_us", "center_us", "maximum_us", "channel", "direction"), f"Servo {i+1}")
        for k in ("minimum_us", "center_us", "maximum_us"):
            integer(s[k], 900, 2100, f"Servo {i+1}: {k}")
        integer(s["channel"], 1, 4, "Servo channel")
        if type(s["direction"]) is not int or s["direction"] not in (-1, 1):
            raise ValueError("Servo direction must be +1 or -1.")
        if not s["minimum_us"] + 25 <= s["center_us"] <= s["maximum_us"] - 25:
            raise ValueError("Servo endpoints need at least 25 µs on each side of center.")
    if p["servos"][0]["channel"] == p["servos"][1]["channel"]:
        raise ValueError("Each gimbal axis needs a different servo channel.")
    number(p["maximum_tilt_deg"], 5, 45, "Maximum tilt")
    exact_keys(p["plant"], PLANT_FIELDS, "Simulation model")
    for k, (lo, hi) in zip(PLANT_FIELDS, PLANT_BOUNDS):
        number(p["plant"][k], lo, hi, k)
    if p["plant"]["gust_m_s"] < p["plant"]["wind_m_s"]:
        raise ValueError("Peak gust must be at least the sustained wind speed.")
    w = p["weather"]
    exact_keys(w, defaults()["weather"], "Weather limits")
    number(w["maximum_gust_m_s"], 0, 40, "Validated gust limit")
    number(w["minimum_temperature_c"], -30, 60, "Minimum temperature")
    number(w["maximum_temperature_c"], -30, 60, "Maximum temperature")
    if w["minimum_temperature_c"] > w["maximum_temperature_c"]:
        raise ValueError("Minimum temperature must not exceed maximum temperature.")
    if type(w["wet"]) is not bool or type(w["limits_validated"]) is not bool:
        raise ValueError("Weather flags must be booleans.")
    return p


def weather_assessment(p):
    w, env = p["weather"], p["plant"]
    reasons = []
    if not w["limits_validated"]: reasons.append("Operating limits have not been validated for this vehicle.")
    if w["wet"]: reasons.append("Wet conditions: no waterproofing or wet-weather qualification is established.")
    if env["gust_m_s"] > w["maximum_gust_m_s"]: reasons.append("Peak gust exceeds your entered wind limit.")
    if not w["minimum_temperature_c"] <= env["temperature_c"] <= w["maximum_temperature_c"]:
        reasons.append("Temperature is outside your entered limits.")
    return {"within_entered_limits": not reasons, "reasons": reasons,
            "label": "Outside / unvalidated limits" if reasons else "Within entered limits — not flight clearance"}


class NativeController:
    def __init__(self, path=None):
        suffix = ".dylib" if sys.platform == "darwin" else ".so"
        candidates = [ROOT / ("libeasytvc_studio" + suffix), ROOT.parent / "build-host" / ("libeasytvc_studio" + suffix)]
        library = Path(path) if path else next((p for p in candidates if p.is_file()), candidates[-1])
        if not library.is_file():
            raise RuntimeError("Build the controller first: ./scripts/build-studio.sh")
        self.lib = ctypes.CDLL(str(library))
        self.lib.EasyTVC_StudioValidate.argtypes = [ctypes.c_char_p, ctypes.c_size_t]
        self.lib.EasyTVC_StudioValidate.restype = ctypes.c_int
        self.lib.EasyTVC_StudioSimulate.argtypes = [ctypes.c_char_p, ctypes.c_size_t,
            ctypes.POINTER(ctypes.c_float), ctypes.POINTER(ctypes.c_float), ctypes.c_uint]
        self.lib.EasyTVC_StudioSimulate.restype = ctypes.c_int

    def validate(self, profile):
        validate_profile(profile)
        packet = encode_settings(profile)
        if self.lib.EasyTVC_StudioValidate(packet, len(packet)) != 1:
            raise ValueError("The firmware rejected these settings.")
        return packet

    def simulate(self, profile):
        packet = self.validate(profile)
        count, columns = 1001, 9
        plant = (ctypes.c_float * 13)(*(profile["plant"][k] for k in PLANT_FIELDS))
        data = (ctypes.c_float * (count*columns))()
        result = self.lib.EasyTVC_StudioSimulate(packet, len(packet), plant, data, count)
        if result != count: raise ValueError("The C simulator rejected the model.")
        rows = [[round(float(data[i*columns+j]), 6) for j in range(columns)] for i in range(count)]
        if any(not math.isfinite(v) for row in rows for v in row):
            raise ValueError("The simulation produced invalid numeric output.")
        return {"rows": rows, "weather": weather_assessment(profile), "metrics": {
            "peak_tilt_deg": max(math.hypot(row[1], row[2]) for row in rows),
            "final_tilt_deg": math.hypot(rows[-1][1], rows[-1][2]),
            "saturation_percent": 100 * sum(any(abs(r[3+i]) >= profile["axes"][i]["output_limit"] * .98 for i in range(2)) for r in rows)/count,
            "fault": any(row[8] for row in rows),
        }}


def frame(sequence, command, payload="-"):
    body = f"E1 {sequence} {command} {payload}".encode("ascii")
    return body + f" *{zlib.crc32(body):08x}\n".encode("ascii")


def parse_frame(line):
    if len(line) > 255 or not line.endswith(b"\n"): raise ValueError("Invalid frame length")
    body, crc = line[:-1].rsplit(b" *", 1)
    if len(crc) != 8 or int(crc, 16) != zlib.crc32(body): raise ValueError("Frame CRC mismatch")
    version, seq, status, payload = body.decode("ascii").split(" ")
    if version != "E1" or not seq.isdecimal() or not 0 <= int(seq) <= 0xffffffff or status not in ("OK", "ERR"):
        raise ValueError("Unsupported response")
    return int(seq), status, payload


def ports():
    return sorted(set(glob.glob("/dev/cu.usbmodem*") + glob.glob("/dev/cu.usbserial*") + glob.glob("/dev/ttyACM*") + glob.glob("/dev/ttyUSB*")))


class Device:
    """Bounded POSIX serial transport. No third party driver needed for macOS CDC."""
    def __init__(self):
        self.fd = None
        self.path = None
        self.sequence = 0
        self.pending = bytearray()
        self.telemetry = deque(maxlen=2000)
        self.lock = threading.RLock()
        self.identity = None
        self.last_telemetry = None

    def close(self):
        with self.lock:
            if self.fd is not None: os.close(self.fd)
            self.fd, self.path, self.identity = None, None, None
            self.pending.clear()
            self.telemetry.clear()
            self.last_telemetry = None

    def connect(self, path):
        with self.lock:
            if not isinstance(path, str) or path not in ports():
                raise ValueError("Choose an available USB serial port.")
            self.close()
            fd = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
            self.fd, self.path = fd, path
            try:
                tty.setraw(fd)
                attributes = termios.tcgetattr(fd)
                attributes[4] = attributes[5] = termios.B115200
                attributes[2] |= termios.CLOCAL | termios.CREAD
                termios.tcsetattr(fd, termios.TCSANOW, attributes)
                termios.tcflush(fd, termios.TCIOFLUSH)
                identity = self.request("HELLO")
                if identity not in ("EASYTVC,1,VERIFIED", "EASYTVC,1,UNVERIFIED"):
                    raise ValueError("This device does not run the EasyTVC Studio protocol.")
                self.identity = identity
                return self.status()
            except Exception:
                self.close()
                raise

    def _telemetry(self, line):
        # Existing firmware telemetry: 12 CSV integer fields; SI quantities scaled by 1000.
        try:
            fields = [int(v) for v in line.decode("ascii").strip().split(",")]
            if len(fields) != 12 or not 0 <= fields[0] <= 0xffffffff or not 0 <= fields[1] < 8:
                return
            if any(v not in (0, 1) for v in fields[9:]): return
            if any(abs(v) > 2147483647 for v in fields[2:9]): return
            self.telemetry.append(fields)
            self.last_telemetry = time.monotonic()
        except (ValueError, UnicodeError):
            pass

    def request(self, command, payload="-"):
        with self.lock:
            if self.fd is None: raise ValueError("Connect a device first.")
            self.sequence = (self.sequence + 1) & 0xffffffff
            message = frame(self.sequence, command, payload)
            deadline = time.monotonic() + 2.0
            sent = 0
            try:
                while sent < len(message):
                    remaining = deadline - time.monotonic()
                    if remaining <= 0: raise TimeoutError("USB write timed out.")
                    if select.select([], [self.fd], [], remaining)[1]:
                        try: sent += os.write(self.fd, message[sent:])
                        except BlockingIOError: pass
                while time.monotonic() < deadline:
                    if not select.select([self.fd], [], [], max(0, deadline-time.monotonic()))[0]: continue
                    try: chunk = os.read(self.fd, 4096)
                    except BlockingIOError: continue
                    if not chunk: raise OSError("USB device disconnected.")
                    self.pending.extend(chunk)
                    if len(self.pending) > 16384: raise OSError("Device stream exceeds the receive limit.")
                    while b"\n" in self.pending:
                        line, _, rest = self.pending.partition(b"\n")
                        self.pending = bytearray(rest)
                        line += b"\n"
                        if not line.startswith(b"E1 "):
                            self._telemetry(line)
                            continue
                        try: sequence, status, response = parse_frame(line)
                        except (ValueError, UnicodeError): continue
                        if sequence != self.sequence: continue
                        if status == "ERR": raise ValueError(f"Device rejected command: {response}")
                        return response
                raise TimeoutError("No valid USB response within 2 seconds. Check firmware and cable.")
            except (OSError, TimeoutError):
                self.close()
                raise

    def status(self):
        with self.lock:
            if self.fd is None: return {"connected": False, "ports": ports()}
            fields = self.request("STATUS").split(",")
            if len(fields) != 3 or fields[0] not in tuple(map(str, range(8))) or any(v not in ("0", "1") for v in fields[1:]):
                self.close()
                raise ValueError("Invalid device status.")
            return {"connected": True, "port": self.path, "identity": self.identity,
                    "state": STATE_NAMES[int(fields[0])], "physical_arm": fields[1] == "1",
                    "hardware_verified": fields[2] == "1", "telemetry": list(self.telemetry),
                    "telemetry_age_s": None if self.last_telemetry is None else time.monotonic()-self.last_telemetry}

    def read_settings(self, native):
        with self.lock:
            raw = bytes.fromhex(self.request("GET"))
            if not native.lib.EasyTVC_StudioValidate(raw, len(raw)):
                raise ValueError("Device settings failed firmware validation.")
            return decode_settings(raw)

    def write_settings(self, profile, native):
        with self.lock:
            packet = native.validate(profile)
            status = self.status()
            if not status.get("connected") or not status.get("hardware_verified"):
                raise ValueError("Board hardware has not been verified; settings upload is unavailable.")
            if status["physical_arm"] or status["state"] != "IDLE":
                raise ValueError("The board must be idle and physically disarmed.")
            try:
                result = self.request("SET", packet.hex())
                if result not in ("SAVED", "UNCHANGED"): raise ValueError("Unexpected save acknowledgement.")
                if self.request("GET") != packet.hex(): raise ValueError("Settings read-back did not match.")
            except (ValueError, OSError, TimeoutError) as error:
                raise ValueError(f"Save was not verified. Reconnect and read settings before retrying. {error}") from error
            return {"message": "Saved on device and verified by reading back all settings."}


class StudioServer(ThreadingHTTPServer):
    daemon_threads = True
    def __init__(self, address, native):
        self.native, self.device, self.token = native, Device(), secrets.token_urlsafe(32)
        super().__init__(address, Handler)
        self.origin = f"http://127.0.0.1:{self.server_port}"


class Handler(BaseHTTPRequestHandler):
    server: StudioServer
    def log_message(self, *_): pass

    def send_content(self, code, data, content_type="application/json"):
        if not isinstance(data, bytes): data = json.dumps(data, allow_nan=False).encode()
        self.send_response(code)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", "no-store")
        self.send_header("X-Content-Type-Options", "nosniff")
        self.send_header("Referrer-Policy", "no-referrer")
        self.send_header("Content-Security-Policy", "default-src 'self'; script-src 'self'; style-src 'self'; connect-src 'self'; img-src 'self' data:; frame-ancestors 'none'; base-uri 'none'; form-action 'none'")
        self.end_headers()
        try: self.wfile.write(data)
        except (BrokenPipeError, ConnectionResetError): pass

    def authorized(self):
        if self.headers.get("Host") != self.server.origin.removeprefix("http://"):
            return False
        origin = self.headers.get("Origin")
        if origin is not None and origin != self.server.origin: return False
        return secrets.compare_digest(self.headers.get("Authorization", ""), "Bearer " + self.server.token)

    def do_GET(self):
        if self.headers.get("Host") != self.server.origin.removeprefix("http://"):
            self.send_content(403, {"error": "Invalid host."}); return
        static = {"/": ("index.html", "text/html; charset=utf-8"),
                  "/app.js": ("app.js", "text/javascript; charset=utf-8"),
                  "/style.css": ("style.css", "text/css; charset=utf-8")}
        if self.path in static:
            name, kind = static[self.path]
            self.send_content(200, (ROOT / "web" / name).read_bytes(), kind)
        elif self.path == "/api/defaults" and self.authorized():
            self.send_content(200, {"profile": defaults(), "ports": ports()})
        else: self.send_content(404, {"error": "Not found."})

    def do_POST(self):
        if not self.authorized(): self.send_content(403, {"error": "Invalid session."}); return
        try:
            if self.headers.get("Content-Type") != "application/json": raise ValueError("Expected JSON.")
            size = int(self.headers.get("Content-Length", "0"))
            if not 0 < size <= 32768: raise ValueError("Request size is invalid.")
            self.connection.settimeout(3)
            raw = self.rfile.read(size)
            if len(raw) != size: raise ValueError("Incomplete request.")
            data = json.loads(raw, parse_constant=lambda _: (_ for _ in ()).throw(ValueError("Non-finite JSON number.")))
            if not isinstance(data, dict): raise ValueError("Expected an object.")
            native, device = self.server.native, self.server.device
            if self.path == "/api/simulate": result = native.simulate(data)
            elif self.path == "/api/validate":
                native.validate(data); result = {"profile": data, "weather": weather_assessment(data)}
            elif self.path == "/api/ports": result = {"ports": ports()}
            elif self.path == "/api/connect": result = device.connect(data.get("port"))
            elif self.path == "/api/disconnect": device.close(); result = {"connected": False}
            elif self.path == "/api/status": result = device.status()
            elif self.path == "/api/read": result = device.read_settings(native)
            elif self.path == "/api/write": result = device.write_settings(data, native)
            elif self.path == "/api/quit":
                result = {"message": "Studio closed. You can close this tab."}
                threading.Thread(target=self.server.shutdown, daemon=True).start()
            else: self.send_content(404, {"error": "Not found."}); return
            self.send_content(200, result)
        except (ValueError, TypeError, KeyError, struct.error, OverflowError, OSError, TimeoutError) as error:
            self.send_content(400, {"error": str(error)})


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--no-browser", action="store_true")
    parser.add_argument("--port", type=int, default=0)
    args = parser.parse_args()
    try:
        server = StudioServer(("127.0.0.1", args.port), NativeController())
    except (OSError, RuntimeError) as error:
        parser.exit(1, f"EasyTVC Studio: {error}\n")
    url = server.origin + "/#" + server.token
    print(f"EasyTVC Studio: {url}", flush=True)
    print("Local simulation and tuning. Hardware firmware is not yet implemented for this board.", flush=True)
    if not args.no_browser: webbrowser.open(url)
    try: server.serve_forever(poll_interval=.2)
    except KeyboardInterrupt: pass
    finally:
        server.device.close()
        server.server_close()


if __name__ == "__main__": main()
