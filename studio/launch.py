"""Launch-side tools for EasyTVC Studio: rocket config, build/flash, live board, flight logs.

Python standard library only. Everything runs on this Mac; nothing is uploaded anywhere.
"""
from __future__ import annotations

import bisect
from collections import OrderedDict, deque
import csv
import hashlib
import importlib.util
import io
import json
import math
import os
from pathlib import Path
import plistlib
import re
import select
import shlex
import struct
import subprocess
import sys
import tempfile
import termios
import threading
import time
import tty

HERE = Path(__file__).resolve().parent
TOOL_PATH = "/opt/homebrew/bin:/usr/local/bin:/usr/bin:/bin:/usr/sbin:/sbin"
STATES = ("IDLE", "ARMED", "BOOST", "COAST", "APOGEE", "DESCENT", "LANDED", "FAULT")
AXES = ("POS_X", "NEG_X", "POS_Y", "NEG_Y", "POS_Z", "NEG_Z")
SERVO_RATES = (50, 100, 125, 200, 250)

# Log layout, mirrors Core/Inc/data_logger.h
DIR_SECTOR, FIRST_SESSION, SESSION_SECTORS, MAX_SESSIONS = 1024, 2048, 16384, 32


def find_project(explicit: str | None = None) -> Path | None:
    """The firmware checkout: next to the .app, or the parent of studio/."""
    candidates = [Path(explicit)] if explicit else []
    if os.environ.get("EASYTVC_PROJECT"):
        candidates.append(Path(os.environ["EASYTVC_PROJECT"]))
    candidates += [HERE.parent, *list(HERE.parents)[2:3]]   # studio/.. or X.app/Contents/Resources -> ../../..
    for candidate in candidates:
        if (candidate / "Core" / "Inc" / "vehicle_config.h").is_file() and (candidate / "CMakeLists.txt").is_file():
            return candidate.resolve()
    return None


def tool_env():
    env = dict(os.environ)
    env["PATH"] = TOOL_PATH + ":" + env.get("PATH", "")
    return env


# --------------------------------------------------------------------------------------------
# vehicle_config.h
# --------------------------------------------------------------------------------------------

# (macro suffix, kind, low, high)   kinds: int, uint (U suffix), float (f suffix), axis
CONFIG_FIELDS = (
    ("NOSE_AXIS", "axis", None, None), ("LED_ACTIVE_LOW", "int", 0, 1), ("LED_BRIGHTNESS_PCT", "int", 10, 100),
    ("SERVO_A_CHANNEL", "int", 1, 4), ("SERVO_A_MIN_US", "int", 1000, 2000),
    ("SERVO_A_CENTER_US", "int", 1000, 2000), ("SERVO_A_MAX_US", "int", 1000, 2000),
    ("SERVO_A_DIRECTION", "int", -1, 1),
    ("SERVO_B_CHANNEL", "int", 1, 4), ("SERVO_B_MIN_US", "int", 1000, 2000),
    ("SERVO_B_CENTER_US", "int", 1000, 2000), ("SERVO_B_MAX_US", "int", 1000, 2000),
    ("SERVO_B_DIRECTION", "int", -1, 1),
    ("SERVO_RATE_HZ", "int", 50, 250),
    ("KP", "float", 0, 100), ("KI", "float", 0, 100), ("KD", "float", 0, 100),
    ("INTEGRATOR_LIMIT", "float", 0, 1), ("OUTPUT_RATE_LIMIT", "float", 0.01, 100),
    ("DERIVATIVE_CUTOFF", "float", 0, 100), ("MAX_TILT_DEG", "float", 5, 45),
    ("LAUNCH_ACCEL_G", "float", 1.2, 10), ("LAUNCH_CONFIRM_MS", "uint", 10, 500),
    ("TVC_ENABLE_DELAY_MS", "uint", 0, 1000),
    ("CHUTE_CHANNEL", "int", 0, 4), ("CHUTE_CLOSED_US", "int", 1000, 2000),
    ("CHUTE_OPEN_US", "int", 1000, 2000), ("CHUTE_BACKUP_MS", "uint", 1000, 30000),
    ("ARM_HOLD_MS", "uint", 1000, 30000), ("ARM_MAX_TILT_DEG", "float", 1, 30),
    ("DISARM_TILT_DEG", "float", 2, 60), ("REQUIRE_SD", "int", 0, 1),
)
FIELD_KINDS = {name: (kind, low, high) for name, kind, low, high in CONFIG_FIELDS}
DEFINE = re.compile(r"^(\s*#define\s+EASYTVC_)([A-Z0-9_]+)(\s+)(\S+)(.*)$")


def config_path(project: Path) -> Path:
    return project / "Core" / "Inc" / "vehicle_config.h"


def parse_token(name, token):
    kind = FIELD_KINDS[name][0]
    if kind == "axis":
        if not token.startswith("EASYTVC_AXIS_") or token[13:] not in AXES:
            raise ValueError(f"EASYTVC_{name} has an unknown value {token}")
        return token[13:]
    text = token.rstrip("uUfF")
    return float(text) if kind == "float" else int(text)


def format_token(name, value):
    kind = FIELD_KINDS[name][0]
    if kind == "axis":
        return f"EASYTVC_AXIS_{value}"
    if kind == "float":
        text = f"{float(value):.6g}"
        if "e" in text:
            text = f"{float(value):.6f}".rstrip("0")
        if "." not in text:
            text += ".0"
        return text + "f"
    if kind == "uint":
        return f"{int(value)}U"
    return str(int(value))


def read_config(project: Path) -> dict:
    values = {}
    for line in config_path(project).read_text().splitlines():
        match = DEFINE.match(line)
        if match and match.group(2) in FIELD_KINDS:
            values[match.group(2)] = parse_token(match.group(2), match.group(4))
    missing = [name for name in FIELD_KINDS if name not in values]
    if missing:
        raise ValueError("vehicle_config.h is missing: " + ", ".join(missing))
    return values


def validate_config(values: dict) -> dict:
    if not isinstance(values, dict) or set(values) != set(FIELD_KINDS):
        raise ValueError("The rocket configuration has missing or unknown fields.")
    clean = {}
    for name, (kind, low, high) in FIELD_KINDS.items():
        value = values[name]
        if kind == "axis":
            if value not in AXES:
                raise ValueError("Nose axis must be one of " + ", ".join(AXES))
            clean[name] = value
            continue
        if isinstance(value, bool) or not isinstance(value, (int, float)) or value != value:
            raise ValueError(f"{name} must be a number.")
        if kind in ("int", "uint") and int(value) != value:
            raise ValueError(f"{name} must be a whole number.")
        if not low <= value <= high:
            raise ValueError(f"{name} must be between {low:g} and {high:g}.")
        clean[name] = int(value) if kind in ("int", "uint") else float(value)
    for s in ("A", "B"):
        lo, mid, hi = (clean[f"SERVO_{s}_{k}_US"] for k in ("MIN", "CENTER", "MAX"))
        if not lo + 25 <= mid <= hi - 25:
            raise ValueError(f"Servo {s}: need MIN + 25 ≤ CENTER ≤ MAX − 25 µs.")
        if clean[f"SERVO_{s}_DIRECTION"] not in (-1, 1):
            raise ValueError(f"Servo {s} direction must be +1 or −1.")
    if clean["SERVO_A_CHANNEL"] == clean["SERVO_B_CHANNEL"]:
        raise ValueError("TVC servos A and B need different channels.")
    if clean["CHUTE_CHANNEL"] in (clean["SERVO_A_CHANNEL"], clean["SERVO_B_CHANNEL"]):
        raise ValueError("The parachute servo cannot share a channel with a TVC servo.")
    if clean["SERVO_RATE_HZ"] not in SERVO_RATES:
        raise ValueError("Servo rate must be one of " + ", ".join(map(str, SERVO_RATES)) + " Hz.")
    if clean["LED_BRIGHTNESS_PCT"] % 10:
        raise ValueError("LED brightness must be a multiple of 10 %.")
    if clean["DISARM_TILT_DEG"] <= clean["ARM_MAX_TILT_DEG"]:
        raise ValueError("Disarm tilt must be larger than the arming tilt.")
    return clean


def write_config(project: Path, values: dict) -> dict:
    clean = validate_config(values)
    path = config_path(project)
    lines, seen = [], set()
    for line in path.read_text().splitlines():
        match = DEFINE.match(line)
        if match and match.group(2) in clean:
            name = match.group(2)
            seen.add(name)
            line = match.group(1) + name + match.group(3) + format_token(name, clean[name]) + match.group(5)
        lines.append(line)
    if seen != set(clean):
        raise ValueError("vehicle_config.h no longer has the expected #define lines.")
    temporary = path.with_suffix(".h.tmp")
    temporary.write_text("\n".join(lines) + "\n")
    os.replace(temporary, path)
    if read_config(project) != clean:
        raise ValueError("vehicle_config.h did not read back identically.")
    return clean


# --------------------------------------------------------------------------------------------
# build / flash
# --------------------------------------------------------------------------------------------

IMAGES = ("bench", "flight", "sensortest")


def image_info(project: Path) -> dict:
    config_mtime = config_path(project).stat().st_mtime
    images = {}
    for key in IMAGES:
        path = project / "build-arm" / f"easytvc_{key}.bin"
        if path.is_file():
            data = path.read_bytes()
            images[key] = {"path": str(path), "size": len(data),
                           "sha256": hashlib.sha256(data).hexdigest(),
                           "built": path.stat().st_mtime,
                           "stale": path.stat().st_mtime < config_mtime}
        else:
            images[key] = None
    return {"images": images, "config_modified": config_mtime}


def build(project: Path) -> dict:
    try:
        result = subprocess.run([str(project / "scripts" / "build-firmware.sh")], cwd=project,
                                env=tool_env(), capture_output=True, text=True, timeout=600)
    except subprocess.TimeoutExpired:
        raise ValueError("The build took longer than 10 minutes and was stopped.")
    log = (result.stdout + result.stderr)[-20000:]
    info = image_info(project)
    info.update(ok=result.returncode == 0, log=log)
    return info


def _dfu(project: Path, *args, timeout=180):
    return subprocess.run(["python3", str(project / "scripts" / "dfu.py"), *args], cwd=project,
                          env=tool_env(), capture_output=True, text=True, timeout=timeout)


def dfu_devices(project: Path) -> dict:
    try:
        result = _dfu(project, "list", timeout=20)
    except subprocess.TimeoutExpired:
        return {"available": False, "devices": [], "message": "dfu-util did not answer."}
    text = result.stdout + result.stderr
    if result.returncode:
        return {"available": False, "devices": [], "message": text.strip()}
    spec = importlib.util.spec_from_file_location("easytvc_dfu", project / "scripts" / "dfu.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    devices = module.discover(text)
    return {"available": True, "devices": devices,
            "message": f"{len(devices)} STM32 DFU device(s) found." if devices else
                       "No board in DFU mode. Hold BOOT, tap RST (or plug in USB), release BOOT."}


def flash(project: Path, image: str, sha256: str, serial: str | None) -> dict:
    if image not in IMAGES:
        raise ValueError("Choose the bench, flight or sensor test image.")
    info = image_info(project)["images"][image]
    if info is None:
        raise ValueError("Build the firmware first.")
    if info["stale"]:
        raise ValueError("vehicle_config.h changed after this image was built. Build again first.")
    if not isinstance(sha256, str) or sha256 != info["sha256"]:
        raise ValueError("The image changed since you reviewed it. Review the new hash and try again.")
    args = ["flash", info["path"], "--sha256", sha256, "--board", "easytvc-v0.1"]
    if serial:
        args += ["--serial", str(serial)]
    try:
        result = _dfu(project, *args, timeout=240)
    except subprocess.TimeoutExpired:
        raise ValueError("Flashing timed out. Keep the board in DFU and try again.")
    log = (result.stdout + result.stderr).strip()
    if result.returncode:
        raise ValueError(log or "Flashing failed.")
    return {"ok": True, "image": image, "sha256": sha256, "log": log}


# --------------------------------------------------------------------------------------------
# live board monitor (USB-UART on TX2/RX2): read-only
# --------------------------------------------------------------------------------------------

def serial_ports():
    import glob
    return sorted(set(glob.glob("/dev/cu.usbserial*") + glob.glob("/dev/cu.usbmodem*") +
                      glob.glob("/dev/cu.SLAB*") + glob.glob("/dev/cu.wchusbserial*")))


SENSOR_CHECKS = [("accel_id", 0), ("gyro_id", 0), ("baro_id", 0), ("accel_1g", 0), ("gyro_still", 0),
                 ("imu_alive", 0), ("imu_rate", 0), ("pressure", 0), ("baro_temp", 0), ("humidity", 0),
                 ("baro_rate", 0), ("accel_temp", 0), ("sd", 0), ("supply", 0)]   # bit order of sensor_test.h


def parse_line(text: str):
    """Classify one firmware telemetry line. Returns (kind, dict) or None."""
    parts = text.strip().split(",")
    try:
        if parts[0] == "S" and len(parts) == 13:
            v = [int(x) for x in parts[1:]]
            return "status", {"imu": v[0], "baro": v[1], "sd": v[2], "gyro_cal": v[3],
                              "records": v[4], "dropped": v[5], "write_errors": v[6],
                              "session": v[7], "state": STATES[v[8]] if 0 <= v[8] < 8 else str(v[8]),
                              "armed": v[9], "recovery": v[10], "chute": v[11]}
        if parts[0] == "T" and len(parts) == 15:
            v = [int(x) for x in parts[1:]]
            return "sensors", {"ms": v[0], "accel_g": [v[1] / 1000, v[2] / 1000, v[3] / 1000],
                               "gyro_dps": [v[4] / 100, v[5] / 100, v[6] / 100], "imu_temp_c": v[7] / 100,
                               "pressure_pa": v[8], "baro_temp_c": v[9] / 100, "humidity_pct": v[10] / 10,
                               "vdda_v": v[11] / 1000, "batt_pin_v": v[12] / 1000, "mcu_temp_c": v[13] / 100}
        if parts[0] == "Q" and len(parts) == 17:
            v = [int(x) for x in parts[1:]]
            names = [key for key, _ in SENSOR_CHECKS]
            return "sensor_checks", {"ms": v[0], "checks": {n: bool(v[1] >> i & 1) for i, n in enumerate(names)},
                                     "boot": {n: bool(v[2] >> i & 1) for i, n in enumerate(names)},
                                     "ids": [v[3], v[4], v[5]], "imu_samples": v[6], "imu_errors": v[7],
                                     "baro_samples": v[8], "baro_errors": v[9], "sd": v[10], "records": v[11],
                                     "dropped": v[12], "write_errors": v[13], "session": v[14], "overruns": v[15]}
        if parts[0] == "B" and len(parts) == 10:
            v = [int(x) for x in parts[1:]]
            return "bench", {"ms": v[0], "tilt_x": v[1] / 100, "tilt_y": v[2] / 100,
                             "pulse_a": v[3], "pulse_b": v[4],
                             "raw": [v[5] / 1000, v[6] / 1000, v[7] / 1000], "body_z": v[8] / 1000}
        if len(parts) == 12:
            v = [int(x) for x in parts]
            if 0 <= v[1] < 8 and all(x in (0, 1) for x in v[9:]):
                return "flight", {"ms": v[0], "state": STATES[v[1]], "roll": v[2] / 1000,
                                  "pitch": v[3] / 1000, "yaw": v[4] / 1000, "alt": v[5] / 1000,
                                  "vspeed": v[6] / 1000, "cmd_a": v[7] / 1000, "cmd_b": v[8] / 1000,
                                  "tvc": v[9], "armed": v[10]}
    except (ValueError, IndexError):
        return None
    return None


def suggest_nose_axis(raw):
    """From raw accel (g) while the rocket stands nose-up: the axis reading ~ +/-1 g."""
    if raw is None:
        return None
    index = max(range(3), key=lambda i: abs(raw[i]))
    if not 0.85 <= abs(raw[index]) <= 1.15:
        return None
    others = [abs(raw[i]) for i in range(3) if i != index]
    if max(others) > 0.3:
        return None
    return ("POS_" if raw[index] > 0 else "NEG_") + "XYZ"[index]


class Monitor:
    def __init__(self):
        self.lock = threading.Lock()
        self.fd = None
        self.port = None
        self.thread = None
        self.latest = {}
        self.updated = {}
        self.lines = deque(maxlen=200)
        self.history = deque(maxlen=600)

    def connect(self, port):
        if port not in serial_ports():
            raise ValueError("Choose an available USB serial port.")
        self.disconnect()
        fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        try:
            tty.setraw(fd)
            attributes = termios.tcgetattr(fd)
            attributes[4] = attributes[5] = termios.B115200
            attributes[2] |= termios.CLOCAL | termios.CREAD
            termios.tcsetattr(fd, termios.TCSANOW, attributes)
            termios.tcflush(fd, termios.TCIOFLUSH)
        except OSError:
            os.close(fd)
            raise
        with self.lock:
            self.fd, self.port = fd, port
            self.latest, self.updated = {}, {}
            self.lines.clear()
            self.history.clear()
        self.thread = threading.Thread(target=self._run, args=(fd,), daemon=True)
        self.thread.start()
        return self.status()

    def disconnect(self):
        with self.lock:
            fd, self.fd, self.port = self.fd, None, None
        if fd is not None:
            try:
                os.close(fd)
            except OSError:
                pass

    def _run(self, fd):
        pending = bytearray()
        while True:
            with self.lock:
                if self.fd != fd:
                    return
            try:
                ready = select.select([fd], [], [], 0.2)[0]
                chunk = os.read(fd, 4096) if ready else b""
            except (OSError, ValueError):
                self.disconnect()
                return
            if ready and not chunk:
                self.disconnect()
                return
            pending.extend(chunk)
            if len(pending) > 65536:
                pending.clear()
            while b"\n" in pending:
                raw, _, rest = pending.partition(b"\n")
                pending = bytearray(rest)
                text = raw.decode("ascii", "replace").strip()
                if not text:
                    continue
                parsed = parse_line(text)
                with self.lock:
                    self.lines.append(text)
                    if parsed:
                        kind, data = parsed
                        self.latest[kind] = data
                        self.updated[kind] = time.monotonic()
                        if kind in ("bench", "flight"):
                            self.history.append((kind, data))

    def status(self):
        with self.lock:
            now = time.monotonic()
            bench = self.latest.get("bench")
            return {"connected": self.fd is not None, "port": self.port, "ports": serial_ports(),
                    "latest": dict(self.latest),
                    "age": {k: round(now - t, 2) for k, t in self.updated.items()},
                    "lines": list(self.lines)[-40:],
                    "suggested_nose_axis": suggest_nose_axis(bench["raw"]) if bench else None}


# --------------------------------------------------------------------------------------------
# flight logs
# --------------------------------------------------------------------------------------------

def _viewer(project: Path | None):
    candidates = ([project / "scripts" / "telemetry_viewer.py"] if project else []) + \
        [HERE / "telemetry_viewer.py", HERE.parent / "scripts" / "telemetry_viewer.py"]
    for candidate in candidates:
        if candidate.is_file():
            spec = importlib.util.spec_from_file_location("easytvc_viewer", candidate)
            module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(module)
            return module
    raise ValueError("telemetry_viewer.py is missing.")


def flights_dir(create: bool = True) -> Path:
    path = Path.home() / "Documents" / "EasyTVC Flights"
    if create:
        path.mkdir(parents=True, exist_ok=True)
    return path


SENSOR_SERIES = {"imu": ("accel_x_g", "accel_y_g", "accel_z_g", "gyro_x_dps", "gyro_y_dps", "gyro_z_dps",
                         "imu_temp_c", "read_us"),
                 "baro": ("pressure_pa", "baro_temp_c", "humidity_pct"),
                 "adc": ("vdda_v", "batt_pin_v", "mcu_temp_c", "max_loop_us")}
_sensor_sessions: "OrderedDict[str, dict]" = OrderedDict()   # parsed sessions, for zoom and CSV


def _sensor_lookup(data_id):
    if not isinstance(data_id, str) or data_id not in _sensor_sessions:
        raise ValueError("That sensor session is no longer loaded. Open it again.")
    return _sensor_sessions[data_id]


def sensor_window(data_id, t0=None, t1=None, points=6000) -> dict:
    """Every channel between t0 and t1 (s), at most `points` samples per record type, evenly picked.
    Zoomed in far enough, this is every sample the board recorded."""
    parsed = _sensor_lookup(data_id)
    points = max(100, min(20000, int(points)))
    out = {}
    for group, cols in SENSOR_SERIES.items():
        rows = parsed[group]
        if t0 is not None and t1 is not None:
            lo = bisect.bisect_left([r["t_s"] for r in rows], float(t0))
            hi = bisect.bisect_right([r["t_s"] for r in rows], float(t1))
            rows = rows[max(0, lo - 1):hi + 1]
        step = max(1, len(rows) // points)
        rows = rows[::step]
        out[group] = {"t": [r["t_s"] for r in rows], "step": step, **{c: [r[c] for r in rows] for c in cols}}
    return out


def sensor_csv_text(data_id) -> dict:
    parsed = _sensor_lookup(data_id)
    keys, rows = _viewer(None).sensor_csv(parsed)
    buffer = io.StringIO()
    writer = csv.DictWriter(buffer, fieldnames=keys)
    writer.writeheader()
    writer.writerows(rows)
    return {"name": f"easytvc_sensortest_{parsed['session']}.csv", "csv": buffer.getvalue(), "rows": len(rows)}


def summarize_sensor_session(viewer, data: bytes, source: str) -> dict:
    parsed = viewer.parse_sensor_session(data)
    report = viewer.sensor_report(parsed)
    data_id = hashlib.sha256(data).hexdigest()[:16]
    _sensor_sessions[data_id] = parsed
    _sensor_sessions.move_to_end(data_id)
    while len(_sensor_sessions) > 3:
        _sensor_sessions.popitem(last=False)
    return {"kind": "sensortest", "session": parsed["session"], "header": parsed["header"], "source": source,
            "report": report, "count": parsed["records"], "data_id": data_id,
            "series": sensor_window(data_id, points=4000)}


_raw_sessions: "OrderedDict[str, bytes]" = OrderedDict()   # recently opened sessions, for 3D replay


def _remember(data: bytes) -> str:
    data_id = hashlib.sha256(data).hexdigest()[:16]
    _raw_sessions[data_id] = data
    _raw_sessions.move_to_end(data_id)
    while len(_raw_sessions) > 4:
        _raw_sessions.popitem(last=False)
    return data_id


def summarize_session(project, data: bytes, source: str) -> dict:
    result = _summarize_session(project, data, source)
    result["data_id"] = _remember(data)
    return result


def _summarize_session(project, data: bytes, source: str) -> dict:
    viewer = _viewer(project)
    if viewer.is_sensor_session(data):
        return summarize_sensor_session(viewer, data, source)
    session, header, rows = viewer.parse_session(data)
    events, previous = [], None
    for r in rows:
        if r["state"] != previous:
            events.append({"t": r["t_s"], "event": r["state"]})
            previous = r["state"]
    opened = next((r["t_s"] for r in rows if r["chute_open"]), None)
    if opened is not None:
        events.append({"t": opened, "event": "PARACHUTE OPEN"})
    fault = next((r["t_s"] for r in rows if r["ctrl_fault"]), None)
    if fault is not None:
        events.append({"t": fault, "event": "CONTROLLER FAULT"})
    events.sort(key=lambda e: e["t"])
    boost = [r for r in rows if r["state"] == "BOOST"]
    keys = list(rows[0].keys()) if rows else []
    return {"session": session, "header": header, "source": source, "count": len(rows),
            "events": events, "columns": keys,
            "rows": [[r[k] for k in keys] for r in rows],
            "summary": {
                "duration_s": rows[-1]["t_s"] - rows[0]["t_s"] if rows else 0,
                "max_alt_m": max((r["baro_alt_m"] for r in rows), default=0),
                "flew": bool(boost),
                "worst_boost_tilt_deg": max((max(abs(r["tilt_x_deg"]), abs(r["tilt_y_deg"])) for r in boost), default=None),
                "chute_open_s": opened, "fault_s": fault,
                "dropped_data": any(r["accel_sat"] or r["gyro_sat"] for r in rows),
            }}


def external_disks():
    try:
        result = subprocess.run(["diskutil", "list", "-plist", "external", "physical"],
                                capture_output=True, timeout=15)
    except subprocess.TimeoutExpired:
        raise ValueError("macOS didn't list the disks in time. Re-seat the card reader and press Refresh.")
    if result.returncode:
        return []
    disks = []
    for disk in plistlib.loads(result.stdout).get("WholeDisks", []):
        # A card reader can be slow to answer (still spinning up): list it anyway, without details.
        try:
            info = subprocess.run(["diskutil", "info", "-plist", disk], capture_output=True, timeout=8)
            meta = plistlib.loads(info.stdout) if info.returncode == 0 else {}
        except (subprocess.TimeoutExpired, plistlib.InvalidFileException):
            meta = {"MediaName": "Card reader (not answering yet)"}
        disks.append({"id": disk, "name": meta.get("MediaName", "Unknown"),
                      "size_gb": round(meta.get("Size", meta.get("TotalSize", 0)) / 1e9, 1)})
    return disks


def _sd_dump_script(project) -> Path:
    for candidate in ([Path(project) / "scripts" / "sd_dump.py"] if project else []) + \
            [HERE / "sd_dump.py", HERE.parent / "scripts" / "sd_dump.py"]:
        if candidate.is_file():
            return candidate
    raise ValueError("sd_dump.py is missing.")


def _run_dump(project, device: str, out: Path, args: list, timeout_s: int = 600):
    """Copy sessions off the card into `out`. Raw card access needs admin rights, and macOS only
    lets Terminal (not other apps, even as admin) read a raw card, so the password is asked there."""
    command = [sys.executable, "-I", str(_sd_dump_script(project)), device, str(out), *args]
    if os.access(device, os.R_OK):
        subprocess.run(command, capture_output=True, timeout=timeout_s)
    else:
        log = out / "terminal.log"
        script = out / "read-card.command"
        script.write_text(
            "#!/bin/sh\n"
            'echo "EasyTVC Studio: copying the SD card to your Mac (read-only)."\n'
            f"diskutil unmountDisk {shlex.quote(device.replace('/dev/r', '/dev/'))} >/dev/null 2>&1\n"
            'sudo -p "Mac login password (nothing shows as you type), then Return: " '
            + " ".join(shlex.quote(c) for c in command) + f" > {shlex.quote(str(log))} 2>&1\n"
            f'echo "rc=$?" >> {shlex.quote(str(log))}\n'
            'echo "Done: back to EasyTVC Studio. You can close this window."\n')
        script.chmod(0o755)
        subprocess.run(["open", "-a", "Terminal", str(script)], check=True, timeout=30)
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if log.exists() and "rc=" in log.read_text(errors="replace"):
                break
            time.sleep(1)
        else:
            raise ValueError("Timed out waiting for your password in the Terminal window.")
    result_file = out / "result.json"
    if not result_file.exists():
        raise ValueError("The card was not read (wrong password, or the Terminal window was closed). Try again.")
    result = json.loads(result_file.read_text())
    if result.get("error") == "nodir":
        raise ValueError("No EasyTVC logs on this card (it has never been used in the board).")
    if not result.get("ok"):
        raise ValueError(f"Could not read the card: {result.get('error')}")
    return result


def _save_session(data: bytes, number: int) -> tuple[Path, bool]:
    """Save into the flights folder. Same data already saved: keep that file. Same name but
    different data (another card, or the session number came round again): add a suffix."""
    folder = flights_dir()
    digest = hashlib.sha256(data).hexdigest()
    for existing in folder.glob(f"session_{number}*.bin"):
        if hashlib.sha256(existing.read_bytes()).hexdigest() == digest:
            return existing, False
    path, n = folder / f"session_{number}.bin", 2
    while path.exists():
        path, n = folder / f"session_{number} ({n}).bin", n + 1
    path.write_bytes(data)
    return path, True


def read_sd(project, disk: str, session: int | None, everything: bool = False) -> dict:
    """Copy the latest session (or `session`, or every session with `everything`) to this Mac and
    open the newest one copied."""
    if not isinstance(disk, str) or not re.fullmatch(r"disk[0-9]+", disk):
        raise ValueError("Choose a disk from the list.")
    if disk not in {d["id"] for d in external_disks()}:
        raise ValueError("That disk is not an external card reader.")
    if session is not None and (isinstance(session, bool) or not isinstance(session, int) or session < 0):
        raise ValueError("Session must be a whole number.")
    subprocess.run(["diskutil", "unmountDisk", f"/dev/{disk}"], capture_output=True, timeout=30)
    out = Path(tempfile.mkdtemp(prefix="easytvc-sd-"))
    try:
        args = ["--all"] if everything else (["--session", str(session)] if session is not None else [])
        result = _run_dump(project, f"/dev/r{disk}", out, args)
        if not result["sessions"]:
            raise ValueError("That session is no longer on the card." if session is not None else "The card has no sessions yet.")
        saved = []
        for item in sorted(result["sessions"], key=lambda x: x["session"]):
            data = Path(item["file"]).read_bytes()
            path, new = _save_session(data, item["session"])
            saved.append({"session": item["session"], "name": path.name, "new": new})
        newest = saved[-1]
        opened = summarize_session(project, (flights_dir() / newest["name"]).read_bytes(),
                                   f"SD card {disk}, saved as {newest['name']}")
        opened["saved"] = saved
        opened["saved_name"] = newest["name"]
        opened["available_sessions"] = list(range(max(0, result["next"] - MAX_SESSIONS), result["next"]))
        return opened
    finally:
        for file in out.iterdir():
            try:
                file.unlink()
            except OSError:
                pass
        try:
            out.rmdir()
        except OSError:
            pass


# ---- sessions saved on this Mac ----

_saved_index: dict = {}


def _describe(project, path: Path) -> dict:
    stat = path.stat()
    key = (path.name, stat.st_size, stat.st_mtime)
    if key in _saved_index:
        return _saved_index[key]
    info = {"name": path.name, "bytes": stat.st_size, "saved": stat.st_mtime, "kind": "unknown"}
    try:
        data = path.read_bytes()
        viewer = _viewer(project)
        if viewer.is_sensor_session(data):
            parsed = viewer.parse_sensor_session(data)
            report = viewer.sensor_report(parsed)
            info.update(kind="sensortest", session=parsed["session"], passed=report["passed"],
                        duration_s=report["duration_s"])
        else:
            session, _, rows = viewer.parse_session(data)
            info.update(kind="flight", session=session, duration_s=rows[-1]["t_s"] - rows[0]["t_s"] if rows else 0,
                        flew=any(r["state"] == "BOOST" for r in rows),
                        max_alt_m=max((r["baro_alt_m"] for r in rows), default=0))
    except (ValueError, OSError, struct.error, IndexError) as error:
        info["error"] = str(error)
    _saved_index[key] = info
    return info


def saved_sessions(project) -> dict:
    folder = flights_dir()
    items = [_describe(project, p) for p in folder.glob("*.bin") if p.is_file()]
    items.sort(key=lambda x: x["saved"], reverse=True)
    return {"folder": str(folder), "sessions": items}


def _saved_path(name) -> Path:
    if not isinstance(name, str) or "/" in name or "\\" in name or name.startswith(".") or not name.endswith(".bin"):
        raise ValueError("Choose a saved session from the list.")
    path = flights_dir() / name
    if not path.is_file():
        raise ValueError("That saved session no longer exists.")
    return path


def open_saved(project, name) -> dict:
    path = _saved_path(name)
    result = summarize_session(project, path.read_bytes(), f"saved on this Mac: {path.name}")
    result["saved_name"] = path.name
    return result


def delete_saved(name) -> dict:
    _saved_path(name).unlink()
    return {"ok": True}


# --------------------------------------------------------------------------------------------
# app state (checklist, vehicle data, last flash)
# --------------------------------------------------------------------------------------------

def state_path() -> Path:
    path = Path.home() / "Library" / "Application Support" / "EasyTVC Studio"
    path.mkdir(parents=True, exist_ok=True)
    return path / "launch_state.json"


def load_state() -> dict:
    try:
        data = json.loads(state_path().read_text())
        return data if isinstance(data, dict) else {}
    except (OSError, ValueError):
        return {}


def save_state(update: dict) -> dict:
    allowed = {"checklist", "vehicle", "flashed", "packing", "sim", "ui"}
    if not isinstance(update, dict) or not set(update) <= allowed:
        raise ValueError("Unsupported state fields.")
    if len(json.dumps(update)) > 65536:
        raise ValueError("State is too large.")
    state = load_state()
    state.update(update)
    temporary = state_path().with_suffix(".tmp")
    temporary.write_text(json.dumps(state, indent=1))
    os.replace(temporary, state_path())
    return state


# --------------------------------------------------------------------------------------------
# 3D flight simulation (Core/Src/flight_sim3d.c, real flight software in the loop)
# --------------------------------------------------------------------------------------------

SIM3D_PARAMS = (
    "KP", "KI", "KD", "INTEGRATOR_LIMIT", "OUTPUT_RATE_LIMIT", "DERIVATIVE_CUTOFF", "MAX_TILT_DEG",
    "SERVO_A_MIN_US", "SERVO_A_CENTER_US", "SERVO_A_MAX_US", "SERVO_A_DIRECTION",
    "SERVO_B_MIN_US", "SERVO_B_CENTER_US", "SERVO_B_MAX_US", "SERVO_B_DIRECTION",
    "SERVO_RATE_HZ", "LAUNCH_ACCEL_G", "LAUNCH_CONFIRM_MS", "TVC_ENABLE_DELAY_MS",
    "chute_enabled", "CHUTE_BACKUP_MS", "ARM_HOLD_MS", "ARM_MAX_TILT_DEG", "DISARM_TILT_DEG",
    "dry_mass_kg", "motor_mass_kg", "prop_mass_kg", "avg_thrust_n", "burn_s", "thrust_profile",
    "diameter_m", "body_cd", "chute_diameter_m", "chute_cd",
    "length_m", "gimbal_arm_m", "cp_ahead_m", "inertia", "cn_alpha", "linkage_ratio", "servo_speed_dps",
    "wind_m_s", "gust_m_s", "wind_dir_deg", "pad_tilt_deg", "misalign_deg", "gyro_bias_dps", "noise",
    "seed", "reverse_servo_a", "tower_m", "servo_deg_per_us", "gimbal_stop_deg", "cg_shift_m",
    "inertia_burnout", "planform_area_m2", "planform_ahead_m",
    "servo_deadband_us", "servo_backlash_deg", "servo_tau_s",
    "thrust_scale", "site_alt_m", "temp_c", "nozzle_arm_m", "motor_gimbal_inertia", "servo_stall_nm",
    "imu_ahead_m", "turbulence_m_s", "gyro_lpf_hz", "accel_lpf_hz", "roll_torque_nmm", "ignition_s",
)
# Estes E12, NAR certification data (thrustcurve.org, C. J. Kobel 2012): 27.2 N*s, 33 N peak
E12_CURVE = [[0.052, 5.045], [0.096, 9.910], [0.196, 24.144], [0.251, 31.351], [0.287, 32.973], [0.300, 29.910],
             [0.344, 17.117], [0.370, 14.414], [0.400, 12.973], [0.500, 11.712], [0.600, 11.171], [0.700, 10.631],
             [0.800, 10.09], [0.900, 9.73], [1.000, 9.55], [1.101, 9.91], [1.200, 9.55], [1.300, 9.73], [1.400, 9.73],
             [1.500, 9.73], [1.600, 9.73], [1.700, 9.55], [1.800, 9.73], [1.900, 9.73], [2.000, 9.55], [2.100, 9.55],
             [2.200, 9.73], [2.300, 9.19], [2.375, 9.37], [2.400, 5.95], [2.440, 0.0]]


def parse_eng(text: str) -> dict:
    """RASP .eng: comment lines ';', header 'name dia_mm len_mm delays prop_kg total_kg maker', then 't F' pairs."""
    lines = [l.split(";")[0].strip() for l in text.splitlines()]
    lines = [l for l in lines if l]
    if len(lines) < 3:
        raise ValueError("Not a RASP .eng thrust curve.")
    head = lines[0].split()
    if len(head) < 7:
        raise ValueError("The .eng header line is incomplete.")
    try:
        curve = [[float(a), float(b)] for a, b in (l.split()[:2] for l in lines[1:])]
        info = {"name": f"{head[6]} {head[0]}", "diameter_mm": float(head[1]), "length_mm": float(head[2]),
                "prop_mass_g": float(head[4]) * 1000, "total_mass_g": float(head[5]) * 1000}
    except (ValueError, IndexError):
        raise ValueError("The .eng file has numbers I can't read.")
    if len(curve) > 120 or any(curve[i][0] <= (curve[i - 1][0] if i else 0) for i in range(len(curve))) or any(f < 0 for _, f in curve):
        raise ValueError("The thrust curve must have up to 120 points with increasing times and positive thrust.")
    impulse = sum(0.5 * (curve[i][1] + (curve[i - 1][1] if i else 0)) * (curve[i][0] - (curve[i - 1][0] if i else 0)) for i in range(len(curve)))
    burn = curve[-1][0]
    info.update(curve=curve, burn_s=round(burn, 3), avg_thrust_n=round(impulse / burn, 3), impulse_ns=round(impulse, 2))
    return info
SIM3D_COLUMNS = ("t", "x", "y", "z", "qw", "qx", "qy", "qz", "speed", "vz", "tilt_x", "tilt_y",
                 "gimbal_a", "gimbal_b", "state", "tvc", "chute", "thrust", "est_tilt_x", "est_tilt_y", "armed",
                 "pulse_a", "pulse_b", "horn_a", "horn_b", "alpha",
                 "tq_tvc_x", "tq_tvc_y", "tq_aero_x", "tq_aero_y", "servo_fault", "wind", "roll_rate")
SIM3D_SUMMARY = ("apogee_m", "apogee_t", "max_speed", "max_boost_tilt", "drift_m", "landing_speed",
                 "chute_t", "chute_speed", "armed_t", "launch_t", "tvc_fault", "saturation_pct",
                 "flight_time", "final_state", "under_chute", "est_error_deg", "liftoff", "done", "time", "max_roll_dps")
# (low, high, default) for the simulation-only inputs
SIM_INPUTS = {
    "cn_alpha": (0, 10, 2.0), "wind_m_s": (0, 25, 2), "gust_m_s": (0, 35, 4),
    "wind_dir_deg": (0, 360, 30), "pad_tilt_deg": (0, 30, 1), "misalign_deg": (0, 5, 0.5),
    "gyro_bias_dps": (0, 10, 0.5), "noise": (0, 3, 1), "seed": (0, 100000, 1),
    "reverse_servo_a": (0, 1, 0), "tower_m": (0, 3, 0.3), "thrust_profile": (0, 2, 2),
    "site_alt_m": (-400, 4000, 0), "temp_c": (-30, 50, 15), "turbulence_m_s": (0, 5, 0.5),
    "roll_torque_nmm": (-20, 20, 0.1),
}


def sim_defaults():
    return {k: v[2] for k, v in SIM_INPUTS.items()}


def sim_params(config: dict, vehicle: dict, sim: dict) -> list:
    clean = validate_config(config)
    if not isinstance(sim, dict) or set(sim) != set(SIM_INPUTS):
        raise ValueError("Simulation inputs are incomplete.")
    for key, (low, high, _) in SIM_INPUTS.items():
        value = sim[key]
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not low <= value <= high:
            raise ValueError(f"{key} must be between {low:g} and {high:g}.")
    if sim["gust_m_s"] < sim["wind_m_s"]:
        raise ValueError("Gusts must be at least the steady wind.")
    model = rocket_model(vehicle)
    v = model["vehicle"]
    motor = v["motor"]
    values = {
        "dry_mass_kg": (model["mass_wet_g"] - float(motor["total_mass_g"])) / 1000,
        "motor_mass_kg": float(motor["total_mass_g"]) / 1000, "prop_mass_kg": float(motor["prop_mass_g"]) / 1000,
        "avg_thrust_n": _num(motor["avg_thrust_n"], 0.1, 10000, "Average thrust"),
        "burn_s": _num(motor["burn_s"], 0.1, 20, "Burn time"),
        "diameter_m": float(v["diameter_mm"]) / 1000, "body_cd": _num(v["cd"], 0.05, 3, "Body drag coefficient"),
        "chute_diameter_m": _num(v["chute_diameter_cm"], 0, 1000, "Parachute diameter") / 100,
        "chute_cd": _num(v["chute_cd"], 0.1, 3, "Parachute drag coefficient"),
        "length_m": model["length_mm"] / 1000, "gimbal_arm_m": model["gimbal_arm_mm"] / 1000,
        "cp_ahead_m": model["cp_ahead_mm"] / 1000, "inertia": model["inertia_wet"],
        "inertia_burnout": model["inertia_dry"], "cg_shift_m": (model["cg_wet_mm"] - model["cg_dry_mm"]) / 1000,
        "planform_area_m2": model["planform_m2"], "planform_ahead_m": (model["cg_wet_mm"] - model["planform_center_mm"]) / 1000,
        "linkage_ratio": model["linkage_ratio"], "servo_speed_dps": model["servo_speed_dps"],
        "servo_deg_per_us": model["servo_deg_per_us"], "gimbal_stop_deg": model["stop_deg"],
        "servo_deadband_us": model["servo_deadband_us"], "servo_backlash_deg": model["servo_backlash_deg"],
        "servo_tau_s": model["servo_tau_s"],
        "thrust_scale": 1.0, "nozzle_arm_m": (model["nozzle_mm"] - model["cg_wet_mm"]) / 1000,
        "motor_gimbal_inertia": model["motor_gimbal_inertia"], "servo_stall_nm": model["servo_stall_nm"],
        "imu_ahead_m": (model["cg_wet_mm"] - model["imu_mm"]) / 1000, "gyro_lpf_hz": 47.0, "accel_lpf_hz": 145.0,
    }
    if model["gimbal_arm_mm"] <= 5:
        raise ValueError("The gimbal pivot must be behind the CG: check part positions.")
    values.update(clean)
    values.update(sim)
    values["chute_enabled"] = 1 if clean["CHUTE_CHANNEL"] else 0
    values["ignition_s"] = clean["ARM_HOLD_MS"] / 1000 + 2.0
    return [float(values[name]) for name in SIM3D_PARAMS]


def sim_verdict(s: dict, config: dict) -> list:
    out = []
    add = lambda level, text: out.append({"level": level, "text": text})
    if s["armed_t"] < 0:
        add("bad", "Never armed on the pad: check pad tilt against the arming tilt limit. TVC did not run.")
    if not s["liftoff"]:
        add("bad", "Did not leave the stand: too heavy for the motor.")
        return out
    if s["launch_t"] < 0:
        add("bad", "Launch was not detected, so TVC never switched on. Lower the launch threshold or make the rocket lighter.")
    if s["tvc_fault"]:
        add("bad", f"TVC gave up: tilt passed the {config['MAX_TILT_DEG']:g}° limit during boost. Raise the gains, gimbal travel or speed, or reduce instability (CP ahead of CG).")
    elif s["max_boost_tilt"] > 10:
        add("warn", f"Held, but tilted up to {s['max_boost_tilt']:.1f}° during boost.")
    elif s["launch_t"] >= 0:
        add("good", f"Stable boost: worst tilt {s['max_boost_tilt']:.1f}°.")
    if s.get("max_roll_dps", 0) > 720:
        add("warn", f"Rolling up to {s['max_roll_dps']:.0f}°/s during boost. The TVC compensates, but find the asymmetry (a lug, a crooked motor mount) before it gets worse.")
    if s["saturation_pct"] > 10:
        add("warn", f"The gimbal was at full travel {s['saturation_pct']:.0f}% of the burn: little margin left.")
    if config["CHUTE_CHANNEL"]:
        if s["chute_t"] < 0:
            add("bad", "Parachute never opened.")
        elif s["chute_speed"] > 20:
            add("warn", f"Parachute opened at {s['chute_speed']:.0f} m/s (apogee was {s['apogee_t']:.1f} s after ignition). Set the backup closer to apogee.")
        else:
            add("good", f"Parachute opened at {s['chute_t']:.1f} s, {s['chute_speed']:.0f} m/s.")
    if s["landing_speed"] > 0:
        level = "good" if s["landing_speed"] < 6.5 and s["under_chute"] else "warn" if s["landing_speed"] < 9 else "bad"
        add(level, f"Lands at {s['landing_speed']:.1f} m/s, {s['drift_m']:.0f} m from the pad.")
    if s["est_error_deg"] > 3:
        add("warn", f"The board's attitude estimate drifted {s['est_error_deg']:.1f}° from the truth during boost.")
    return out


def motor_curve(vehicle):
    curve = upgrade_vehicle(vehicle)["motor"].get("curve")
    return curve if isinstance(curve, list) and len(curve) >= 2 else E12_CURVE


def run_sim3d(native, config, vehicle, sim):
    params = sim_params(config, vehicle, sim)
    rows, summary = native.sim3d(params, curve=motor_curve(vehicle))
    s = dict(zip(SIM3D_SUMMARY, summary))
    model = rocket_model(vehicle)
    model.pop("vehicle")
    return {"columns": SIM3D_COLUMNS, "rows": rows, "summary": s, "verdict": sim_verdict(s, validate_config(config)),
            "ignition_s": params[SIM3D_PARAMS.index("ignition_s")], "model": model}


def run_sim3d_batch(native, config, vehicle, sim, count=40):
    """Monte Carlo: random wind direction/strength, turbulence, misalignment, pad tilt, sensor noise seed,
    plus build and motor spread: thrust +-5 %, CG +-5 mm, inertia +-10 %, and a random roll torque
    (either direction, up to 5x the set value) because nothing on a finless rocket stops it rolling."""
    import random
    count = max(5, min(200, int(count)))
    rng = random.Random(int(sim.get("seed", 1)))
    runs, ok = [], 0
    for i in range(count):
        trial = dict(sim)
        trial["seed"] = rng.randint(1, 99999)
        trial["wind_m_s"] = round(rng.uniform(0, max(sim["wind_m_s"], 0.1) * 1.5), 2)
        trial["gust_m_s"] = round(trial["wind_m_s"] * rng.uniform(1.2, 2.0), 2)
        trial["wind_dir_deg"] = round(rng.uniform(0, 360), 1)
        trial["misalign_deg"] = round(rng.uniform(0, max(sim["misalign_deg"], 0.2) * 2), 2)
        trial["pad_tilt_deg"] = round(rng.uniform(0, max(sim["pad_tilt_deg"], 0.5) * 1.5), 2)
        for key in ("wind_m_s", "gust_m_s", "misalign_deg", "pad_tilt_deg"):
            low, high, _ = SIM_INPUTS[key]
            trial[key] = min(high, max(low, trial[key]))
        params = sim_params(config, vehicle, trial)
        at = SIM3D_PARAMS.index
        params[at("thrust_scale")] = rng.uniform(0.95, 1.05)
        dcg = rng.uniform(-0.005, 0.005)        # CG measured 5 mm off, + = further aft
        params[at("gimbal_arm_m")] -= dcg
        params[at("nozzle_arm_m")] -= dcg
        params[at("cp_ahead_m")] += dcg
        params[at("planform_ahead_m")] += dcg
        params[at("imu_ahead_m")] += dcg
        k = rng.uniform(0.9, 1.1)
        params[at("inertia")] *= k
        params[at("inertia_burnout")] *= k
        roll = max(abs(sim["roll_torque_nmm"]), 0.1) * 5
        params[at("roll_torque_nmm")] = rng.uniform(-roll, roll)
        _, summary = native.sim3d(params, rows=False, curve=motor_curve(vehicle))
        s = dict(zip(SIM3D_SUMMARY, summary))
        success = bool(s["liftoff"] and s["launch_t"] >= 0 and not s["tvc_fault"] and
                       (s["under_chute"] or not config.get("CHUTE_CHANNEL")) and 0 < s["landing_speed"] < 9)
        ok += success
        runs.append({"success": success, "wind": trial["wind_m_s"], "gust": trial["gust_m_s"],
                     "misalign": trial["misalign_deg"], "pad_tilt": trial["pad_tilt_deg"],
                     "tilt": s["max_boost_tilt"], "apogee": s["apogee_m"], "drift": s["drift_m"],
                     "landing": s["landing_speed"], "fault": bool(s["tvc_fault"])})
    return {"count": count, "success": ok, "runs": runs}


# --------------------------------------------------------------------------------------------
# Rocket model: geometry in millimetres -> mass properties and aerodynamics
# --------------------------------------------------------------------------------------------
#
# Positions are measured along the body from the NOSE TIP (mm, aft is positive), like OpenRocket.
# Every part is a mass spread evenly over its length. The motor's propellant burns in place, so
# the CG moves forward during the burn. CP uses Barrowman's method for a finless rocket: only the
# nose cone produces normal force at small angles (CN_alpha = 2/rad at 0.466 L for an ogive,
# 2/3 L for a cone, 1/2 L for a parabola, 1/3 L for an ellipse).

NOSE_SHAPES = {"ogive": (0.466, 2 / 3), "conical": (2 / 3, 1 / 2), "parabolic": (0.5, 2 / 3), "elliptical": (1 / 3, 0.785)}
# shape -> (CP fraction of nose length, planform area / (D * L) for the nose)

GIMBAL_DEFAULTS = {"horn_mm": 10.5, "lever_mm": 30.0, "servo_deg_per_us": 0.09, "servo_s_per_60": 0.10, "stop_deg": 8.0,
                   # how a real hobby servo behaves (MG90D-class): ignored pulse change, gear + linkage slop, loop speed
                   "deadband_us": 4.0, "backlash_deg": 0.8, "tau_s": 0.010,
                   "stall_nm": 0.2}   # MG90D: ~2.0-2.2 kg*cm at the horn
BODY_DEFAULTS = {"length_mm": 450.0, "nose_length_mm": 150.0, "nose_shape": "ogive",
                 "motor_overhang_mm": 10.0, "pivot_from_nozzle_mm": 45.0}
DEFAULT_PARTS = [  # name, grams, centre from nose tip (mm), length (mm)
    ["Nose cone", 30, 90, 150], ["Nose ballast", 80, 40, 30], ["Body tube", 55, 375, 450], ["Parachute & shock cord", 25, 220, 90],
    ["Parachute servo", 9, 270, 30], ["EasyTVC board", 20, 330, 65], ["2S LiPo 300 mAh", 22, 385, 55],
    ["Wiring & hardware", 15, 420, 100], ["TVC servos ×2", 26, 505, 30], ["Gimbal & motor mount", 40, 545, 70],
]


def upgrade_vehicle(vehicle: dict | None) -> dict:
    """Older saved vehicles had parts as [name, grams]; add positions and the new sections."""
    v = json.loads(json.dumps(vehicle)) if isinstance(vehicle, dict) else {}
    v.setdefault("motor", {"name": "Estes E12-0 (approx.)", "avg_thrust_n": 12, "burn_s": 2.5,
                           "total_mass_g": 58, "prop_mass_g": 36})
    v["motor"].setdefault("diameter_mm", 24.0)
    v["motor"].setdefault("length_mm", 95.0)
    if "curve" not in v["motor"] and "E12" in str(v["motor"].get("name", "")):
        v["motor"]["curve"] = E12_CURVE
        v["motor"].update(name="Estes E12 (NAR data)", avg_thrust_n=11.15, burn_s=2.44, total_mass_g=59.9, prop_mass_g=35.9)
    v.setdefault("diameter_mm", 66.0)
    v.setdefault("cd", 0.6)
    v.setdefault("chute_diameter_cm", 55)
    v.setdefault("chute_cd", 0.8)
    v["body"] = {**BODY_DEFAULTS, **(v.get("body") or {})}
    v["gimbal"] = {**GIMBAL_DEFAULTS, **(v.get("gimbal") or {})}
    parts = v.get("parts") or []
    if parts and all(len(p) == 2 for p in parts):
        # old format: keep the weights, place them like the defaults by name or spread along the body
        by_name = {p[0]: p for p in DEFAULT_PARTS}
        total = v["body"]["nose_length_mm"] + v["body"]["length_mm"]
        parts = [[p[0], p[1], *(by_name[p[0]][2:] if p[0] in by_name else (round(total * (i + 1) / (len(parts) + 1)), 40))]
                 for i, p in enumerate(parts)]
    v["parts"] = parts if parts else [list(p) for p in DEFAULT_PARTS]
    return v


def _num(value, low, high, label):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or value != value or not low <= value <= high:
        raise ValueError(f"{label} must be between {low:g} and {high:g}.")
    return float(value)


def rocket_model(vehicle: dict) -> dict:
    v = upgrade_vehicle(vehicle)
    body, motor, gim = v["body"], v["motor"], v["gimbal"]
    D = _num(v["diameter_mm"], 10, 500, "Body diameter")
    Lb = _num(body["length_mm"], 50, 5000, "Body tube length")
    Ln = _num(body["nose_length_mm"], 10, 1500, "Nose cone length")
    if body["nose_shape"] not in NOSE_SHAPES:
        raise ValueError("Unknown nose cone shape.")
    overhang = _num(body["motor_overhang_mm"], -200, 200, "Motor overhang")
    md = _num(motor["diameter_mm"], 6, D, "Motor diameter")
    ml = _num(motor["length_mm"], 10, 2000, "Motor length")
    pivot_from_nozzle = _num(body["pivot_from_nozzle_mm"], 0, ml, "Gimbal pivot (from nozzle)")
    m_motor = _num(motor["total_mass_g"], 1, 50000, "Motor mass")
    m_prop = _num(motor["prop_mass_g"], 0, m_motor, "Propellant mass")
    length = Ln + Lb + max(0.0, overhang)
    nozzle = Ln + Lb + overhang
    motor_center = nozzle - ml / 2
    pivot = nozzle - pivot_from_nozzle

    items = []   # (grams, centre mm, length mm)
    for i, part in enumerate(v["parts"]):
        if not isinstance(part, list) or len(part) != 4 or not isinstance(part[0], str):
            raise ValueError(f"Part {i + 1} needs a name, grams, position and length.")
        g = _num(part[1], 0, 100000, f"{part[0]}: grams")
        x = _num(part[2], 0, length, f"{part[0]}: position (mm from nose tip)")
        l = _num(part[3], 0, length, f"{part[0]}: length")
        items.append((g, x, l))

    def props(motor_grams):
        allm = items + [(motor_grams, motor_center, ml)]
        mass = sum(g for g, _, _ in allm)
        cg = sum(g * x for g, x, _ in allm) / mass
        # thin rods about the CG + a solid cylinder's radial term, kg*m^2
        inertia = sum(g * ((x - cg) ** 2 + l * l / 12 + (D / 2) ** 2 / 4) for g, x, l in allm) * 1e-9
        return mass, cg, inertia

    mass_wet, cg_wet, i_wet = props(m_motor)
    mass_dry, cg_dry, i_dry = props(m_motor - m_prop)
    cp_frac, plan_frac = NOSE_SHAPES[body["nose_shape"]]
    cp = cp_frac * Ln
    nose_plan = plan_frac * D * Ln
    body_plan = D * Lb
    planform = nose_plan + body_plan
    plan_x = (nose_plan * Ln * (0.6 if body["nose_shape"] != "conical" else 2 / 3) + body_plan * (Ln + Lb / 2)) / planform

    horn = _num(gim["horn_mm"], 1, 100, "Servo horn radius")
    lever = _num(gim["lever_mm"], 1, 200, "Gimbal lever radius")
    if horn > lever:
        raise ValueError("The servo horn must be shorter than the gimbal lever.")
    dpu = _num(gim["servo_deg_per_us"], 0.02, 0.4, "Servo degrees per µs")
    s60 = _num(gim["servo_s_per_60"], 0.02, 1.0, "Servo speed (s per 60°)")
    stop = _num(gim["stop_deg"], 0.5, 30, "Gimbal mechanical stop")
    stall = _num(gim["stall_nm"], 0.01, 5, "Servo stall torque")
    deadband = _num(gim["deadband_us"], 0, 50, "Servo deadband")
    backlash = _num(gim["backlash_deg"], 0, 10, "Servo backlash")
    tau = _num(gim["tau_s"], 0.001, 0.2, "Servo response time")
    return {
        "length_mm": length, "nose_tip_mm": 0.0, "nozzle_mm": nozzle, "motor_top_mm": nozzle - ml,
        "motor_center_mm": motor_center, "pivot_mm": pivot,
        "mass_wet_g": mass_wet, "mass_dry_g": mass_dry, "cg_wet_mm": cg_wet, "cg_dry_mm": cg_dry,
        "cp_mm": cp, "planform_center_mm": plan_x, "planform_m2": planform * 1e-6,
        "inertia_wet": i_wet, "inertia_dry": i_dry,
        "gimbal_arm_mm": pivot - cg_wet, "cp_ahead_mm": cg_wet - cp,
        "stability_cal": (cp - cg_wet) / D, "stability_cal_dry": (cp - cg_dry) / D,
        "linkage_ratio": horn / lever, "servo_deg_per_us": dpu, "servo_speed_dps": 60.0 / s60, "stop_deg": stop,
        "servo_deadband_us": deadband, "servo_backlash_deg": backlash, "servo_tau_s": tau, "servo_stall_nm": stall,
        # gimballed motor about its pivot (thin rod + offset), wet; the airframe feels its reaction
        "motor_gimbal_inertia": m_motor * 1e-3 * ((ml / 1000) ** 2 / 12 + ((motor_center - pivot) / 1000) ** 2),
        "imu_mm": next((x for (g, x, l), part in zip(items, v["parts"]) if "board" in part[0].lower()), cg_wet),
        "vehicle": v,
    }


def gimbal_angle_deg(pulse_offset_us, model):
    """Pulse offset from centre -> gimbal angle, same linkage maths as the C simulator."""
    import math
    horn = math.radians(pulse_offset_us * model["servo_deg_per_us"])
    g = math.degrees(math.asin(max(-1.0, min(1.0, model["linkage_ratio"] * math.sin(horn)))))
    return max(-model["stop_deg"], min(model["stop_deg"], g))


# ---- 3D replay of a real session ----
# Turns a flight log (or a sensor test) into the same rows the simulator produces, so the 3D view,
# gimbal view, HUD and playback controls show what really happened.

def _qmul(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (aw * bw - ax * bx - ay * by - az * bz, aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx, aw * bz + ax * by - ay * bx + az * bw)


def _qnorm(q):
    n = math.sqrt(sum(c * c for c in q)) or 1.0
    return tuple(c / n for c in q)


def _qaxis(axis, angle):
    n = math.sqrt(sum(c * c for c in axis))
    if n < 1e-12 or abs(angle) < 1e-12:
        return (1.0, 0.0, 0.0, 0.0)
    s_ = math.sin(angle / 2) / n
    return (math.cos(angle / 2), axis[0] * s_, axis[1] * s_, axis[2] * s_)


def _qrot(q, v):
    """body -> world"""
    w, x, y, z = q
    tx, ty, tz = 2 * (y * v[2] - z * v[1]), 2 * (z * v[0] - x * v[2]), 2 * (x * v[1] - y * v[0])
    return (v[0] + w * tx + (y * tz - z * ty), v[1] + w * ty + (z * tx - x * tz), v[2] + w * tz + (x * ty - y * tx))


def _up_in_body(q):
    """World up seen from the body (third row of the body->world matrix)."""
    w, x, y, z = q
    return (2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y))


def _tilt_deg(q):
    """Same as EasyTVC_ControllerTilt: rotations about body X and Y away from vertical."""
    w, x, y, z = q
    vertical = 1 - 2 * (x * x + y * y)
    return (math.degrees(math.atan2(2 * (w * x + y * z), vertical)), math.degrees(math.atan2(2 * (w * y - x * z), vertical)))


def _toward(q, up_target, gain=1.0):
    """Rotate the body (about a body axis) so world-up-in-body moves `gain` of the way to up_target."""
    cur = _up_in_body(q)
    axis = (up_target[1] * cur[2] - up_target[2] * cur[1], up_target[2] * cur[0] - up_target[0] * cur[2],
            up_target[0] * cur[1] - up_target[1] * cur[0])
    dot = max(-1.0, min(1.0, sum(a * b for a, b in zip(up_target, cur))))
    return _qnorm(_qmul(q, _qaxis(axis, gain * math.acos(dot))))


def _up_from_tilt(tx_deg, ty_deg):
    tx, ty = (math.radians(max(-89.0, min(89.0, v))) for v in (tx_deg, ty_deg))
    v = (-math.tan(ty), math.tan(tx), 1.0)
    n = math.sqrt(sum(c * c for c in v))
    return tuple(c / n for c in v)


def _imu_to_body(axis: str, v):
    """Sensor frame -> body frame (+Z = nose), the same table as main.c imu_to_body."""
    x, y, z = v
    return {"POS_X": (y, z, x), "NEG_X": (y, -z, -x), "POS_Y": (z, x, y), "NEG_Y": (z, -x, -y),
            "POS_Z": (x, y, z), "NEG_Z": (x, -y, -z)}[axis]


def _replay_rows(samples, model, config):
    """samples: dicts with t, q, z, vz, vx, vy, x, y, state, tvc, chute, armed, pulse_a, pulse_b, roll_rate, thrust."""
    rows = []
    for s_ in samples:
        tilt = _tilt_deg(s_["q"])
        horn, gim = [], []
        for key, letter in (("pulse_a", "A"), ("pulse_b", "B")):
            offset = (s_[key] - config[f"SERVO_{letter}_CENTER_US"]) * config[f"SERVO_{letter}_DIRECTION"]
            horn.append(offset * model["servo_deg_per_us"])
            gim.append(gimbal_angle_deg(offset, model))
        values = {"t": s_["t"], "x": s_["x"], "y": s_["y"], "z": s_["z"], "qw": s_["q"][0], "qx": s_["q"][1],
                  "qy": s_["q"][2], "qz": s_["q"][3], "speed": math.sqrt(s_["vx"] ** 2 + s_["vy"] ** 2 + s_["vz"] ** 2),
                  "vz": s_["vz"], "tilt_x": tilt[0], "tilt_y": tilt[1], "gimbal_a": gim[0], "gimbal_b": gim[1],
                  "state": STATES.index(s_["state"]) if s_["state"] in STATES else 0, "tvc": s_["tvc"],
                  "chute": s_["chute"], "thrust": s_["thrust"], "est_tilt_x": tilt[0], "est_tilt_y": tilt[1],
                  "armed": s_["armed"], "pulse_a": s_["pulse_a"], "pulse_b": s_["pulse_b"], "horn_a": horn[0],
                  "horn_b": horn[1], "roll_rate": s_["roll_rate"]}
        rows.append([float(values.get(c, 0.0)) for c in SIM3D_COLUMNS])
    return rows


def _replay_flight(rows, model, config):
    g = 9.80665
    mass = model["mass_wet_g"] / 1000
    q = None
    x = y = vx = vy = 0.0
    out, prev_t = [], None
    flying = ("BOOST", "COAST")
    for r in rows:
        target = _up_from_tilt(r["tilt_x_deg"], r["tilt_y_deg"])
        if q is None:
            q = _toward((1.0, 0.0, 0.0, 0.0), target)
        else:
            dt = max(0.0, r["t_s"] - prev_t)
            w = tuple(math.radians(r[k]) for k in ("gyro_x_dps", "gyro_y_dps", "gyro_z_dps"))
            rate = math.sqrt(sum(c * c for c in w))
            q = _qnorm(_qmul(q, _qaxis(w, rate * dt)))   # roll and heading from the gyro
            q = _toward(q, target)                        # tilt exactly as the board estimated it
            a_world = _qrot(q, tuple(r[k] * g for k in ("accel_x_g", "accel_y_g", "accel_z_g")))
            if r["state"] in flying:
                vx += a_world[0] * dt
                vy += a_world[1] * dt
            elif r["state"] in ("APOGEE", "DESCENT"):
                decay = math.exp(-dt / 3.0)                # under the parachute: drift with the air
                vx *= decay
                vy *= decay
            else:
                vx = vy = 0.0
            x += vx * dt
            y += vy * dt
        prev_t = r["t_s"]
        on_ground = r["state"] in ("IDLE", "ARMED", "LANDED", "FAULT")
        out.append({"t": r["t_s"], "q": q, "x": x, "y": y, "z": max(0.0, r["baro_alt_m"]) if not on_ground or r["state"] == "LANDED" else 0.0,
                    "vz": r["vspeed_m_s"], "vx": vx, "vy": vy, "state": r["state"], "tvc": r["tvc_active"],
                    "chute": r["chute_open"], "armed": 1 if r["state"] not in ("IDLE", "FAULT") else 0,
                    "pulse_a": r["servo_a_us"], "pulse_b": r["servo_b_us"], "roll_rate": r["gyro_z_dps"],
                    "thrust": max(0.0, r["accel_z_g"]) * g * mass if r["state"] == "BOOST" else 0.0})
    return out


def _replay_sensor_test(parsed, model, config):
    """Orientation from the raw IMU (gyro integrated, pulled toward the accelerometer's gravity),
    mapped through the configured nose axis: stand the rocket nose-up and the 3D rocket must point up."""
    imu = parsed["imu"]
    if not imu:
        raise ValueError("This sensor test has no IMU data to replay.")
    axis = config["NOSE_AXIS"]
    still = imu[:min(len(imu), 200)]
    bias = [sorted(r[k] for r in still)[len(still) // 2] for k in ("gyro_x_dps", "gyro_y_dps", "gyro_z_dps")]
    q, prev_t, out = None, None, []
    step = 8 if len(imu) <= 8 * 12000 else 16
    centre_a, centre_b = config["SERVO_A_CENTER_US"], config["SERVO_B_CENTER_US"]
    for i, r in enumerate(imu):
        acc = _imu_to_body(axis, (r["accel_x_g"], r["accel_y_g"], r["accel_z_g"]))
        gyr = _imu_to_body(axis, tuple(r[k] - b for k, b in zip(("gyro_x_dps", "gyro_y_dps", "gyro_z_dps"), bias)))
        n = math.sqrt(sum(c * c for c in acc))
        up = tuple(c / n for c in acc) if n > 0.1 else None
        if q is None:
            if up is None:
                continue
            q = _toward((1.0, 0.0, 0.0, 0.0), up)
        else:
            dt = max(0.0, r["t_s"] - prev_t)
            w = tuple(math.radians(c) for c in gyr)
            q = _qnorm(_qmul(q, _qaxis(w, math.sqrt(sum(c * c for c in w)) * dt)))
            if up is not None and 0.8 < n < 1.2:          # only trust gravity when not being shaken
                q = _toward(q, up, 0.02)
        prev_t = r["t_s"]
        if i % step == 0:                                 # 400 Hz in, 50 Hz rows out (25 Hz for long runs)
            out.append({"t": r["t_s"], "q": q, "x": 0.0, "y": 0.0, "z": 0.0, "vz": 0.0, "vx": 0.0, "vy": 0.0,
                        "state": "IDLE", "tvc": 0, "chute": 0, "armed": 0, "pulse_a": centre_a, "pulse_b": centre_b,
                        "roll_rate": gyr[2], "thrust": 0.0})
    return out


def replay_session(project, data_id, config, vehicle) -> dict:
    if not isinstance(data_id, str) or data_id not in _raw_sessions:
        raise ValueError("That session is no longer loaded. Open it again in Flight logs.")
    data = _raw_sessions[data_id]
    clean = validate_config(config)
    model = rocket_model(vehicle)
    model.pop("vehicle", None)
    viewer = _viewer(project)
    if viewer.is_sensor_session(data):
        parsed = viewer.parse_sensor_session(data)
        samples = _replay_sensor_test(parsed, model, clean)
        kind, session = "sensortest", parsed["session"]
    else:
        session, _, log_rows = viewer.parse_session(data)
        if len(log_rows) < 2:
            raise ValueError("This log is too short to replay.")
        samples = _replay_flight(log_rows, model, clean)
        kind = "flight"
    rows = _replay_rows(samples, model, clean)
    col = {c: i for i, c in enumerate(SIM3D_COLUMNS)}
    boost = next((r for r in rows if STATES[int(r[col["state"]])] == "BOOST"), None)
    ignition = boost[0] if boost else rows[0][0] + 1.5
    flown = [r for r in rows if r[0] >= ignition]
    apogee = max(rows, key=lambda r: r[col["z"]])
    chute = next((r for r in rows if r[col["chute"]]), None)
    armed = next((r for r in rows if r[col["armed"]]), None)
    descent = [r for r in rows if STATES[int(r[col["state"]])] == "DESCENT"]
    boost_rows = [r for r in rows if STATES[int(r[col["state"]])] == "BOOST"]
    summary = {k: 0.0 for k in SIM3D_SUMMARY}
    summary.update({
        "apogee_m": apogee[col["z"]], "apogee_t": max(0.0, apogee[0] - ignition) if boost else 0.0,
        "max_speed": max((r[col["speed"]] for r in flown), default=0.0),
        "max_boost_tilt": max((math.hypot(r[col["tilt_x"]], r[col["tilt_y"]]) for r in boost_rows), default=0.0),
        "chute_t": chute[0] - ignition if chute and boost else -1.0, "chute_speed": chute[col["speed"]] if chute else 0.0,
        "landing_speed": abs(descent[-1][col["vz"]]) if descent else 0.0,
        "drift_m": math.hypot(rows[-1][col["x"]], rows[-1][col["y"]]),
        "armed_t": armed[0] - rows[0][0] if armed else -1.0, "liftoff": 1.0 if boost else 0.0,
        "max_roll_dps": max((abs(r[col["roll_rate"]]) for r in (boost_rows or rows)), default=0.0),
        "done": 1.0, "time": rows[-1][0] - rows[0][0], "launch_t": 0.0 if boost else -1.0,
    })
    if kind == "sensortest":
        verdict = [{"level": "info", "text": f"Replay of sensor test session {session}: orientation worked out from the raw IMU, "
                    f"with the nose axis set to {clean['NOSE_AXIS']}. Stand the rocket nose-up while it records: if this 3D rocket "
                    "doesn't point straight up then, the nose axis setting is wrong."}]
    else:
        verdict = [{"level": "info", "text": f"Replay of real flight data, session {session}: tilt is the board's own estimate, roll from the gyro, "
                    "height from the barometer, gimbal from the logged servo pulses. Sideways drift is integrated from the "
                    "accelerometer and only approximate."}]
        if not boost:
            verdict.append({"level": "warn", "text": "This log never left the pad (no BOOST), so there is no flight to show: it replays the pad time."})
    return {"columns": SIM3D_COLUMNS, "rows": rows, "summary": summary, "verdict": verdict, "ignition_s": ignition,
            "model": model, "replay": {"kind": kind, "session": session}}


# ---- live flight: the same engine, advanced in small slices as wall time passes ----
INJECTIONS = {"kick": 1, "gust": 2, "jam": 3, "free": 4, "manual": 5, "auto": 6, "spin": 7}


def live_start(native, config, vehicle, sim):
    params = sim_params(config, vehicle, sim)
    native.sim3d_begin(params, curve=motor_curve(vehicle))
    model = rocket_model(vehicle)
    model.pop("vehicle")
    return {"columns": SIM3D_COLUMNS, "ignition_s": params[SIM3D_PARAMS.index("ignition_s")], "model": model}


def live_step(native, config, seconds, manual=None):
    if isinstance(seconds, bool) or not isinstance(seconds, (int, float)) or not 0 <= seconds <= 1:
        raise ValueError("Step must be 0..1 s.")
    if manual is not None:
        # joystick: [a, b] in -1..1 drives the servos directly; None leaves the mode unchanged; "auto" hands back
        if manual == "auto":
            native.sim3d_inject(INJECTIONS["auto"], 0.0, 0.0)
        elif isinstance(manual, list) and len(manual) == 2 and all(isinstance(x, (int, float)) and not isinstance(x, bool) for x in manual):
            native.sim3d_inject(INJECTIONS["manual"], float(max(-1, min(1, manual[0]))), float(max(-1, min(1, manual[1]))))
        else:
            raise ValueError("Joystick input must be [a, b] in -1..1.")
    rows, summary = native.sim3d_advance(float(seconds))
    s = dict(zip(SIM3D_SUMMARY, summary))
    out = {"rows": rows, "summary": s, "done": bool(s["done"])}
    if out["done"]:
        out["verdict"] = sim_verdict(s, validate_config(config))
    return out


def live_inject(native, kind, a=0.0, b=0.0):
    if kind not in INJECTIONS:
        raise ValueError("Unknown disturbance.")
    for v in (a, b):
        if isinstance(v, bool) or not isinstance(v, (int, float)):
            raise ValueError("Disturbance values must be numbers.")
    code = native.sim3d_inject(INJECTIONS[kind], float(a), float(b))
    if code == -2:
        raise ValueError("The rocket is still on the stand: launch first.")
    if code < 0:
        raise ValueError("No live flight is running.")
    return {"ok": True}
