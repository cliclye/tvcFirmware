"""Launch-side tools for EasyTVC Studio: rocket config, build/flash, live board, flight logs.

Python standard library only. Everything runs on this Mac; nothing is uploaded anywhere.
"""
from __future__ import annotations

from collections import deque
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import plistlib
import re
import select
import shlex
import subprocess
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
    ("NOSE_AXIS", "axis", None, None),
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

def image_info(project: Path) -> dict:
    config_mtime = config_path(project).stat().st_mtime
    images = {}
    for key in ("bench", "flight"):
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
    if image not in ("bench", "flight"):
        raise ValueError("Choose the bench or the flight image.")
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


def summarize_session(project, data: bytes, source: str) -> dict:
    session, header, rows = _viewer(project).parse_session(data)
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
    result = subprocess.run(["diskutil", "list", "-plist", "external", "physical"],
                            capture_output=True, timeout=15)
    if result.returncode:
        return []
    disks = []
    for disk in plistlib.loads(result.stdout).get("WholeDisks", []):
        info = subprocess.run(["diskutil", "info", "-plist", disk], capture_output=True, timeout=15)
        meta = plistlib.loads(info.stdout) if info.returncode == 0 else {}
        disks.append({"id": disk, "name": meta.get("MediaName", "Unknown"),
                      "size_gb": round(meta.get("Size", meta.get("TotalSize", 0)) / 1e9, 1)})
    return disks


def sd_script(device: str, out: Path, session: int | None) -> str:
    """One script so that macOS asks for the password at most once. Read-only: dd from the card."""
    return f"""#!/bin/sh
set -e
DEV={shlex.quote(device)}
OUT={shlex.quote(str(out))}
dd if="$DEV" of="$OUT/dir.bin" bs=512 skip={DIR_SECTOR} count=1 2>/dev/null
MAGIC=$(head -c 8 "$OUT/dir.bin")
if [ "$MAGIC" != "ETVCDIR1" ]; then echo nodir > "$OUT/result"; chmod 644 "$OUT"/*; exit 0; fi
NEXT=$(od -An -t u4 -j 8 -N 4 "$OUT/dir.bin" | tr -d ' ')
SESSION={'' if session is None else int(session)}
if [ -z "$SESSION" ]; then SESSION=$((NEXT - 1)); fi
START=$(({FIRST_SESSION} + (SESSION % {MAX_SESSIONS}) * {SESSION_SECTORS}))
dd if="$DEV" of="$OUT/session.bin" bs=512 skip=$START count={SESSION_SECTORS} 2>/dev/null
echo "$NEXT $SESSION" > "$OUT/result"
chmod 644 "$OUT"/*
"""


def read_sd(project, disk: str, session: int | None) -> dict:
    if not isinstance(disk, str) or not re.fullmatch(r"disk[0-9]+", disk):
        raise ValueError("Choose a disk from the list.")
    if disk not in {d["id"] for d in external_disks()}:
        raise ValueError("That disk is not an external card reader.")
    if session is not None and (isinstance(session, bool) or not isinstance(session, int) or session < 0):
        raise ValueError("Session must be a whole number.")
    subprocess.run(["diskutil", "unmountDisk", f"/dev/{disk}"], capture_output=True, timeout=30)
    device = f"/dev/r{disk}"
    out = Path(tempfile.mkdtemp(prefix="easytvc-sd-"))
    script = out / "read.sh"
    script.write_text(sd_script(device, out, session))
    readable = os.access(device, os.R_OK)
    if readable:
        result = subprocess.run(["/bin/sh", str(script)], capture_output=True, text=True, timeout=300)
    else:
        command = f"do shell script {json.dumps('/bin/sh ' + shlex.quote(str(script)))} with administrator privileges"
        result = subprocess.run(["osascript", "-e", command], capture_output=True, text=True, timeout=300)
    if result.returncode:
        message = (result.stderr or result.stdout).strip()
        raise ValueError("Reading the card was cancelled." if "-128" in message else f"Could not read the card: {message}")
    status = (out / "result").read_text().split()
    if status == ["nodir"]:
        raise ValueError("No EasyTVC logs on this card (it has never been used in the board).")
    next_session, chosen = int(status[0]), int(status[1])
    if next_session == 0:
        raise ValueError("The card has no sessions yet.")
    data = (out / "session.bin").read_bytes()
    saved = flights_dir() / f"session_{chosen}.bin"
    saved.write_bytes(data)
    result = summarize_session(project, data, f"SD card {disk}, saved to {saved}")
    result["available_sessions"] = list(range(max(0, next_session - MAX_SESSIONS), next_session))
    for file in out.iterdir():
        file.unlink()
    out.rmdir()
    return result


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
    allowed = {"checklist", "vehicle", "flashed", "packing"}
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
