"""
Ground link to the rocket over USART2: a USB-UART adapter or a DroneBridge ESP32 bridge.

Same E1 frames whichever way they travel (docs/wireless-telemetry.md):

    E1 <sequence> <ID> <payload> *<crc32>\\n

Transports: USB serial (a USB-UART adapter on TX2/RX2, or the DroneBridge ESP-NOW ground XIAO on
USB), UDP (DroneBridge Wi-Fi AP: listen on 14550 and send a keep-alive so it knows where to send),
or TCP (DroneBridge port 5760). All I/O runs on a background thread; the UI polls `status()`.

Read-only in flight: `request()` refuses everything but HELLO / PING unless the last state the board
reported was IDLE and recent. The firmware enforces the same rule on its side.
"""

from __future__ import annotations

import base64
from collections import deque
import glob
import os
from pathlib import Path
import select
import socket
import termios
import threading
import time
import tty
import zlib

STATES = ("IDLE", "ARMED", "BOOST", "COAST", "APOGEE", "DESCENT", "LANDED", "FAULT")
AXES = ("POS_X", "NEG_X", "POS_Y", "NEG_Y", "POS_Z", "NEG_Z")
UNSOLICITED = ("TLM", "SYS", "EVT", "BEN", "SNS", "SCK")
ALWAYS_ALLOWED = ("HELLO", "PING")
MAX_FRAME = 255
DRONEBRIDGE_HOST, UDP_PORT, TCP_PORT = "192.168.2.1", 14550, 5760
STALE_S = 1.0
HISTORY = 900          # TLM rows kept (>= 30 s at 20 Hz, 90 s at 10 Hz)

TLM_FLAGS = ("imu_ok", "baro_ok", "gyro_cal", "accel_sat", "gyro_sat", "tvc", "ctrl_fault", "chute", "armed", "recovery",
             "servo_test")
SYS_ERRORS = ("imu_fail", "baro_stale", "sd_missing", "config_bad", "recovery", "fault", "sd_errors", "sd_full",
              "link_drops", "late_ticks")
SENSOR_CHECKS = ("accel_id", "gyro_id", "baro_id", "accel_1g", "gyro_still", "imu_alive", "imu_rate", "pressure",
                 "baro_temp", "humidity", "baro_rate", "accel_temp", "sd", "supply")   # bit order of sensor_test.h
# CFG reply order (main.c link_handle) -> vehicle_config.h names; "m" = sent in thousandths
CFG_FIELDS = (("KP", "m"), ("KI", "m"), ("KD", "m"), ("INTEGRATOR_LIMIT", "m"), ("OUTPUT_RATE_LIMIT", "m"),
              ("DERIVATIVE_CUTOFF", "m"), ("MAX_TILT_DEG", "m"), ("SERVO_A_CHANNEL", ""), ("SERVO_A_MIN_US", ""),
              ("SERVO_A_CENTER_US", ""), ("SERVO_A_MAX_US", ""), ("SERVO_A_DIRECTION", ""), ("SERVO_B_CHANNEL", ""),
              ("SERVO_B_MIN_US", ""), ("SERVO_B_CENTER_US", ""), ("SERVO_B_MAX_US", ""), ("SERVO_B_DIRECTION", ""),
              ("SERVO_RATE_HZ", ""), ("LAUNCH_ACCEL_G", "m"), ("LAUNCH_CONFIRM_MS", ""), ("TVC_ENABLE_DELAY_MS", ""),
              ("CHUTE_CHANNEL", ""), ("CHUTE_CLOSED_US", ""), ("CHUTE_OPEN_US", ""), ("CHUTE_BACKUP_MS", ""),
              ("ARM_HOLD_MS", ""), ("ARM_MAX_TILT_DEG", "m"), ("DISARM_TILT_DEG", "m"), ("NOSE_AXIS", "axis"),
              ("LED_ACTIVE_LOW", ""), ("LED_BRIGHTNESS_PCT", ""), ("REQUIRE_SD", ""),
              ("PYRO_CHUTE_CHANNEL", ""), ("PYRO_CHUTE_DELAY_MS", ""), ("PYRO_FIRE_MS", ""),
              ("PYRO_OUTPUTS_COMPILED_IN", ""),   # the master lock in easytvc_safety.h, not a setting
              ("PYRO_DEPLOY_ALT_M", "m"), ("PYRO_BACKUP_MS", ""),
              ("RADIO_SETTINGS", ""))             # 1: settings sent over the link are in use, not a setting
CFG_FIELDS_V1 = 32   # firmware 0.3 sent only the first 32
CFG_FIELD_COUNTS = (32, 36, 39)                    # 0.3, 0.4 (pyro), 0.5 (altitude trigger, link settings)

# Settings that can be sent over the link (Core/Inc/tunables.h, same order): (name, scale on the wire)
TUNABLE_FIELDS = (("KP", 1000), ("KI", 1000), ("KD", 1000), ("INTEGRATOR_LIMIT", 1000), ("OUTPUT_RATE_LIMIT", 1000),
                  ("DERIVATIVE_CUTOFF", 1000), ("MAX_TILT_DEG", 1000), ("SERVO_A_CENTER_US", 1), ("SERVO_B_CENTER_US", 1),
                  ("CHUTE_BACKUP_MS", 1), ("PYRO_DEPLOY_ALT_M", 100), ("PYRO_CHUTE_DELAY_MS", 1), ("PYRO_FIRE_MS", 1),
                  ("PYRO_BACKUP_MS", 1))
LOG_CHUNK = 128
SESSION_FIRST, SESSION_SECTORS, SECTOR = 2048, 16384, 512


# ---- frames --------------------------------------------------------------------------------------

def encode_frame(sequence: int, ident: str, payload: str = "-") -> bytes:
    if not ident or not payload or any(c <= " " or c > "~" for c in ident + payload):
        raise ValueError("Frame fields cannot contain spaces.")
    body = f"E1 {sequence & 0xffffffff} {ident} {payload}".encode("ascii")
    frame = body + f" *{zlib.crc32(body):08x}\n".encode("ascii")
    if len(frame) > MAX_FRAME:
        raise ValueError("Frame too long.")
    return frame


def decode_frame(line: bytes):
    """(sequence, id, payload) for a valid E1 line, else None. Never raises."""
    try:
        if len(line) > MAX_FRAME or not line.startswith(b"E1 "):
            return None
        line = line.rstrip(b"\r\n")
        body, sep, crc = line.rpartition(b" *")
        if not sep or len(crc) != 8 or int(crc, 16) != zlib.crc32(body):
            return None
        parts = body.decode("ascii").split(" ")
        if len(parts) != 4 or not parts[1].isdecimal() or int(parts[1]) > 0xffffffff or not parts[2] or not parts[3]:
            return None
        return int(parts[1]), parts[2], parts[3]
    except (ValueError, UnicodeError):
        return None


def _ints(payload: str, count: int):
    fields = payload.split(",")
    if len(fields) != count:
        raise ValueError("field count")
    return [int(x) for x in fields]


def parse_message(ident: str, payload: str):
    """(kind, dict) for a board message, else None."""
    try:
        if ident == "TLM":
            v = [int(x) for x in payload.split(",")]
            if len(v) not in (23, 24, 26):     # 23 before firmware 0.4, 24 without the barometer fields
                raise ValueError("field count")
            return "tlm", {"ms": v[0], "state": STATES[v[1]] if 0 <= v[1] < 8 else str(v[1]), "state_id": v[1],
                           "since_launch_s": None if v[2] < 0 else v[2] / 1000,
                           "q": [x / 1e4 for x in v[3:7]], "gyro_dps": [x / 1000 * 57.29578 for x in v[7:10]],
                           "accel_g": [x / 1000 for x in v[10:13]], "alt_m": v[13] / 100, "vspeed": v[14] / 100,
                           "tilt_x": v[15] / 100, "tilt_y": v[16] / 100, "cmd": [v[17] / 1000, v[18] / 1000],
                           "pulse": v[19:22], "flags": {n: bool(v[22] >> i & 1) for i, n in enumerate(TLM_FLAGS)},
                           "deploy_eta_s": None if len(v) < 24 or v[23] < 0 else v[23] / 1000,
                           "pressure_pa": v[24] if len(v) >= 26 and v[24] > 0 else None,
                           "baro_temp_c": v[25] / 100 if len(v) >= 26 and v[24] > 0 else None}
        if ident == "SYS":
            v = _ints(payload, 23)
            errors = {n: bool(v[22] >> i & 1) for i, n in enumerate(SYS_ERRORS)}
            return "sys", {"ms": v[0], "image": ("flight", "bench", "sensortest", "pyrobench")[v[1]] if 0 <= v[1] < 4 else str(v[1]),
                           "state": STATES[v[2]] if 0 <= v[2] < 8 else str(v[2]), "state_id": v[2], "armed": v[3],
                           "chute": v[4], "batt_pin_v": v[5] / 1000, "vdda_v": v[6] / 1000,
                           "continuity": [bool(v[7] >> i & 1) for i in range(3)], "pyro_armed": v[8], "pyro_fired": v[9],
                           "sd": v[10], "session": v[11], "records": v[12], "dropped": v[13], "write_errors": v[14],
                           "sd_left_kb": v[15], "loop_max_us": v[16], "late_ticks": v[17], "imu_errors": v[18],
                           "baro_age_ms": v[19], "link_dropped": v[20], "link_ignored": v[21], "errors": errors,
                           # the keys the older status tiles use
                           "imu": int(not errors["imu_fail"]), "baro": int(not errors["baro_stale"]),
                           "recovery": int(errors["recovery"])}
        if ident == "EVT":
            ms, _, text = payload.partition(",")
            return "event", {"ms": int(ms), "text": text}
        if ident == "BEN":
            v = _ints(payload, 9)
            return "bench", {"ms": v[0], "tilt_x": v[1] / 100, "tilt_y": v[2] / 100, "pulse_a": v[3], "pulse_b": v[4],
                             "raw": [v[5] / 1000, v[6] / 1000, v[7] / 1000], "body_z": v[8] / 1000}
        if ident == "SNS":
            return parse_legacy("T," + payload)
        if ident == "SCK":
            return parse_legacy("Q," + payload)
    except (ValueError, IndexError):
        return None
    return None


def parse_legacy(text: str):
    """Text lines from firmware older than the E1 link (no CRC): (kind, dict) or None."""
    parts = text.strip().split(",")
    try:
        if parts[0] == "T" and len(parts) == 15:
            v = [int(x) for x in parts[1:]]
            return "sensors", {"ms": v[0], "accel_g": [v[1] / 1000, v[2] / 1000, v[3] / 1000],
                               "gyro_dps": [v[4] / 100, v[5] / 100, v[6] / 100], "imu_temp_c": v[7] / 100,
                               "pressure_pa": v[8], "baro_temp_c": v[9] / 100, "humidity_pct": v[10] / 10,
                               "vdda_v": v[11] / 1000, "batt_pin_v": v[12] / 1000, "mcu_temp_c": v[13] / 100}
        if parts[0] == "Q" and len(parts) == 17:
            v = [int(x) for x in parts[1:]]
            return "sensor_checks", {"ms": v[0], "checks": {n: bool(v[1] >> i & 1) for i, n in enumerate(SENSOR_CHECKS)},
                                     "boot": {n: bool(v[2] >> i & 1) for i, n in enumerate(SENSOR_CHECKS)},
                                     "ids": [v[3], v[4], v[5]], "imu_samples": v[6], "imu_errors": v[7],
                                     "baro_samples": v[8], "baro_errors": v[9], "sd": v[10], "records": v[11],
                                     "dropped": v[12], "write_errors": v[13], "session": v[14], "overruns": v[15]}
        if parts[0] == "S" and len(parts) == 13:
            v = [int(x) for x in parts[1:]]
            return "status", {"imu": v[0], "baro": v[1], "sd": v[2], "gyro_cal": v[3], "records": v[4], "dropped": v[5],
                              "write_errors": v[6], "session": v[7],
                              "state": STATES[v[8]] if 0 <= v[8] < 8 else str(v[8]), "armed": v[9],
                              "recovery": v[10], "chute": v[11]}
        if parts[0] == "B" and len(parts) == 10:
            v = [int(x) for x in parts[1:]]
            return "bench", {"ms": v[0], "tilt_x": v[1] / 100, "tilt_y": v[2] / 100, "pulse_a": v[3], "pulse_b": v[4],
                             "raw": [v[5] / 1000, v[6] / 1000, v[7] / 1000], "body_z": v[8] / 1000}
        if len(parts) == 12:
            v = [int(x) for x in parts]
            if 0 <= v[1] < 8 and all(x in (0, 1) for x in v[9:]):
                return "flight", {"ms": v[0], "state": STATES[v[1]], "roll": v[2] / 1000, "pitch": v[3] / 1000,
                                  "yaw": v[4] / 1000, "alt": v[5] / 1000, "vspeed": v[6] / 1000,
                                  "cmd_a": v[7] / 1000, "cmd_b": v[8] / 1000, "tvc": v[9], "armed": v[10]}
    except (ValueError, IndexError):
        return None
    return None


def parse_cfg(payload: str) -> dict:
    v = [int(x) for x in payload.split(",")]
    if len(v) not in CFG_FIELD_COUNTS:
        raise ValueError("The board's config reply has the wrong number of fields (older firmware?).")
    out = {}
    for (name, kind), value in zip(CFG_FIELDS[:len(v)], v):
        out[name] = value / 1000 if kind == "m" else AXES[value - 1] if kind == "axis" and 1 <= value <= 6 else value
    return out


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


# ---- link quality --------------------------------------------------------------------------------

class LinkStats:
    """Packet loss from gaps in the board's unsolicited sequence numbers."""
    def __init__(self):
        self.received = 0
        self.lost = 0
        self.bad = 0
        self.last_seq = None
        self.last_time = None
        self.times = deque(maxlen=400)

    def frame(self, sequence: int, now: float):
        self.received += 1
        self.times.append(now)
        self.last_time = now
        if self.last_seq is not None:
            gap = (sequence - self.last_seq) & 0xffffffff
            if 1 < gap < 10000:
                self.lost += gap - 1
            elif gap == 0 or gap >= 10000:
                pass          # a repeat, or the board rebooted: start counting again
        self.last_seq = sequence

    def loss_percent(self) -> float:
        total = self.received + self.lost
        return 100.0 * self.lost / total if total else 0.0

    def rate(self, now: float, window: float = 2.0) -> float:
        recent = [t for t in self.times if now - t <= window]
        return len(recent) / window

    def summary(self, now: float) -> dict:
        age = None if self.last_time is None else now - self.last_time
        return {"frames": self.received, "lost": self.lost, "bad": self.bad, "loss_pct": round(self.loss_percent(), 2),
                "per_s": round(self.rate(now), 1), "age_s": None if age is None else round(age, 2),
                "stale": age is None or age > STALE_S}


# ---- transports ----------------------------------------------------------------------------------

def serial_ports():
    return sorted(set(glob.glob("/dev/cu.usbserial*") + glob.glob("/dev/cu.usbmodem*") +
                      glob.glob("/dev/cu.SLAB*") + glob.glob("/dev/cu.wchusbserial*")))


class SerialTransport:
    kind = "serial"

    def __init__(self, path: str):
        if path not in serial_ports():
            raise ValueError("Choose an available USB serial port.")
        fd = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
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
        self.fd, self.label = fd, path

    def read(self, timeout: float) -> bytes:
        if not select.select([self.fd], [], [], timeout)[0]:
            return b""
        data = os.read(self.fd, 4096)
        if not data:
            raise OSError("The serial port closed.")
        return data

    def write(self, data: bytes):
        try:
            os.write(self.fd, data)
        except BlockingIOError:
            pass          # the line is busy: a lost request simply times out

    def close(self):
        try:
            os.close(self.fd)
        except OSError:
            pass


class UdpTransport:
    """DroneBridge Wi-Fi AP: it sends UDP only to clients it has heard from, so keep-alives matter."""
    kind = "udp"

    def __init__(self, host: str, port: int, listen_port: int, bind_host: str = "0.0.0.0"):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((bind_host, listen_port))
        self.remote = (host, port) if host else None
        self.label = f"UDP :{listen_port}" + (f" ↔ {host}:{port}" if host else "")

    def read(self, timeout: float) -> bytes:
        if not select.select([self.sock], [], [], timeout)[0]:
            return b""
        data, sender = self.sock.recvfrom(65535)
        if self.remote is None:
            self.remote = sender          # reply to whoever is sending
        return data

    def write(self, data: bytes):
        if self.remote is not None:
            try:
                self.sock.sendto(data, self.remote)
            except OSError:
                pass

    def close(self):
        self.sock.close()


class TcpTransport:
    kind = "tcp"

    def __init__(self, host: str, port: int):
        self.sock = socket.create_connection((host, port), timeout=4)
        self.sock.setblocking(False)
        self.label = f"TCP {host}:{port}"

    def read(self, timeout: float) -> bytes:
        if not select.select([self.sock], [], [], timeout)[0]:
            return b""
        data = self.sock.recv(65536)
        if not data:
            raise OSError("The TCP connection closed.")
        return data

    def write(self, data: bytes):
        try:
            self.sock.send(data)
        except (BlockingIOError, OSError):
            pass

    def close(self):
        self.sock.close()


BLE_RELAY_PORT = 14570        # BleRelay.swift, inside the Studio app


class BleTransport(UdpTransport):
    """Bluetooth LE to a DroneBridge in BLE mode, through the CoreBluetooth relay in the Studio app
    (Python's standard library has no Bluetooth). The relay sends its own status as whole "#BLE ..."
    datagrams, never mixed into the board's bytes, so they are taken out here before line assembly."""
    kind = "ble"

    def __init__(self, relay_port: int = BLE_RELAY_PORT):
        super().__init__("127.0.0.1", relay_port, 0, bind_host="127.0.0.1")
        self.label = "Bluetooth (via the Studio app)"
        self.opened = time.monotonic()
        self.relay = None              # {"state", "name", "rssi"} from the relay
        self.relay_at = None

    def read(self, timeout: float) -> bytes:
        data = super().read(timeout)
        if data.startswith(b"#BLE "):
            fields = dict(part.split("=", 1) for part in data[5:].decode("ascii", "replace").split() if "=" in part)
            self.relay = {"state": fields.get("state", "?"), "name": fields.get("name", "").replace("_", " "),
                          "rssi": int(fields["rssi"]) if fields.get("rssi", "").lstrip("-").isdigit() else None}
            self.relay_at = time.monotonic()
            return b""
        return data

    def info(self) -> dict:
        now = time.monotonic()
        if self.relay_at is None or now - self.relay_at > 3:
            # nothing from the relay: Studio is running outside its app (python3 studio/studio.py)
            return {"state": "no_relay" if now - self.opened > 2.5 else "starting", "name": "", "rssi": None}
        return self.relay


def open_transport(kind: str, options: dict):
    if kind == "serial":
        return SerialTransport(str(options.get("port", "")))
    if kind == "udp":
        return UdpTransport(str(options.get("host", DRONEBRIDGE_HOST)), int(options.get("port", UDP_PORT)),
                            int(options.get("listen_port", UDP_PORT)))
    if kind == "tcp":
        return TcpTransport(str(options.get("host", DRONEBRIDGE_HOST)), int(options.get("port", TCP_PORT)))
    if kind == "ble":
        return BleTransport(int(options.get("relay_port", BLE_RELAY_PORT)))
    raise ValueError("Unknown link type.")


# ---- recorder ------------------------------------------------------------------------------------

RECORD_HEADER = "# EasyTVC live telemetry"


class Recorder:
    """Every received line, as `<unix_ms> <line>`, in the flights folder. Opens in Flight logs."""
    def __init__(self, folder: Path, label: str):
        folder.mkdir(parents=True, exist_ok=True)
        self.path = folder / time.strftime("live_%Y%m%d-%H%M%S.etlog")
        self.file = open(self.path, "w", encoding="ascii", errors="replace")
        self.file.write(f"{RECORD_HEADER} {time.strftime('%Y-%m-%dT%H:%M:%S')} via {label}\n")
        self.lines = 0
        self.last_flush = time.monotonic()

    def write(self, line: str, now: float):
        self.file.write(f"{int(time.time() * 1000)} {line}\n")
        self.lines += 1
        if now - self.last_flush > 1.0:
            self.file.flush()
            self.last_flush = now

    def close(self):
        self.file.close()
        if self.lines == 0:
            try:
                self.path.unlink()
            except OSError:
                pass


def read_recording(text: str):
    """Decoded messages from a recording: list of (host_ms, kind, dict)."""
    out = []
    for raw in text.splitlines():
        if not raw or raw.startswith("#"):
            continue
        stamp, _, line = raw.partition(" ")
        try:
            host_ms = int(stamp)
        except ValueError:
            continue
        frame = decode_frame((line + "\n").encode("ascii", "replace"))
        parsed = parse_message(frame[1], frame[2]) if frame else parse_legacy(line)
        if parsed:
            out.append((host_ms, parsed[0], parsed[1]))
    return out


# ---- the link ------------------------------------------------------------------------------------

class Monitor:
    """One connection to the board. Thread-safe; the HTTP server calls status() a few times a second."""

    def __init__(self, folder_fn=None):
        self.lock = threading.Lock()
        self.folder_fn = folder_fn
        self.transport = None
        self.thread = None
        self.stop = threading.Event()
        self._reset()

    def _reset(self):
        self.latest, self.updated = {}, {}
        self.lines = deque(maxlen=200)
        self.history = deque(maxlen=HISTORY)
        self.history_base = 0          # index of history[0] since connect
        self.events = deque(maxlen=200)
        self.event_keys = deque(maxlen=400)
        self.stats = LinkStats()
        self.recorder = None
        self.host_seq = 0
        self.pending = {}
        self.rtt_ms = None
        self.identity = None
        self.download = None
        self.connected_at = None

    # -- connection
    def connect(self, kind="serial", **options):
        if kind == "serial" and "port" not in options and "path" in options:
            options["port"] = options["path"]
        self.disconnect()
        transport = open_transport(kind, options)
        with self.lock:
            self._reset()
            self.transport = transport
            self.connected_at = time.monotonic()
            if self.folder_fn is not None:
                try:
                    self.recorder = Recorder(self.folder_fn(), transport.label)
                except OSError:
                    self.recorder = None
            self.stop = threading.Event()
        self.thread = threading.Thread(target=self._run, args=(transport, self.stop), daemon=True)
        self.thread.start()
        return self.status()

    def disconnect(self):
        with self.lock:
            transport, self.transport = self.transport, None
            self.stop.set()
            recorder, self.recorder = self.recorder, None
            for waiter in self.pending.values():
                waiter["event"].set()
        if transport is not None:
            transport.close()
        if recorder is not None:
            recorder.close()

    # -- receive
    def _run(self, transport, stop):
        pending = bytearray()
        last_ping = 0.0
        while not stop.is_set():
            now = time.monotonic()
            if now - last_ping >= 1.0:
                last_ping = now
                self._send_request("PING", "-", track=True)
            try:
                chunk = transport.read(0.1)
            except (OSError, ValueError):
                self.disconnect()
                return
            if not chunk:
                continue
            pending.extend(chunk)
            if len(pending) > 65536:
                pending.clear()
            while b"\n" in pending:
                raw, _, rest = pending.partition(b"\n")
                pending = bytearray(rest)
                self._line(raw + b"\n", time.monotonic())

    def _line(self, raw: bytes, now: float):
        text = raw.decode("ascii", "replace").strip()
        if not text:
            return
        with self.lock:
            if self.recorder is not None:
                self.recorder.write(text, now)
            self.lines.append(text)
            frame = decode_frame(raw)
            if raw.startswith(b"E1 ") and frame is None:
                self.stats.bad += 1
                return
            if frame is None:
                parsed = parse_legacy(text)
                if parsed:
                    self._store(parsed, now)
                return
            sequence, ident, payload = frame
            if ident in ("OK", "ERR"):
                waiter = self.pending.get(sequence)
                if waiter is not None:
                    waiter["reply"] = (ident, payload)
                    waiter["event"].set()
                    if waiter.get("ping"):
                        self.rtt_ms = round((now - waiter["sent"]) * 1000)
                        self.pending.pop(sequence, None)
                return
            if ident in UNSOLICITED:
                self.stats.frame(sequence, now)
            parsed = parse_message(ident, payload)
            if parsed:
                self._store(parsed, now)

    def _store(self, parsed, now):
        kind, data = parsed
        if kind == "event":
            key = (data["ms"], data["text"])
            if key in self.event_keys:
                return             # the board sends every event twice
            self.event_keys.append(key)
            self.events.append({**data, "host": time.time()})
            return
        self.latest[kind] = data
        self.updated[kind] = now
        if kind == "sys":
            status = dict(data)
            tlm = self.latest.get("tlm")
            status["gyro_cal"] = int(tlm["flags"]["gyro_cal"]) if tlm else 1
            self.latest["status"] = status
            self.updated["status"] = now
        elif kind == "tlm":
            if len(self.history) == self.history.maxlen:
                self.history_base += 1      # the append below drops the oldest entry
            self.history.append({**data, "host": now})

    # -- requests
    def _send_request(self, command, payload, track=False):
        with self.lock:
            if self.transport is None:
                return None
            self.host_seq = (self.host_seq + 1) & 0xffffffff
            seq = self.host_seq
            waiter = {"event": threading.Event(), "reply": None, "sent": time.monotonic(), "ping": command == "PING"}
            self.pending[seq] = waiter
            transport = self.transport
            for old in [s for s, w in self.pending.items() if time.monotonic() - w["sent"] > 10]:
                self.pending.pop(old, None)
        transport.write(encode_frame(seq, command, payload))
        return seq, waiter

    def board_idle(self):
        """True only if the board recently reported IDLE, and nothing recent says otherwise
        (a fresh TLM in flight outweighs an older SYS from the pad)."""
        now = time.monotonic()
        fresh = [self.latest[kind]["state"] for kind in ("sys", "tlm")
                 if self.latest.get(kind) is not None and self.updated.get(kind) is not None
                 and now - self.updated[kind] <= 2.5]
        return bool(fresh) and all(state == "IDLE" for state in fresh)

    def request(self, command: str, payload: str = "-", timeout: float = 2.5, retries: int = 2):
        if command not in ALWAYS_ALLOWED:
            with self.lock:
                idle = self.board_idle()
            if not idle:
                raise ValueError("Blocked: the board isn't reporting IDLE on the pad (or no recent telemetry). "
                                 "Only telemetry is allowed in flight.")
        for _ in range(retries + 1):
            sent = self._send_request(command, payload)
            if sent is None:
                raise ValueError("Connect to the board first.")
            seq, waiter = sent
            if waiter["event"].wait(timeout) and waiter["reply"] is not None:
                with self.lock:
                    self.pending.pop(seq, None)
                status, reply = waiter["reply"]
                if status == "ERR":
                    if reply == "BUSY":
                        time.sleep(0.02)
                        continue
                    raise ValueError(f"The board refused {command}: {reply}")
                return reply
            with self.lock:
                self.pending.pop(seq, None)
                if self.transport is None:
                    raise ValueError("Disconnected.")
        raise TimeoutError(f"No reply to {command} (link dropped the frames?). Try again.")

    def read_config(self):
        return parse_cfg(self.request("CFG"))

    # -- settings over the link (IDLE only): sent, stored on the board's SD card, echoed back, compared
    def send_settings(self, values: dict) -> dict:
        """values: every TUNABLE_FIELDS name in Rocket-setup units. Returns the set the board now uses
        (what it read back from its card), and raises if it differs from what was sent."""
        missing = [name for name, _ in TUNABLE_FIELDS if name not in values]
        if missing:
            raise ValueError("Missing settings: " + ", ".join(missing))
        wire = []
        for name, scale in TUNABLE_FIELDS:
            v = values[name]
            if isinstance(v, bool) or not isinstance(v, (int, float)) or v != v:
                raise ValueError(f"{name} must be a number.")
            wire.append(round(v * scale))
        reply = self.request("SETCFG", ",".join(map(str, wire)), timeout=3.0, retries=3)
        echoed = [int(x) for x in reply.split(",")]
        if echoed != wire:
            raise ValueError("The board stored different values than were sent: nothing to trust. Read config and try again.")
        return {name: x / scale for (name, scale), x in zip(TUNABLE_FIELDS, echoed)}

    def reset_settings(self) -> dict:
        reply = self.request("CFGRESET", timeout=3.0, retries=3)
        return {name: int(x) / scale for (name, scale), x in zip(TUNABLE_FIELDS, reply.split(","))}

    # -- pyro channel ground test (IDLE + not armed only; a test load, never a charge)
    def pyro_arm(self, channel: int) -> dict:
        if channel not in (1, 3):
            raise ValueError("Test P1 or P3 (P2 can't be used).")
        ch, code, locked, continuity = (int(x) for x in self.request("PYROARM", str(channel), retries=0).split(","))
        return {"channel": ch, "code": code, "locked": bool(locked), "continuity": [bool(continuity >> i & 1) for i in range(3)]}

    def pyro_fire(self, channel: int, code: int, pulse_ms: int) -> dict:
        if channel not in (1, 3) or not 1000 <= code <= 9999 or not 20 <= pulse_ms <= 1000:
            raise ValueError("Arm the test first; pulses are 20-1000 ms.")
        # no retries: a lost reply must not fire twice (the board uses each code once anyway)
        ch, ms, locked, continuity = (int(x) for x in self.request("PYROFIRE", f"{channel},{code},{pulse_ms}", retries=0).split(","))
        return {"channel": ch, "pulse_ms": ms, "locked": bool(locked), "continuity": [bool(continuity >> i & 1) for i in range(3)]}

    def beep(self) -> dict:
        return {"ms": int(self.request("BEEP"))}

    # -- TVC gimbal test (IDLE only, servos A/B only; the board ends it the moment it leaves IDLE)
    GIMBAL_COMMANDS = ("SERVO", "SWEEP", "CENTER", "TRIM")

    def gimbal(self, command: str, a: int = 0, b: int = 0):
        """SERVO a,b in milli of travel (-1000..1000, held while repeated), SWEEP, CENTER, or TRIM a,b
        in us (-100..100). Returns {"mode", "trim_us"}."""
        if command not in self.GIMBAL_COMMANDS:
            raise ValueError("Unknown gimbal command.")
        limit = 1000 if command == "SERVO" else 100
        for v in (a, b):
            if isinstance(v, bool) or not isinstance(v, int) or not -limit <= v <= limit:
                raise ValueError(f"Gimbal values are whole numbers from -{limit} to {limit}.")
        payload = f"{a},{b}" if command in ("SERVO", "TRIM") else "-"
        # SERVO repeats several times a second while held: one quick try, no retries piling up
        reply = self.request(command, payload, timeout=0.6, retries=0) if command == "SERVO" else self.request(command, payload)
        fields = [int(x) for x in reply.split(",")]
        return {"mode": ("off", "hold", "sweep")[fields[0]] if 0 <= fields[0] < 3 else str(fields[0]), "trim_us": fields[1:3]}

    # -- log download (resumable)
    def start_download(self, session=None, save_fn=None):
        with self.lock:
            if self.download and self.download.get("running"):
                raise ValueError("A download is already running.")
            previous = self.download
        if session is None:
            info = self.request("LOGINFO").split(",")
            if info[0] != "1":
                raise ValueError("The board has no SD card ready.")
            session = int(info[1])
        resume = previous if previous and previous.get("session") == session and not previous.get("done") else None
        job = resume or {"session": session, "data": bytearray(), "offset": 0}
        job.pop("cancel", None)
        job.update(running=True, error=None, done=False, started=time.monotonic(), start_offset=job["offset"])
        with self.lock:
            self.download = job
        threading.Thread(target=self._download, args=(job, save_fn), daemon=True).start()
        return self.download_status()

    def cancel_download(self):
        with self.lock:
            if self.download:
                self.download["cancel"] = True

    def _download(self, job, save_fn, window=4, chunk_timeout=0.8):
        """Pipelined: up to `window` LOGREAD requests in flight, lost or corrupted chunks asked again.
        The firmware answers one per loop tick and only when its transmit buffer has room."""
        session, tag = job["session"], job["session"] & 0xFF
        region = SESSION_SECTORS * SECTOR
        chunks, inflight = {}, {}          # offset -> bytes; seq -> (offset, waiter, sent)
        next_offset, end_offset, attempts = job["offset"], None, 0
        try:
            while not job.get("cancel"):
                # assemble what has arrived, in order, and look for the end of the session
                while job["offset"] in chunks:
                    offset = job["offset"]
                    chunk = chunks.pop(offset)
                    if offset == 0 and (chunk[:8] != b"ETVCLOG2" or int.from_bytes(chunk[8:12], "little") != session):
                        raise ValueError(f"Session {session} is not on the card any more.")
                    end = None
                    if offset >= SECTOR:
                        for i in range(0, len(chunk), 32):
                            if chunk[i + 31] != 0xA5 or chunk[i + 28] != tag:
                                end = i
                                break
                    job["data"].extend(chunk if end is None else chunk[:end])
                    job["offset"] = offset + LOG_CHUNK
                    if end is not None or job["offset"] >= region:
                        end_offset = job["offset"]
                        break
                if end_offset is not None:
                    job["done"] = True
                    break
                # keep the pipe full
                with self.lock:
                    idle = self.board_idle()
                if not idle:
                    raise ValueError("Paused: the board isn't reporting IDLE any more. Resume when it's back on the pad.")
                while len(inflight) < window and next_offset < region:
                    sent = self._send_request("LOGREAD", f"{session},{next_offset}")
                    if sent is None:
                        raise ValueError("Disconnected.")
                    inflight[sent[0]] = (next_offset, sent[1], time.monotonic())
                    next_offset += LOG_CHUNK
                time.sleep(0.004)
                now = time.monotonic()
                for seq, (offset, waiter, sent_at) in list(inflight.items()):
                    if waiter["event"].is_set() and waiter["reply"] is not None:
                        inflight.pop(seq)
                        with self.lock:
                            self.pending.pop(seq, None)
                        status, reply = waiter["reply"]
                        fields = reply.split(",")
                        ok = status == "OK" and len(fields) == 5 and int(fields[0]) == session and int(fields[1]) == offset
                        if ok:
                            chunk = base64.b64decode(fields[4])
                            ok = len(chunk) == int(fields[2]) == LOG_CHUNK and zlib.crc32(chunk) == int(fields[3], 16)
                        if ok:
                            chunks[offset] = chunk
                        elif status == "ERR" and reply not in ("BUSY",):
                            raise ValueError(f"The board refused LOGREAD: {reply}")
                        else:
                            self._resend(inflight, session, offset)   # BUSY or damaged: ask again
                    elif now - sent_at > chunk_timeout:
                        inflight.pop(seq)
                        with self.lock:
                            self.pending.pop(seq, None)
                        attempts += 1
                        if attempts > 2000:
                            raise TimeoutError("Too many lost chunks: move closer or check the link.")
                        self._resend(inflight, session, offset)
            if job["done"] and save_fn is not None:
                job["saved"] = save_fn(bytes(job["data"]), session)
        except (ValueError, TimeoutError, OSError) as error:
            job["error"] = str(error)
        finally:
            for seq in inflight:
                with self.lock:
                    self.pending.pop(seq, None)
            job["running"] = False

    def _resend(self, inflight, session, offset):
        sent = self._send_request("LOGREAD", f"{session},{offset}")
        if sent is not None:
            inflight[sent[0]] = (offset, sent[1], time.monotonic())

    def download_status(self):
        job = self.download
        if not job:
            return None
        elapsed = max(0.001, time.monotonic() - job["started"])
        speed = (job["offset"] - job["start_offset"]) / elapsed
        return {"session": job["session"], "bytes": len(job["data"]), "offset": job["offset"], "running": job["running"],
                "done": job["done"], "error": job["error"], "saved": job.get("saved"),
                "speed_kb_s": round(speed / 1024, 2), "cancelled": bool(job.get("cancel"))}

    # -- status for the UI
    def status(self, since: int | None = None):
        with self.lock:
            now = time.monotonic()
            bench = self.latest.get("bench")
            first = self.history_base
            history = list(self.history)
            start = 0 if since is None else max(0, since - first)
            return {"connected": self.transport is not None,
                    "transport": None if self.transport is None else self.transport.kind,
                    "port": None if self.transport is None else self.transport.label,
                    "ports": serial_ports(), "latest": dict(self.latest),
                    "age": {k: round(now - t, 2) for k, t in self.updated.items()},
                    "lines": list(self.lines)[-40:], "link": {**self.stats.summary(now), "rtt_ms": self.rtt_ms},
                    "history": history[start:], "history_next": first + len(history),
                    "events": list(self.events)[-60:],
                    "recording": None if self.recorder is None else {"file": self.recorder.path.name, "lines": self.recorder.lines},
                    "ble": self.transport.info() if isinstance(self.transport, BleTransport) else None,
                    "idle": self.board_idle(), "download": self.download_status(),
                    "defaults": {"host": DRONEBRIDGE_HOST, "udp_port": UDP_PORT, "tcp_port": TCP_PORT},
                    "suggested_nose_axis": suggest_nose_axis(bench["raw"]) if bench else None}
