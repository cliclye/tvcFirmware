from contextlib import redirect_stdout, redirect_stderr
import hashlib
import http.client
import importlib.util
import io
import json
import math
import os
from pathlib import Path
import pty
import struct
import subprocess
import sys
import threading
import tempfile
import tty
import unittest
from unittest.mock import patch
import zlib

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "studio"))
import studio

spec = importlib.util.spec_from_file_location("dfu", ROOT / "scripts" / "dfu.py")
dfu = importlib.util.module_from_spec(spec)
spec.loader.exec_module(dfu)


class StudioTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls): cls.native = studio.NativeController()

    def test_settings_match_c(self):
        p = studio.defaults()
        p["servos"][1]["direction"] = -1
        p["servos"][1]["channel"] = 4
        packet = self.native.validate(p)
        parsed = studio.decode_settings(packet)
        self.assertEqual(parsed["servos"], p["servos"])
        self.assertAlmostEqual(parsed["axes"][0]["kp"], p["axes"][0]["kp"], places=5)
        for i in range(len(packet)):
            corrupt = bytearray(packet)
            corrupt[i] ^= 1
            self.assertEqual(self.native.lib.EasyTVC_StudioValidate(bytes(corrupt), len(corrupt)), 0)

    def test_invalid_profiles(self):
        for value in (None, True, "1", float("nan"), float("inf"), -1, 101):
            p = studio.defaults(); p["axes"][0]["kp"] = value
            with self.assertRaises(ValueError): self.native.validate(p)
        for key, value in (("center_us", 800), ("channel", 1.5), ("direction", True)):
            p = studio.defaults(); p["servos"][0][key] = value
            with self.assertRaises(ValueError): self.native.validate(p)
        p = studio.defaults(); p["servos"][1]["channel"] = 1
        with self.assertRaises(ValueError): self.native.validate(p)
        p = studio.defaults(); p["unknown"] = 1
        with self.assertRaises(ValueError): self.native.validate(p)

    def test_model_uses_gains_and_wind(self):
        p = studio.defaults()
        p["plant"]["wind_m_s"] = p["plant"]["gust_m_s"] = 0
        calm = self.native.simulate(p)
        self.assertFalse(calm["metrics"]["fault"])
        self.assertLess(calm["metrics"]["final_tilt_deg"], .1)
        p["axes"][0]["kp"] = p["axes"][0]["ki"] = p["axes"][0]["kd"] = 0
        no_control = self.native.simulate(p)
        self.assertGreater(no_control["metrics"]["final_tilt_deg"], 2.9)
        p = studio.defaults(); p["plant"]["wind_m_s"] = 30; p["plant"]["gust_m_s"] = 40
        windy = self.native.simulate(p)
        self.assertTrue(windy["metrics"]["fault"])
        self.assertTrue(all(math.isfinite(value) for row in windy["rows"] for value in row))
        fault_rows = [r for r in windy["rows"] if r[8]]
        self.assertTrue(all(row[3] == row[4] == 0 for row in fault_rows))

    def test_weather_never_assumes_validation(self):
        p = studio.defaults()
        self.assertFalse(studio.weather_assessment(p)["within_entered_limits"])
        p["weather"]["limits_validated"] = True
        self.assertTrue(studio.weather_assessment(p)["within_entered_limits"])
        p["weather"]["wet"] = True
        self.assertFalse(studio.weather_assessment(p)["within_entered_limits"])
        p = studio.defaults(); p["plant"]["gust_m_s"] = 1
        with self.assertRaises(ValueError): self.native.validate(p)

    def test_serial_crc_and_sequence(self):
        packet = studio.frame(42, "OK", "EASYTVC,1,VERIFIED")
        self.assertEqual(studio.parse_frame(packet), (42, "OK", "EASYTVC,1,VERIFIED"))
        with self.assertRaises(ValueError): studio.parse_frame(packet.replace(b"VERIFIED", b"XERIFIED"))
        with self.assertRaises(ValueError): studio.parse_frame(packet[:-1])
        with self.assertRaises(ValueError): studio.parse_frame(studio.frame(0x100000000, "OK"))

    def test_real_c_protocol_over_serial_pty(self):
        master, slave = pty.openpty()
        tty.setraw(master); tty.setraw(slave)
        process = subprocess.Popen([str(ROOT / "build-host" / "easytvc_protocol_fixture"), "--verified-fixture"],
                                   stdin=master, stdout=master, stderr=subprocess.PIPE)
        device = studio.Device()
        device.fd = slave
        device.path = "test-pty"
        try:
            self.assertEqual(device.request("HELLO"), "EASYTVC,1,VERIFIED")
            device.identity = "EASYTVC,1,VERIFIED"
            p = studio.defaults(); p["axes"][0]["kp"] = 2.5
            p["servos"][1]["direction"] = -1
            self.assertIn("verified", device.write_settings(p, self.native)["message"])
            readback = device.read_settings(self.native)
            self.assertEqual(readback["axes"][0]["kp"], 2.5)
            self.assertEqual(readback["servos"][1]["direction"], -1)
            with self.assertRaises(ValueError): device.request("ARM")
            # Malformed oversized input must not desynchronize subsequent frames.
            os.write(slave, b"X"*500 + b"\n")
            self.assertEqual(device.request("HELLO"), "EASYTVC,1,VERIFIED")
            process.terminate(); process.wait(timeout=3)
            with self.assertRaises((OSError, TimeoutError)): device.request("STATUS")
            self.assertIsNone(device.fd)
        finally:
            device.close()
            if process.poll() is None: process.terminate(); process.wait(timeout=3)
            process.stderr.close()
            os.close(master)

    def test_telemetry_validation(self):
        d = studio.Device()
        d._telemetry(b"1,0,0,0,0,0,0,0,0,0,0,1\n")
        self.assertEqual(len(d.telemetry), 1)
        for bad in (b"nan\n", b"1,99,0,0,0,0,0,0,0,0,0,1\n", b"1,0,0,0,0,0,0,0,0,9,0,1\n"):
            d._telemetry(bad)
        self.assertEqual(len(d.telemetry), 1)

    def test_dfu_only_targets_internal_flash(self):
        listing = 'Found DFU: [0483:df11] ver=2200, path="1-2", alt=0, name="@Internal Flash /0x08000000/04*016Kg,01*064Kg,07*128Kg", serial="1234"\n'
        self.assertEqual(dfu.discover(listing)[0]["serial"], "1234")
        self.assertEqual(dfu.discover(listing.replace("alt=0", "alt=1")), [])
        self.assertEqual(dfu.discover(listing.replace("0483:df11", "1209:0001")), [])
        self.assertEqual(dfu.discover(listing.replace("Internal Flash", "Option Bytes")), [])


class HTTPTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.server = studio.StudioServer(("127.0.0.1", 0), studio.NativeController())
        cls.thread = threading.Thread(target=cls.server.serve_forever, daemon=True)
        cls.thread.start()

    @classmethod
    def tearDownClass(cls):
        cls.server.shutdown(); cls.server.server_close(); cls.thread.join(timeout=2)

    def request(self, path, payload=None, token=True, origin=None, host=None):
        connection = http.client.HTTPConnection("127.0.0.1", self.server.server_port, timeout=4)
        headers = {"Content-Type": "application/json"}
        if token: headers["Authorization"] = "Bearer " + self.server.token
        if origin: headers["Origin"] = origin
        if host: headers["Host"] = host
        try:
            connection.request("POST" if payload is not None else "GET", path,
                               json.dumps(payload) if payload is not None else None, headers)
            response = connection.getresponse()
            return response.status, response.read()
        finally: connection.close()

    def test_auth_origin_and_host(self):
        self.assertEqual(self.request("/api/defaults")[0], 200)
        self.assertNotEqual(self.request("/api/defaults", token=False)[0], 200)
        self.assertEqual(self.request("/api/simulate", studio.defaults(), token=False)[0], 403)
        self.assertEqual(self.request("/api/simulate", studio.defaults(), origin="https://evil.example")[0], 403)
        self.assertEqual(self.request("/", host="evil.example")[0], 403)
        self.assertEqual(self.request("/../studio.py")[0], 404)

    def test_api_simulation_and_errors(self):
        code, body = self.request("/api/simulate", studio.defaults())
        self.assertEqual(code, 200)
        self.assertEqual(len(json.loads(body)["rows"]), 1001)
        self.assertEqual(self.request("/api/simulate", {})[0], 400)
        self.assertEqual(self.request("/api/simulate", {"schema": float("nan")})[0], 400)
        self.assertEqual(self.request("/api/connect", {"port": "/etc/passwd"})[0], 400)
        self.assertEqual(self.request("/api/status", {})[0], 200)
        self.assertEqual(self.request("/api/write", studio.defaults())[0], 400)


class DFUWorkflowTests(unittest.TestCase):
    def workflow(self, corrupt=False, wrong_hash=False):
        data = struct.pack("<II", 0x20020000, 0x08000009) + bytes(24)
        calls = []
        listing = 'Found DFU: [0483:df11] ver=2200, path="1-2", alt=0, name="@Internal Flash /0x08000000/04*016Kg,01*064Kg,07*128Kg", serial="1234"\n'
        def fake_run(args, timeout=120):
            calls.append(args)
            if "-l" in args: return listing
            self.assertIn("0483:df11", args)
            self.assertIn("1234", args)
            self.assertEqual(args[args.index("-a")+1], "0")
            self.assertFalse(any(":leave" in arg for arg in args))
            if "-D" in args:
                self.assertEqual(Path(args[-1]).read_bytes(), data)
            elif "-U" in args:
                Path(args[-1]).write_bytes(data[:-1] + b"x" if corrupt else data)
                self.assertEqual(args[args.index("-s")+1], "0x08000000:32")
            return "simulated DFU transfer"
        with tempfile.TemporaryDirectory(prefix="easytvc-dfu-test-") as directory:
            path = Path(directory)/"reviewed.bin"; path.write_bytes(data)
            digest = "0"*64 if wrong_hash else hashlib.sha256(data).hexdigest()
            argv = ["dfu.py", "flash", str(path), "--board", "easytvc-v0.1", "--sha256", digest]
            with patch.object(sys,"argv",argv), patch.object(dfu,"run",side_effect=fake_run), \
                 patch.object(dfu.shutil,"which",return_value="/fake/dfu-util"), \
                 redirect_stdout(io.StringIO()), redirect_stderr(io.StringIO()):
                if corrupt or wrong_hash:
                    with self.assertRaises(SystemExit) as error: dfu.main()
                    self.assertEqual(error.exception.code, 1)
                else: dfu.main()
        return calls

    def test_write_and_readback_use_identical_reviewed_bytes(self):
        calls = self.workflow()
        self.assertEqual(len(calls), 3)
        self.assertIn("-D",calls[1]); self.assertIn("-U",calls[2])

    def test_wrong_hash_never_accesses_usb(self):
        self.assertEqual(self.workflow(wrong_hash=True), [])

    def test_corrupt_readback_is_failure(self):
        self.assertEqual(len(self.workflow(corrupt=True)), 3)


if __name__ == "__main__": unittest.main(verbosity=2)
