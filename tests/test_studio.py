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

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "studio"))
import studio

spec = importlib.util.spec_from_file_location("dfu", ROOT / "scripts" / "dfu.py")
dfu = importlib.util.module_from_spec(spec)
spec.loader.exec_module(dfu)


class FlightSim3DTests(unittest.TestCase):
    """The 3D sim runs the firmware in the loop; check the Studio wiring end to end."""
    @classmethod
    def setUpClass(cls):
        cls.native = studio.NativeController()
        cls.config = studio.launch.read_config(studio.launch.find_project())
        cls.vehicle = studio.launch.upgrade_vehicle({})   # the default 610 mm / 380 g rocket model

    def test_configured_rocket_flies(self):
        result = studio.launch.run_sim3d(self.native, self.config, self.vehicle, studio.launch.sim_defaults())
        s = result["summary"]
        self.assertGreater(len(result["rows"]), 100)
        self.assertEqual(s["tvc_fault"], 0)
        self.assertLess(s["max_boost_tilt"], 10)
        self.assertGreater(s["chute_t"], 0)
        self.assertTrue(any(v["level"] == "good" for v in result["verdict"]))

    def test_reversed_servo_is_caught(self):
        sim = studio.launch.sim_defaults(); sim["reverse_servo_a"] = 1
        result = studio.launch.run_sim3d(self.native, self.config, self.vehicle, sim)
        self.assertEqual(result["summary"]["tvc_fault"], 1)
        self.assertTrue(any(v["level"] == "bad" for v in result["verdict"]))

    def test_bad_inputs_rejected(self):
        sim = studio.launch.sim_defaults(); sim["gust_m_s"] = 0; sim["wind_m_s"] = 5
        with self.assertRaises(ValueError):
            studio.launch.run_sim3d(self.native, self.config, self.vehicle, sim)
        bad = studio.launch.upgrade_vehicle({}); bad["parts"].append(["lost", 10, 5000, 10])   # beyond the nose-to-tail length
        with self.assertRaises(ValueError):
            studio.launch.run_sim3d(self.native, self.config, bad, studio.launch.sim_defaults())

    def test_rocket_model_mass_properties(self):
        """Hand-checkable numbers: two point masses and the motor."""
        v = studio.launch.upgrade_vehicle({})
        v["parts"] = [["front", 100, 100, 0], ["back", 100, 500, 0]]
        v["motor"].update(total_mass_g=50, prop_mass_g=20, length_mm=100, diameter_mm=24)
        v["body"].update(length_mm=450, nose_length_mm=150, motor_overhang_mm=0, pivot_from_nozzle_mm=50, nose_shape="conical")
        m = studio.launch.rocket_model(v)
        self.assertAlmostEqual(m["length_mm"], 600)
        self.assertAlmostEqual(m["motor_center_mm"], 550)
        self.assertAlmostEqual(m["pivot_mm"], 550)
        self.assertAlmostEqual(m["cg_wet_mm"], (100 * 100 + 100 * 500 + 50 * 550) / 250)
        self.assertAlmostEqual(m["cg_dry_mm"], (100 * 100 + 100 * 500 + 30 * 550) / 230)
        self.assertAlmostEqual(m["cp_mm"], 100)   # cone: 2/3 of the nose length
        self.assertAlmostEqual(m["stability_cal"], (100 - m["cg_wet_mm"]) / 66)

    def test_linkage_matches_c(self):
        m = studio.launch.rocket_model({})
        self.assertAlmostEqual(studio.launch.gimbal_angle_deg(250, m), math.degrees(math.asin(0.35 * math.sin(math.radians(22.5)))), places=6)
        self.assertEqual(studio.launch.gimbal_angle_deg(700, m), m["stop_deg"])   # 63 deg of horn would be 18 deg: clamped

    def test_batch(self):
        result = studio.launch.run_sim3d_batch(self.native, self.config, self.vehicle, studio.launch.sim_defaults(), 10)
        self.assertEqual(result["count"], 10)
        self.assertGreaterEqual(result["success"], 8)


class SensorTestImageTests(unittest.TestCase):
    """The sensor-test firmware's SD records and serial lines, read by Studio."""

    def test_sd_records_match_the_c_layout(self):
        with tempfile.TemporaryDirectory() as tmp:
            exe, out = Path(tmp) / "fixture", Path(tmp) / "session.bin"
            subprocess.run(["cc", "-std=c11", "-Wall", "-Werror", "-I", str(ROOT / "Core" / "Inc"),
                            str(ROOT / "tests" / "sensor_test_fixture.c"), "-o", str(exe)], check=True)
            subprocess.run([str(exe), str(out)], check=True)
            result = studio.launch.summarize_session(ROOT, out.read_bytes(), "test")
        self.assertEqual(result["kind"], "sensortest")
        r = result["report"]
        self.assertTrue(r["passed"])
        self.assertEqual(r["counts"]["gaps"], 0)
        self.assertEqual((r["counts"]["imu"], r["counts"]["baro"], r["counts"]["adc"]), (4000, 500, 100))
        self.assertAlmostEqual(r["rates_hz"]["imu"], 400, delta=1)
        self.assertEqual(r["ids"], {"accel_id": 0x1E, "gyro_id": 0x0F, "baro_id": 0x60})
        self.assertAlmostEqual(r["channels"]["accel_z_g"]["mean"], 1.0, delta=0.002)
        self.assertAlmostEqual(r["channels"]["gyro_y_dps"]["mean"], -16 * 2000 / 32768, places=4)
        self.assertEqual(r["channels"]["pressure_pa"]["mean"], 101325)
        self.assertAlmostEqual(r["channels"]["vdda_v"]["mean"], 3.301)
        self.assertEqual([c["key"] for c in r["checks"] if not c["now"]], ["humidity"])
        self.assertEqual(studio.launch.sensor_csv_text(result["data_id"])["rows"], r["counts"]["records"])
        zoom = studio.launch.sensor_window(result["data_id"], 2.0, 2.1)   # zoomed in: every sample
        self.assertEqual(zoom["imu"]["step"], 1)
        self.assertTrue(40 <= len(zoom["imu"]["t"]) <= 43)
        with self.assertRaises(ValueError):
            studio.launch.sensor_window("missing", 0, 1)

    def _fixture_session(self, tmp):
        exe, out = Path(tmp) / "fixture", Path(tmp) / "session.bin"
        subprocess.run(["cc", "-std=c11", "-Wall", "-Werror", "-I", str(ROOT / "Core" / "Inc"),
                        str(ROOT / "tests" / "sensor_test_fixture.c"), "-o", str(exe)], check=True)
        subprocess.run([str(exe), str(out)], check=True)
        return out.read_bytes()

    def test_card_dump_and_saved_library(self):
        launch = studio.launch
        with tempfile.TemporaryDirectory() as tmp:
            session = self._fixture_session(tmp)          # session 7, 4610 records
            card = Path(tmp) / "card.img"
            with open(card, "wb") as f:                   # sparse 8 GB-style layout, only what we need
                f.write(b"\0" * 512)
                f.seek(1024 * 512); f.write(b"ETVCDIR1" + struct.pack("<I", 8))
                f.seek((2048 + 7 * 16384) * 512); f.write(session)
                f.seek((2048 + 6 * 16384) * 512); f.write(b"ETVCLOG2" + struct.pack("<II", 38, 32))   # overwritten slot
                f.truncate((2048 + 32 * 16384) * 512)
            out = Path(tmp) / "out"; out.mkdir()
            result = launch._run_dump(ROOT, str(card), out, ["--all"])
            self.assertEqual([s["session"] for s in result["sessions"]], [7])
            dumped = (out / "session_7.bin").read_bytes()
            self.assertEqual(len(dumped), 512 + 4610 * 32)  # only the used records, not 8 MB
            self.assertEqual(dumped, session[:len(dumped)])
            folder = Path(tmp) / "flights"; folder.mkdir()
            with patch.object(launch, "flights_dir", lambda create=True: folder):
                first, new = launch._save_session(dumped, 7)
                again, new_again = launch._save_session(dumped, 7)
                self.assertTrue(new); self.assertFalse(new_again); self.assertEqual(first, again)
                other, _ = launch._save_session(dumped[:-32], 7)   # same number, different data
                self.assertEqual(other.name, "session_7 (2).bin")
                listing = launch.saved_sessions(ROOT)["sessions"]
                self.assertEqual({s["kind"] for s in listing}, {"sensortest"})
                self.assertTrue(all(s["passed"] for s in listing))
                opened = launch.open_saved(ROOT, "session_7.bin")
                self.assertEqual((opened["kind"], opened["saved_name"]), ("sensortest", "session_7.bin"))
                for bad in ("../x.bin", "x.txt", ".hidden.bin", "missing.bin"):
                    with self.assertRaises(ValueError):
                        launch.open_saved(ROOT, bad)
                launch.delete_saved("session_7 (2).bin")
                self.assertEqual([s["name"] for s in launch.saved_sessions(ROOT)["sessions"]], ["session_7.bin"])

    def test_replay_in_3d(self):
        """A flight log replays with the board's own tilt, the barometer's height and the logged pulses."""
        launch = studio.launch
        header = bytearray(512); header[:8] = b"ETVCLOG2"; struct.pack_into("<II", header, 8, 3, 32)
        records = b""
        for i in range(600):
            state = 0 if i < 100 else 2 if i < 300 else 5 if i < 590 else 6
            tx, ty = (3 * math.sin(i / 40), -2 * math.cos(i / 30)) if state == 2 else (0.5, -0.3)
            alt = 0 if state == 0 else min(80, (i - 100) * 0.4) if state == 2 else max(0, 80 - (i - 300) * 0.3)
            records += struct.pack("<I3h3h2h2HhhBBBB", 1000 + 10 * i, 0, 0, 1000, 0, 0, 900 if state == 2 else 0,
                                   int(tx * 100), int(ty * 100), 1500 + int(tx * 20), 1500, int(alt * 10), 0, 3, state,
                                   0x07 | (0x80 if state >= 5 else 0), 0xA5)
        data = bytes(header) + records
        opened = launch.summarize_session(ROOT, data, "test")
        config = launch.read_config(ROOT)
        replay = launch.replay_session(ROOT, opened["data_id"], config, launch.upgrade_vehicle({}))
        col = {c: i for i, c in enumerate(replay["columns"])}
        self.assertEqual(tuple(replay["columns"]), launch.SIM3D_COLUMNS)
        _, _, log = launch._viewer(ROOT).parse_session(data)
        self.assertEqual(len(replay["rows"]), len(log))
        for row, rec in zip(replay["rows"], log):
            self.assertAlmostEqual(row[col["tilt_x"]], rec["tilt_x_deg"], places=2)
            self.assertAlmostEqual(row[col["tilt_y"]], rec["tilt_y_deg"], places=2)
            self.assertEqual(row[col["pulse_a"]], rec["servo_a_us"])
        self.assertAlmostEqual(replay["summary"]["apogee_m"], 80, delta=0.5)
        self.assertEqual(replay["ignition_s"], 2.0)   # first BOOST record, t = 1000 + 10*100 ms
        self.assertGreater(replay["summary"]["chute_t"], 0)
        with self.assertRaises(ValueError):
            launch.replay_session(ROOT, "gone", config, launch.upgrade_vehicle({}))

    def test_serial_lines(self):
        kind, t = studio.launch.parse_line("T,1234,10,-20,998,150,-5,0,2650,101325,2412,452,3301,1234,3100")
        self.assertEqual(kind, "sensors")
        self.assertEqual(t["accel_g"], [0.01, -0.02, 0.998])
        self.assertEqual(t["gyro_dps"], [1.5, -0.05, 0.0])
        self.assertEqual((t["pressure_pa"], t["humidity_pct"], t["vdda_v"]), (101325, 45.2, 3.301))
        kind, q = studio.launch.parse_line(f"Q,5000,{(1 << 14) - 1 - (1 << 12)},{(1 << 14) - 1},30,15,96,2000,0,250,1,0,0,0,0,7,3")
        self.assertEqual(kind, "sensor_checks")
        self.assertFalse(q["checks"]["sd"])
        self.assertTrue(q["boot"]["sd"] and q["checks"]["supply"])
        self.assertEqual((q["ids"], q["session"], q["overruns"]), ([30, 15, 96], 7, 3))

    def test_flash_accepts_the_sensor_test_image(self):
        self.assertIn("sensortest", studio.launch.IMAGES)
        self.assertIn("sensortest", studio.launch.image_info(ROOT)["images"])


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
        # exact listing from a real EasyTVC v0.1 (STM32F405 ROM bootloader pads the name with two spaces)
        real = ('Found DFU: [0483:df11] ver=2200, devnum=1, cfg=1, intf=0, path="1-1", alt=1, name="@Option Bytes  /0x1FFFC000/01*016 e", serial="305B344F3034"\n'
                'Found DFU: [0483:df11] ver=2200, devnum=1, cfg=1, intf=0, path="1-1", alt=0, name="@Internal Flash  /0x08000000/04*016Kg,01*064Kg,07*128Kg", serial="305B344F3034"\n')
        found = dfu.discover(real)
        self.assertEqual([d["serial"] for d in found], ["305B344F3034"])
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
