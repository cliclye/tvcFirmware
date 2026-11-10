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
        cls.vehicle = studio.launch.upgrade_vehicle({})   # the default TVCRocketV3 model (978 g, F15)
        cls.servo_config = {**cls.config, "CHUTE_CHANNEL": 3, "PYRO_CHUTE_CHANNEL": 0}   # recovery by release servo

    def test_configured_rocket_flies(self):
        result = studio.launch.run_sim3d(self.native, self.servo_config, self.vehicle, studio.launch.sim_defaults())
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
        self.assertAlmostEqual(m["stability_cal"], (100 - m["cg_wet_mm"]) / v["diameter_mm"])

    def test_linkage_matches_c(self):
        m = studio.launch.rocket_model({})   # TVCRocketV3: 9 mm horn / 42 mm lever, 12 deg stop
        ratio = m["linkage_ratio"]
        self.assertAlmostEqual(studio.launch.gimbal_angle_deg(250, m), math.degrees(math.asin(ratio * math.sin(math.radians(22.5)))), places=6)
        self.assertLess(math.degrees(math.asin(ratio)), 13.0)   # the horn alone can't swing the gimbal far past its stop
        self.assertEqual(studio.launch.gimbal_angle_deg(1000, m), m["stop_deg"])   # 90 deg of horn would be 12.4 deg: clamped

    def test_batch(self):
        result = studio.launch.run_sim3d_batch(self.native, self.servo_config, self.vehicle, studio.launch.sim_defaults(), 10)
        self.assertEqual(result["count"], 10)
        self.assertGreaterEqual(result["success"], 8)

    def test_tuning_lab_engine(self):
        """Gains judged on the real rocket: evaluation, launch margin and the auto-tune job."""
        import time as _t
        launch, sim = studio.launch, studio.launch.sim_defaults()
        # 1.5 g launch threshold: at 1.8 g this rocket misses the odd launch in the "all" case (the real finding)
        base = {**self.servo_config, "LAUNCH_ACCEL_G": 1.5}
        good = launch.tune_evaluate(self.native, base, self.vehicle, sim, flights=10, stress_flights=6)
        self.assertEqual([x["key"] for x in good["stress"]], ["slow", "slop", "weak", "heavy", "all"])
        self.assertTrue(good["held_all"])
        self.assertGreater(good["nominal"]["tvc_on_t"], good["nominal"]["launch_t"])
        self.assertTrue(all(len(p) == 5 for p in good["trace"]) and len(good["trace"]) > 50)
        json.dumps(good)
        weak = launch.tune_evaluate(self.native, {**base, "KP": 0.3, "KI": 0.0, "KD": 0.02}, self.vehicle, sim,
                                    flights=10, stress_flights=6)
        self.assertGreater(weak["score"], good["score"] * 3)                  # far too soft: flights are lost
        self.assertFalse(weak["held_all"])
        # a lower launch threshold leaves more thrust in hand before a launch is missed
        low = launch.launch_margin(self.native, {**self.servo_config, "LAUNCH_ACCEL_G": 1.4}, self.vehicle, sim, seeds=4)
        high = launch.launch_margin(self.native, {**self.servo_config, "LAUNCH_ACCEL_G": 2.4}, self.vehicle, sim, seeds=4)
        self.assertGreater(low["percent"], high["percent"])
        job = launch.TuneJob(self.native, self.servo_config, self.vehicle, sim)
        _t.sleep(0.5)
        self.assertTrue(job.status()["running"])
        job.cancel()
        job.thread.join(20)
        self.assertTrue(job.status()["cancelled"] and not job.status()["running"])

    def test_recovery_is_flown_as_the_firmware_would(self):
        """Release servo, a live charge (also at a deploy height), and a charge behind locked outputs."""
        launch, sim = studio.launch, studio.launch.sim_defaults()
        charge = {**self.config, "CHUTE_CHANNEL": 0, "PYRO_CHUTE_CHANNEL": 1, "PYRO_DEPLOY_ALT_M": 0.0}
        with patch.object(launch, "pyro_live_in_firmware", lambda: False):
            locked = launch.run_sim3d(self.native, charge, self.vehicle, sim)
            self.assertLess(locked["summary"]["chute_t"], 0)                 # nothing deploys ...
            self.assertGreater(locked["summary"]["charge_t"], 0)             # ... though the charge was due
            self.assertTrue(any(v["level"] == "bad" and "locked" in v["text"] for v in locked["verdict"]))
            self.assertEqual(launch.run_sim3d_batch(self.native, charge, self.vehicle, sim, 5)["reasons"], {"no parachute": 5})
            live = launch.run_sim3d(self.native, charge, self.vehicle, {**sim, "pyro_unlocked": 1})
            self.assertGreater(live["summary"]["chute_t"], 0)
            self.assertTrue(live["recovery"]["assumed_unlocked"])
            low = launch.run_sim3d(self.native, {**charge, "PYRO_DEPLOY_ALT_M": 20.0, "PYRO_BACKUP_MS": 20000},
                                   self.vehicle, {**sim, "pyro_unlocked": 1})["summary"]
            self.assertGreater(low["chute_t"], live["summary"]["chute_t"] + 1.0)   # waits to come down to 20 m
        # a motor with a measured curve always flies it, whatever the saved choice says
        at = launch.SIM3D_PARAMS.index("thrust_profile")
        self.assertEqual(launch.sim_params(self.servo_config, self.vehicle, {**sim, "thrust_profile": 1})[at], 2.0)


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


class GroundLinkTests(unittest.TestCase):
    """The E1 ground link: frames, loss, the in-flight block, recording/replay, and a simulated
    DroneBridge link (UDP with loss and corruption) driven by the 3D sim's firmware-in-the-loop flight."""

    @classmethod
    def setUpClass(cls):
        import groundlink
        sys.path.insert(0, str(ROOT / "scripts"))
        import fake_link
        cls.g, cls.fake = groundlink, fake_link
        cls.native = studio.NativeController()

    def test_settings_pyro_test_and_beep_over_the_link(self):
        import socket, time as _t
        launch = studio.launch
        config = launch.read_config(ROOT)
        board = self.fake.FakeBoard(dict(config), b"")
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind(("127.0.0.1", 0))
        stop = threading.Event()
        mon = self.g.Monitor()
        mon.connect("udp", host="127.0.0.1", port=sock.getsockname()[1], listen_port=0)
        # a clean link: PYROFIRE never retries (a lost reply must not fire twice), so loss would only
        # make this test flaky; lossy-link behaviour is covered by test_simulated_link_end_to_end
        thread = threading.Thread(target=self._board_thread, args=(board, [], 0.0, 0.0, sock, stop), daemon=True)
        thread.start()
        try:
            deadline = _t.monotonic() + 5
            while not mon.board_idle() and _t.monotonic() < deadline:
                _t.sleep(0.05)
            values = {n: config[n] for n, _ in self.g.TUNABLE_FIELDS}
            values.update(SERVO_A_CENTER_US=config["SERVO_A_CENTER_US"] - 7, KP=9.85, PYRO_DEPLOY_ALT_M=20.5, PYRO_BACKUP_MS=9000)
            stored = mon.send_settings(values)                     # sent, stored, echoed back, compared
            self.assertEqual(stored["SERVO_A_CENTER_US"], config["SERVO_A_CENTER_US"] - 7)
            self.assertAlmostEqual(stored["KP"], 9.85)
            board_cfg = mon.read_config()
            self.assertEqual(board_cfg["RADIO_SETTINGS"], 1)
            self.assertAlmostEqual(board_cfg["PYRO_DEPLOY_ALT_M"], 20.5)
            mon.reset_settings()
            self.assertEqual(mon.read_config()["RADIO_SETTINGS"], 0)
            armed = mon.pyro_arm(3)
            self.assertTrue(armed["locked"] and 1000 <= armed["code"] <= 9999)
            with self.assertRaises(ValueError):
                mon.pyro_fire(3, armed["code"] + 1 if armed["code"] < 9999 else 1000, 200)   # wrong code
            armed = mon.pyro_arm(3)
            self.assertEqual(mon.pyro_fire(3, armed["code"], 200)["pulse_ms"], 200)
            with self.assertRaises(ValueError):
                mon.pyro_arm(2)                                    # P2: never
            mon.beep()
            self.assertEqual(board.beeps, 1)
            # out of range values are refused before anything is sent
            with self.assertRaises(ValueError):
                self.g.Monitor().send_settings({n: 0 for n, _ in self.g.TUNABLE_FIELDS[:3]})
            # not IDLE: all of these are blocked in Studio (and the board ignores them)
            mon.latest["tlm"], mon.updated["tlm"] = {"state": "ARMED"}, _t.monotonic()
            for call in (lambda: mon.send_settings(values), mon.reset_settings, lambda: mon.pyro_arm(1), mon.beep):
                with self.assertRaises(ValueError):
                    call()
        finally:
            stop.set(); thread.join()
            mon.disconnect()
            sock.close()

    def test_bluetooth_relay_status_never_mixes_into_frames(self):
        """A fake BleRelay.swift: board frames split into 20-byte notifications, with the relay's own
        status datagrams landing between the pieces. Every frame must still decode, none counted bad."""
        import socket, time as _t
        relay = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        relay.bind(("127.0.0.1", 0))
        mon = self.g.Monitor()
        mon.connect("ble", relay_port=relay.getsockname()[1])
        try:
            relay.settimeout(3)
            _, client = relay.recvfrom(4096)            # Studio's keep-alive PING wakes the relay
            board = self.fake.FakeBoard({}, b"")
            for i in range(20):
                frame = board.frame("SYS", f"{i},0,0,0,0,1234,3300,0,0,0,1,7,10,0,0,8000,900,0,0,0,0,0,0")
                for k in range(0, len(frame), 20):
                    relay.sendto(frame[k:k + 20], client)
                    if k == 20:
                        relay.sendto(b"#BLE state=connected name=DroneBridge rssi=-61\n", client)
            deadline = _t.monotonic() + 3
            while mon.stats.received < 20 and _t.monotonic() < deadline:
                _t.sleep(0.02)
            status = mon.status(0)
            self.assertEqual((mon.stats.received, mon.stats.bad), (20, 0))
            self.assertEqual(status["ble"], {"state": "connected", "name": "DroneBridge", "rssi": -61})
            self.assertEqual(status["transport"], "ble")
            self.assertFalse(any(line.startswith("#BLE") for line in status["lines"]))
        finally:
            mon.disconnect()
            relay.close()
        # with no relay at all (Studio not running as the app) the page is told so
        mon = self.g.Monitor()
        mon.connect("ble", relay_port=1)
        try:
            mon.transport.opened -= 5
            self.assertEqual(mon.status()["ble"]["state"], "no_relay")
        finally:
            mon.disconnect()

    def test_recovery_outputs_config_and_formats(self):
        launch = studio.launch
        config = launch.read_config(ROOT)
        for channel in (0, 1, 3):
            launch.validate_config({**config, "PYRO_CHUTE_CHANNEL": channel})
        with self.assertRaisesRegex(ValueError, "P2"):              # the SWD pin
            launch.validate_config({**config, "PYRO_CHUTE_CHANNEL": 2})
        with self.assertRaises(ValueError):
            launch.validate_config({**config, "PYRO_FIRE_MS": 50})
        with tempfile.TemporaryDirectory() as tmp:                   # the new #defines round-trip
            project = Path(tmp)
            (project / "Core" / "Inc").mkdir(parents=True)
            launch.config_path(project).write_text(launch.config_path(ROOT).read_text())
            saved = launch.write_config(project, {**config, "PYRO_CHUTE_CHANNEL": 3, "PYRO_CHUTE_DELAY_MS": 1500, "PYRO_FIRE_MS": 800})
            self.assertEqual(launch.read_config(project), saved)
        self.assertFalse(launch.pyro_outputs_compiled_in(ROOT))     # the master lock stays as it is
        # the board's CFG reply: the lock is reported but isn't a setting to compare
        values = [0] * len(self.g.CFG_FIELDS)
        values[[n for n, _ in self.g.CFG_FIELDS].index("NOSE_AXIS")] = 1
        board = self.g.parse_cfg(",".join(map(str, values)))
        self.assertNotIn("PYRO_OUTPUTS_COMPILED_IN", [r["name"] for r in launch.compare_config(board, config)])
        self.assertEqual(len(self.g.parse_cfg(",".join(map(str, values[:self.g.CFG_FIELDS_V1])))), 32)   # firmware 0.3
        # TLM: 24 fields with the deploy countdown, 23 from firmware 0.3
        base = "1,3,2000,10000,0,0,0,0,0,0,0,0,1000,500,1000,10,20,0,0,1500,1500,1500,5"
        self.assertEqual(self.g.parse_message("TLM", base + ",3700")[1]["deploy_eta_s"], 3.7)
        self.assertIsNone(self.g.parse_message("TLM", base + ",-1")[1]["deploy_eta_s"])
        self.assertIsNone(self.g.parse_message("TLM", base)[1]["deploy_eta_s"])
        sys_frame = "1,3,0,0,0,1234,3300,0,0,0,1,7,10,0,0,8000,900,0,0,0,0,0,0"     # image 3: the pyro bench image
        self.assertEqual(self.g.parse_message("SYS", sys_frame)[1]["image"], "pyrobench")
        self.assertIn("pyrobench", studio.launch.IMAGES)

    def test_history_since_and_ui_state_merge(self):
        import time as _t
        mon = self.g.Monitor()
        for i in range(mon.history.maxlen + 50):      # overflow the history: the oldest 50 drop out
            mon._store(("tlm", {"ms": i, "state": "IDLE"}), _t.monotonic())
        status = mon.status(0)
        self.assertEqual(status["history"][0]["ms"], 50)
        self.assertEqual(status["history_next"], mon.history.maxlen + 50)
        mon._store(("tlm", {"ms": 99999, "state": "IDLE"}), _t.monotonic())
        self.assertEqual([h["ms"] for h in mon.status(status["history_next"])["history"]], [99999])   # nothing skipped
        with tempfile.TemporaryDirectory() as tmp:
            with patch.object(studio.launch, "state_path", lambda: Path(tmp) / "state.json"):
                studio.launch.save_state({"ui": {"theme": "dark"}})
                state = studio.launch.save_state({"ui": {"live": {"kind": "udp"}}})
                self.assertEqual(state["ui"], {"theme": "dark", "live": {"kind": "udp"}})   # one doesn't wipe the other

    def test_frames_round_trip_reject_corruption_and_resync(self):
        g = self.g
        frame = g.encode_frame(123, "TLM", "1,2,3")
        self.assertEqual(g.decode_frame(frame), (123, "TLM", "1,2,3"))
        self.assertIsNone(g.decode_frame(frame[:6] + b"9" + frame[7:]))            # one byte changed
        self.assertIsNone(g.decode_frame(frame.replace(b"*", b"#")))
        self.assertIsNone(g.decode_frame(b"E1 1 TLM a b *00000000\n"))
        with self.assertRaises(ValueError):
            g.encode_frame(1, "TLM", "has space")
        # the line splitter recovers after junk, a cut frame and a corrupted frame
        mon = g.Monitor()
        mon.transport = object()                    # marks it connected; nothing is sent
        good = [g.encode_frame(i, "EVT", f"{i},E{i}") for i in range(3)]
        stream = b"\x00\xffgarbage\n" + good[0][:20] + b"\n" + good[1].replace(b"E1", b"E2") + good[2]
        for line in stream.split(b"\n"):
            mon._line(line + b"\n", 1.0)
        self.assertEqual([e["text"] for e in mon.events], ["E2"])
        self.assertEqual(mon.stats.bad, 1)          # the cut E1 frame; the rest wasn't E1 at all

    def test_c_encoder_matches_python_decoder(self):
        import ctypes
        lib = self.native.lib

        class Tlm(ctypes.Structure):
            _fields_ = [("ms", ctypes.c_uint32), ("state", ctypes.c_uint8), ("since_launch_ms", ctypes.c_int32),
                        ("q", ctypes.c_float * 4), ("gyro_rad_s", ctypes.c_float * 3), ("accel_m_s2", ctypes.c_float * 3),
                        ("altitude_m", ctypes.c_float), ("vertical_speed_m_s", ctypes.c_float),
                        ("tilt_rad", ctypes.c_float * 2), ("command", ctypes.c_float * 2),
                        ("pulse_us", ctypes.c_uint16 * 3), ("flags", ctypes.c_uint16), ("deploy_eta_ms", ctypes.c_int32),
                        ("pressure_pa", ctypes.c_uint32), ("baro_temp_cdeg", ctypes.c_int16)]
        lib.EasyTVC_LinkFormatTlm.restype = ctypes.c_size_t
        lib.EasyTVC_LinkEncode.restype = ctypes.c_size_t
        rows, _ = self.fake.sim_telemetry(self.native)
        for _, t, _ in rows[::7]:
            c = Tlm(t["ms"], t["state"], t["since_launch_ms"], (ctypes.c_float * 4)(*t["q"]),
                    (ctypes.c_float * 3)(*t["gyro_rad_s"]), (ctypes.c_float * 3)(*t["accel_m_s2"]),
                    t["altitude_m"], t["vertical_speed_m_s"], (ctypes.c_float * 2)(*t["tilt_rad"]),
                    (ctypes.c_float * 2)(*t["command"]), (ctypes.c_uint16 * 3)(*t["pulse_us"]), t["flags"],
                    t["deploy_eta_ms"], t["pressure_pa"], t["baro_temp_cdeg"])
            payload = ctypes.create_string_buffer(256)
            n = lib.EasyTVC_LinkFormatTlm(ctypes.byref(c), payload, 256)
            frame = ctypes.create_string_buffer(256)
            m = lib.EasyTVC_LinkEncode(frame, 256, ctypes.c_uint32(77), b"TLM", payload)
            decoded = self.g.decode_frame(frame.raw[:m])
            self.assertEqual(decoded[:2], (77, "TLM"))
            kind, d = self.g.parse_message("TLM", decoded[2])
            self.assertEqual(kind, "tlm")
            self.assertAlmostEqual(d["alt_m"], t["altitude_m"], delta=0.01)
            # the Python formatter (used by the fake board) writes the same numbers as the C one,
            # give or take float32 rounding on the last digit
            py = [int(x) for x in self.fake.format_tlm(t).split(",")]
            cc = [int(x) for x in payload.value[:n].decode().split(",")]
            self.assertEqual(len(py), len(cc))
            self.assertTrue(all(abs(a - b) <= 1 for a, b in zip(py, cc)), (py, cc))
            self.assertEqual(d["deploy_eta_s"], None if t["deploy_eta_ms"] < 0 else t["deploy_eta_ms"] / 1000)
            self.assertEqual((d["pressure_pa"], d["baro_temp_c"]), (t["pressure_pa"], 21.5))

    def test_packet_loss_from_sequence_gaps(self):
        stats = self.g.LinkStats()
        for seq in [0, 1, 2, 5, 6, 10]:            # 3, 4, 7, 8, 9 lost
            stats.frame(seq, 1.0)
        self.assertEqual((stats.received, stats.lost), (6, 5))
        self.assertAlmostEqual(stats.loss_percent(), 100 * 5 / 11)
        stats.frame(0, 2.0)                         # board rebooted: no false loss
        stats.frame(1, 2.0)
        self.assertEqual(stats.lost, 5)
        stats.frame(0xffffffff, 3.0)                 # wrap-around
        stats.frame(1, 3.1)
        self.assertEqual(stats.lost, 6)
        self.assertTrue(stats.summary(10.0)["stale"])

    def test_commands_blocked_unless_idle(self):
        import time as _t
        mon = self.g.Monitor()
        mon.transport = object()
        for state in ("ARMED", "BOOST", "COAST", "DESCENT", "LANDED", "FAULT"):
            mon.latest["sys"], mon.updated["sys"] = {"state": state}, _t.monotonic()
            for command in ("STATUS", "CFG", "LOGINFO", "LOGREAD", "SERVO", "SWEEP", "CENTER", "TRIM"):
                with self.assertRaises(ValueError, msg=state):
                    mon.request(command)
            for command in mon.GIMBAL_COMMANDS:          # the gimbal test: blocked before anything is sent
                with self.assertRaises(ValueError, msg=state):
                    mon.gimbal(command)
        mon.latest.clear(); mon.updated.clear()
        with self.assertRaises(ValueError):          # no recent state at all: also blocked
            mon.request("CFG")
        mon.latest["sys"], mon.updated["sys"] = {"state": "IDLE"}, _t.monotonic() - 10
        with self.assertRaises(ValueError):          # stale IDLE does not count
            mon.request("CFG")
        mon.latest["sys"], mon.updated["sys"] = {"state": "IDLE"}, _t.monotonic() - 1
        mon.latest["tlm"], mon.updated["tlm"] = {"state": "BOOST"}, _t.monotonic()
        with self.assertRaises(ValueError):          # an older SYS IDLE can't outvote a fresh TLM in flight
            mon.request("CFG")
        # the fake board (like the firmware) ignores the same requests once it isn't IDLE
        board = self.fake.FakeBoard({}, b"")
        board.state = 2
        self.assertIsNone(board.answer(self.g.encode_frame(5, "CFG")))
        self.assertIsNone(board.answer(self.g.encode_frame(7, "SERVO", "500,500")))
        self.assertIsNone(board.answer(self.g.encode_frame(8, "TRIM", "10,10")))
        self.assertIsNotNone(board.answer(self.g.encode_frame(6, "PING")))
        with self.assertRaises(ValueError):          # out-of-range values never leave Studio
            self.g.Monitor().gimbal("SERVO", 1001, 0)
        with self.assertRaises(ValueError):
            self.g.Monitor().gimbal("TRIM", 0, -101)
        with self.assertRaises(ValueError):
            self.g.Monitor().gimbal("CHUTE")

    def _board_thread(self, board, frames, loss, corrupt, sock, stop, rng_seed=3):
        import random, select as _select
        rng, target, i, beacon = random.Random(rng_seed), None, 0, 0.0
        import time as _t
        while not stop.is_set():
            if target and not frames and _t.monotonic() - beacon > 0.3:   # on the pad: SYS says IDLE
                beacon = _t.monotonic()
                sock.sendto(board.frame("SYS", "1,0,0,0,0,1234,3300,0,0,0,1,7,10,0,0,8000,900,0,0,0,0,0,0"), target)
            if _select.select([sock], [], [], 0.002)[0]:
                data, sender = sock.recvfrom(4096)
                target = target or sender
                for line in data.split(b"\n"):
                    reply = board.answer(line + b"\n") if line else None
                    if reply and rng.random() >= loss:
                        sock.sendto(reply, sender)
            if target and i < len(frames):
                state, frame = frames[i]
                board.state = state
                i += 1
                if rng.random() < loss:
                    continue
                if rng.random() < corrupt:
                    frame = frame[:8] + b"?" + frame[9:]
                sock.sendto(frame, target)
            elif i >= len(frames):
                stop.wait(0.005)

    def test_simulated_link_end_to_end(self):
        import socket, time as _t
        g, launch = self.g, studio.launch
        rows, config = self.fake.sim_telemetry(self.native)
        with tempfile.TemporaryDirectory() as tmp:
            folder = Path(tmp)
            # a sensor-test session for the log download, from the firmware's own record layout
            exe, out = folder / "fx", folder / "s.bin"
            subprocess.run(["cc", "-std=c11", "-I", str(ROOT / "Core" / "Inc"), str(ROOT / "tests" / "sensor_test_fixture.c"),
                            "-o", str(exe)], check=True)
            subprocess.run([str(exe), str(out)], check=True)
            session_bytes = out.read_bytes().rstrip(b"\0")
            board = self.fake.FakeBoard(config, session_bytes)
            frames, last = [], None
            for _, t, state in rows:
                if state != last:
                    frames.append((state, board.frame("EVT", f"{t['ms']},{self.fake.EVENTS[state]}")))
                    frames.append((state, board.frame("EVT", f"{t['ms']},{self.fake.EVENTS[state]}")))   # sent twice
                    last = state
                frames.append((state, board.frame("TLM", self.fake.format_tlm(t))))
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.bind(("127.0.0.1", 0))
            stop = threading.Event()
            with patch.object(launch, "flights_dir", lambda create=True: folder):
                mon = launch.Monitor()
                mon.connect("udp", host="127.0.0.1", port=sock.getsockname()[1], listen_port=0)
                # on the pad first: config read and a log download work (board IDLE, 10 % loss)
                board_thread = threading.Thread(target=self._board_thread, args=(board, [], 0.10, 0.0, sock, stop), daemon=True)
                board_thread.start()
                deadline = _t.monotonic() + 5
                while mon.rtt_ms is None and _t.monotonic() < deadline:
                    _t.sleep(0.05)
                while not mon.board_idle() and _t.monotonic() < deadline:
                    _t.sleep(0.05)
                cfg = mon.read_config()
                self.assertTrue(all(r["same"] for r in launch.compare_config(cfg, config)))
                # the TVC gimbal test on the pad, over the same lossy link
                self.assertEqual(mon.gimbal("TRIM", -12, 30), {"mode": "off", "trim_us": [-12, 30]})
                self.assertEqual(mon.gimbal("SWEEP")["mode"], "sweep")
                self.assertEqual(mon.gimbal("CENTER")["mode"], "off")
                mon.start_download(None, lambda raw, n: launch._save_session(raw, n)[0].name)   # latest = 7
                while mon.download_status()["running"]:
                    _t.sleep(0.05)
                job = mon.download_status()
                self.assertTrue(job["done"], job)
                print(f"\n  log download over a 10 %-loss link: {job['bytes']} B at {job['speed_kb_s']} kB/s")
                self.assertEqual((folder / job["saved"]).read_bytes(), session_bytes)
                stop.set(); board_thread.join()
                # then the flight: 10 % loss and 2 % corrupted frames
                stop = threading.Event()
                board_thread = threading.Thread(target=self._board_thread, args=(board, frames, 0.10, 0.02, sock, stop), daemon=True)
                board_thread.start()
                deadline = _t.monotonic() + 20
                while mon.status()["link"]["frames"] < 0.8 * len(frames) and _t.monotonic() < deadline:
                    _t.sleep(0.05)
                _t.sleep(0.3)
                status = mon.status(0)
                json.dumps(status)                    # the UI gets plain JSON
                link = status["link"]
                self.assertGreater(link["bad"], 0)
                self.assertTrue(5 < link["loss_pct"] < 20, link)
                self.assertEqual(len([e for e in status["events"] if e["text"] == "APOGEE"]), 1)   # de-duplicated
                self.assertGreater(len(status["history"]), 200)
                self.assertFalse(status["idle"])      # landed, not IDLE: commands are now blocked
                with self.assertRaises(ValueError):
                    mon.request("CFG")
                recording = mon.recorder.path
                stop.set(); board_thread.join()
                mon.disconnect()
                sock.close()
                # the recording opens like an SD session, with events and a 3D replay
                data = recording.read_bytes()
                opened = launch.summarize_session(ROOT, data, "test")
                self.assertEqual(opened["kind"], "telemetry")
                self.assertTrue(opened["summary"]["flew"])
                self.assertEqual([e["event"] for e in opened["events"]].count("APOGEE"), 1)   # state change + EVT: shown once
                replay = launch.replay_session(ROOT, opened["data_id"], config, launch.upgrade_vehicle({}))
                self.assertGreater(replay["summary"]["apogee_m"], 10)
                self.assertIn(recording.name, [s["name"] for s in launch.saved_sessions(ROOT)["sessions"]])


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
