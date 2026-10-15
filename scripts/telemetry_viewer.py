#!/usr/bin/env python3
"""
EasyTVC flight log viewer.

Reads one session copied by scripts/read-sd-data.sh and plots it, or exports CSV.

    python3 scripts/telemetry_viewer.py session_3.bin
    python3 scripts/telemetry_viewer.py session_3.bin --csv flight.csv     (no matplotlib needed)

Format: Core/Inc/data_logger.h. Sector 0 of the session is a header ("ETVCLOG2", session number,
record size, settings text); then 32-byte records, 16 per sector, body frame.
"""

import argparse
import csv
import struct
import sys

SECTOR = 512
RECORD_FORMAT = '<I3h3h2h2HhhBBBB'
RECORD_SIZE = struct.calcsize(RECORD_FORMAT)
MARKER = 0xA5
STATES = ['IDLE', 'ARMED', 'BOOST', 'COAST', 'APOGEE', 'DESCENT', 'LANDED', 'FAULT']
FLAGS = ['imu_ok', 'baro_ok', 'gyro_cal_ok', 'accel_sat', 'gyro_sat', 'tvc_active',
         'ctrl_fault', 'chute_open']
assert RECORD_SIZE == 32

# ---- sensor-test image (Core/Inc/sensor_test.h) ----
SENSOR_HEADER = 'EasyTVC mode=sensortest'
SENSOR_TAIL = '<BBBB'                      # session, type, seq, marker at offset 28
SENSOR_IMU = '<I3h3hhHH6x'
SENSOR_BARO = '<IIiIIIHH'
SENSOR_ADC = '<IHHHHHhHH8x'
SENSOR_STATUS = '<IIIBBBxIIHH'
ACCEL_G_PER_LSB = 24 / 32768
GYRO_DPS_PER_LSB = 2000 / 32768
SENSOR_CHECKS = [
    ('accel_id', 'IMU accelerometer answers (chip ID 0x1E)'),
    ('gyro_id', 'IMU gyroscope answers (chip ID 0x0F)'),
    ('baro_id', 'Barometer answers (chip ID 0x60)'),
    ('accel_1g', 'Accelerometer reads 1 g at rest'),
    ('gyro_still', 'Gyroscope quiet at rest (bias < 5 deg/s, noise < 3 deg/s)'),
    ('imu_alive', 'IMU values change (not stuck)'),
    ('imu_rate', 'IMU read every time (>= 99 %)'),
    ('pressure', 'Pressure 60-110 kPa'),
    ('baro_temp', 'Barometer temperature -20..60 C'),
    ('humidity', 'Humidity 0-100 % and changing'),
    ('baro_rate', 'Barometer read every time (>= 95 %)'),
    ('accel_temp', 'IMU die temperature -20..70 C'),
    ('sd', 'SD card recording, nothing lost'),
    ('supply', '3.3 V rail 3.0-3.6 V'),
]
for _fmt in (SENSOR_IMU, SENSOR_BARO, SENSOR_ADC, SENSOR_STATUS):
    assert struct.calcsize(_fmt) == 28, _fmt


def is_sensor_session(data):
    return data[:8] == b'ETVCLOG2' and data[16:16 + len(SENSOR_HEADER)] == SENSOR_HEADER.encode()


def parse_sensor_session(data):
    """Sensor-test session: returns dict(session, header, imu, baro, adc, status, gaps)."""
    if len(data) < SECTOR or data[:8] != b'ETVCLOG2':
        raise ValueError("not an EasyTVC session (expected an ETVCLOG2 header)")
    session, record_size = struct.unpack_from('<II', data, 8)
    if record_size != RECORD_SIZE:
        raise ValueError(f"record size {record_size}, this reader expects {RECORD_SIZE}")
    header_text = data[16:SECTOR].split(b'\0', 1)[0].decode('ascii', 'replace')
    out = {'session': session, 'header': header_text, 'imu': [], 'baro': [], 'adc': [], 'status': [],
           'gaps': 0, 'records': 0}
    last_t, last_seq = -1, None
    for offset in range(SECTOR, len(data) - RECORD_SIZE + 1, RECORD_SIZE):
        tag, kind, seq, marker = struct.unpack_from(SENSOR_TAIL, data, offset + 28)
        t_us = struct.unpack_from('<I', data, offset)[0]
        if marker != MARKER or tag != (session & 0xFF) or t_us < last_t or kind not in (1, 2, 3, 4):
            break
        if last_seq is not None and seq != (last_seq + 1) & 0xFF:
            out['gaps'] += 1
        last_t, last_seq = t_us, seq
        out['records'] += 1
        t = t_us / 1e6
        if kind == 1:
            _, ax, ay, az, gx, gy, gz, temp, read_us, errors = struct.unpack_from(SENSOR_IMU, data, offset)
            out['imu'].append({'t_s': t, 'accel_x_g': ax * ACCEL_G_PER_LSB, 'accel_y_g': ay * ACCEL_G_PER_LSB,
                               'accel_z_g': az * ACCEL_G_PER_LSB, 'gyro_x_dps': gx * GYRO_DPS_PER_LSB,
                               'gyro_y_dps': gy * GYRO_DPS_PER_LSB, 'gyro_z_dps': gz * GYRO_DPS_PER_LSB,
                               'raw_ax': ax, 'raw_ay': ay, 'raw_az': az, 'raw_gx': gx, 'raw_gy': gy, 'raw_gz': gz,
                               'imu_temp_c': temp / 100, 'read_us': read_us, 'imu_errors': errors})
        elif kind == 2:
            _, p, temp, hum, adc_p, adc_t, adc_h, errors = struct.unpack_from(SENSOR_BARO, data, offset)
            out['baro'].append({'t_s': t, 'pressure_pa': p, 'baro_temp_c': temp / 100, 'humidity_pct': hum / 1024,
                                'adc_p': adc_p, 'adc_t': adc_t, 'adc_h': adc_h, 'baro_errors': errors})
        elif kind == 3:
            _, batt, vref, mcu, vdda, batt_mv, mcu_t, overruns, max_loop = struct.unpack_from(SENSOR_ADC, data, offset)
            out['adc'].append({'t_s': t, 'batt_raw': batt, 'vref_raw': vref, 'mcu_temp_raw': mcu,
                               'vdda_v': vdda / 1000, 'batt_pin_v': batt_mv / 1000, 'mcu_temp_c': mcu_t / 100,
                               'loop_overruns': overruns, 'max_loop_us': max_loop})
        else:
            _, checks, boot, aid, gid, bid, imu_n, baro_n, dropped, werr = struct.unpack_from(SENSOR_STATUS, data, offset)
            out['status'].append({'t_s': t, 'checks': checks, 'boot_checks': boot, 'accel_id': aid, 'gyro_id': gid,
                                  'baro_id': bid, 'imu_samples': imu_n, 'baro_samples': baro_n,
                                  'sd_dropped': dropped, 'sd_write_errors': werr})
    return out


def _stats(values):
    if not values:
        return None
    n = len(values)
    mean = sum(values) / n
    sd = (sum((v - mean) ** 2 for v in values) / n) ** 0.5
    return {'mean': mean, 'sd': sd, 'min': min(values), 'max': max(values)}


def sensor_report(parsed):
    """Pass/fail per check (power-up and latest), measured rates and per-channel statistics."""
    imu, baro, adc, status = parsed['imu'], parsed['baro'], parsed['adc'], parsed['status']
    last = status[-1] if status else None
    checks = []
    for bit, (key, text) in enumerate(SENSOR_CHECKS):
        checks.append({'key': key, 'text': text,
                       'boot': bool(last and last['boot_checks'] >> bit & 1) if last else None,
                       'now': bool(last and last['checks'] >> bit & 1) if last else None})
    rate = lambda rows: (len(rows) - 1) / (rows[-1]['t_s'] - rows[0]['t_s']) if len(rows) > 1 and rows[-1]['t_s'] > rows[0]['t_s'] else 0
    channels = {}
    for rows, keys in ((imu, ('accel_x_g', 'accel_y_g', 'accel_z_g', 'gyro_x_dps', 'gyro_y_dps', 'gyro_z_dps', 'imu_temp_c', 'read_us')),
                       (baro, ('pressure_pa', 'baro_temp_c', 'humidity_pct')),
                       (adc, ('vdda_v', 'batt_pin_v', 'mcu_temp_c', 'max_loop_us'))):
        for key in keys:
            channels[key] = _stats([r[key] for r in rows])
    duration = max((rows[-1]['t_s'] for rows in (imu, baro, adc, status) if rows), default=0) - \
        min((rows[0]['t_s'] for rows in (imu, baro, adc, status) if rows), default=0)
    passed = bool(last) and all(c['boot'] for c in checks)
    return {'passed': passed, 'checks': checks, 'duration_s': duration,
            'rates_hz': {'imu': rate(imu), 'baro': rate(baro), 'adc': rate(adc)},
            'counts': {'imu': len(imu), 'baro': len(baro), 'adc': len(adc), 'status': len(status),
                       'records': parsed['records'], 'gaps': parsed['gaps']},
            'ids': {k: last[k] for k in ('accel_id', 'gyro_id', 'baro_id')} if last else None,
            'channels': channels,
            'sd': {'dropped': last['sd_dropped'], 'write_errors': last['sd_write_errors']} if last else None}


def sensor_csv(parsed):
    """One merged table, every record in time order (blank where a record type has no value)."""
    rows = [dict(r, record='imu') for r in parsed['imu']] + [dict(r, record='baro') for r in parsed['baro']] + \
           [dict(r, record='adc') for r in parsed['adc']] + [dict(r, record='status') for r in parsed['status']]
    rows.sort(key=lambda r: r['t_s'])
    keys = ['t_s', 'record']
    for group in ('imu', 'baro', 'adc', 'status'):
        if parsed[group]:
            keys += [k for k in parsed[group][0] if k not in keys]
    return keys, rows


def parse_session(data):
    """Parse one session dump. Returns (session, header_text, rows); raises ValueError."""
    if len(data) < SECTOR or data[:8] != b'ETVCLOG2':
        raise ValueError("not an EasyTVC session (expected an ETVCLOG2 header)")
    session, record_size = struct.unpack_from('<II', data, 8)
    if record_size != RECORD_SIZE:
        raise ValueError(f"record size {record_size}, this reader expects {RECORD_SIZE}")
    header_text = data[16:SECTOR].split(b'\0', 1)[0].decode('ascii', 'replace')

    rows = []
    last_ts = -1
    for offset in range(SECTOR, len(data) - RECORD_SIZE + 1, RECORD_SIZE):
        f = struct.unpack_from(RECORD_FORMAT, data, offset)
        ts, ax, ay, az, gx, gy, gz, tx, ty, sa, sb, alt, vs, tag, state, flags, marker = f
        # Stop at the end of this session: an unwritten record, an older session's leftovers,
        # or time going backwards.
        if marker != MARKER or tag != (session & 0xFF) or ts < last_ts:
            break
        last_ts = ts
        row = {
            't_s': ts / 1000.0,
            'state': STATES[state] if state < len(STATES) else str(state),
            'accel_x_g': ax / 1000.0, 'accel_y_g': ay / 1000.0, 'accel_z_g': az / 1000.0,
            'gyro_x_dps': gx / 10.0, 'gyro_y_dps': gy / 10.0, 'gyro_z_dps': gz / 10.0,
            'tilt_x_deg': tx / 100.0, 'tilt_y_deg': ty / 100.0,
            'servo_a_us': sa, 'servo_b_us': sb,
            'baro_alt_m': alt / 10.0, 'vspeed_m_s': vs / 100.0,
        }
        for bit, name in enumerate(FLAGS):
            row[name] = (flags >> bit) & 1
        rows.append(row)
    return session, header_text, rows


def read_session(path):
    with open(path, 'rb') as f:
        data = f.read()
    try:
        return parse_session(data)
    except ValueError as error:
        sys.exit(f"{path}: {error}")


def summarize(session, header_text, rows):
    print(f"Session {session}: {len(rows)} records")
    print(f"  settings: {header_text}")
    if not rows:
        return
    print(f"  duration {rows[-1]['t_s'] - rows[0]['t_s']:.1f} s")
    previous = None
    for r in rows:
        if r['state'] != previous:
            print(f"  {r['t_s']:8.2f} s  {r['state']}")
            previous = r['state']
    opened = next((r['t_s'] for r in rows if r['chute_open']), None)
    if opened is not None:
        print(f"  {opened:8.2f} s  parachute servo opened")
    flight = [r for r in rows if r['state'] in ('BOOST', 'COAST', 'APOGEE', 'DESCENT')]
    if flight:
        print(f"  max baro altitude {max(r['baro_alt_m'] for r in rows):.1f} m")
        boost = [r for r in rows if r['state'] == 'BOOST']
        if boost:
            worst = max(boost, key=lambda r: abs(r['tilt_x_deg']) + abs(r['tilt_y_deg']))
            print(f"  worst tilt in boost: X {worst['tilt_x_deg']:.1f} deg, "
                  f"Y {worst['tilt_y_deg']:.1f} deg at {worst['t_s']:.2f} s")
    if any(r['ctrl_fault'] for r in rows):
        t = next(r['t_s'] for r in rows if r['ctrl_fault'])
        print(f"  WARNING: controller fault at {t:.2f} s (tilt limit or bad sensor data)")


def plot(rows, title):
    import matplotlib.pyplot as plt
    t = [r['t_s'] for r in rows]
    fig, ax = plt.subplots(5, 1, figsize=(12, 12), sharex=True)
    fig.suptitle(title)
    for key, label in (('tilt_x_deg', 'X'), ('tilt_y_deg', 'Y')):
        ax[0].plot(t, [r[key] for r in rows], label=label)
    ax[0].set_ylabel('tilt (deg)')
    for key, label in (('servo_a_us', 'A'), ('servo_b_us', 'B')):
        ax[1].plot(t, [r[key] for r in rows], label=label)
    ax[1].set_ylabel('servo (us)')
    for key, label in (('gyro_x_dps', 'X'), ('gyro_y_dps', 'Y'), ('gyro_z_dps', 'Z')):
        ax[2].plot(t, [r[key] for r in rows], label=label)
    ax[2].set_ylabel('rate (deg/s)')
    for key, label in (('accel_x_g', 'X'), ('accel_y_g', 'Y'), ('accel_z_g', 'Z')):
        ax[3].plot(t, [r[key] for r in rows], label=label)
    ax[3].set_ylabel('accel (g)')
    ax[4].plot(t, [r['baro_alt_m'] for r in rows], label='baro alt (m)')
    ax[4].plot(t, [r['vspeed_m_s'] for r in rows], label='v speed (m/s)')
    ax[4].set_xlabel('time since power-up (s)')
    previous = None
    for r in rows:
        if r['state'] != previous:
            for a in ax:
                a.axvline(r['t_s'], color='grey', lw=0.5, ls=':')
            ax[0].annotate(r['state'], (r['t_s'], 0), fontsize=7, rotation=90)
            previous = r['state']
    for a in ax:
        a.grid(True)
        a.legend(loc='upper right')
    plt.tight_layout()
    plt.show()


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    parser.add_argument('session_file')
    parser.add_argument('--csv', help='write all records to this CSV file instead of plotting')
    args = parser.parse_args()

    with open(args.session_file, 'rb') as f:
        data = f.read()
    if is_sensor_session(data):
        parsed = parse_sensor_session(data)
        report = sensor_report(parsed)
        print(f"Sensor test, session {parsed['session']}: {report['duration_s']:.1f} s, "
              f"IMU {report['rates_hz']['imu']:.0f} Hz, baro {report['rates_hz']['baro']:.0f} Hz, "
              f"{report['counts']['gaps']} gaps")
        for c in report['checks']:
            print(f"  {'PASS' if c['boot'] else 'FAIL'}  {c['text']}")
        if args.csv:
            keys, rows = sensor_csv(parsed)
            with open(args.csv, 'w', newline='') as f:
                writer = csv.DictWriter(f, fieldnames=keys)
                writer.writeheader()
                writer.writerows(rows)
            print(f"wrote {args.csv}")
        return
    session, header_text, rows = read_session(args.session_file)
    summarize(session, header_text, rows)
    if not rows:
        sys.exit(1)
    if args.csv:
        with open(args.csv, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
            writer.writeheader()
            writer.writerows(rows)
        print(f"wrote {args.csv}")
    else:
        try:
            plot(rows, f"EasyTVC session {session}")
        except ImportError:
            sys.exit("matplotlib is not installed: pip3 install matplotlib, or use --csv")


if __name__ == '__main__':
    main()
