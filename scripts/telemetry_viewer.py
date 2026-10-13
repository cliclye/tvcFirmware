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
