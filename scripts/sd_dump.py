#!/usr/bin/env python3
"""
Copy EasyTVC logging sessions off a microSD card. Read-only: the card is never written.

    sudo python3 scripts/sd_dump.py /dev/rdisk4 OUT_DIR            latest session
    sudo python3 scripts/sd_dump.py /dev/rdisk4 OUT_DIR --session 7
    sudo python3 scripts/sd_dump.py /dev/rdisk4 OUT_DIR --all      every session still on the card

Writes OUT_DIR/session_N.bin (header sector + only the records actually written, so a 10 s sensor
test is ~150 KB, not 8 MB) and OUT_DIR/result.json. Layout: Core/Inc/data_logger.h. Studio runs
this through Terminal, because macOS lets Terminal read a raw card but not other apps.
Standard library only.
"""

import argparse
import json
import os
import struct
import sys

SECTOR = 512
DIR_SECTOR, FIRST_SESSION, SESSION_SECTORS, MAX_SESSIONS = 1024, 2048, 16384, 32
RECORD = 32
MARKER = 0xA5
CHUNK_SECTORS = 128          # 64 KB per read


def read_sectors(fd, sector, count):
    os.lseek(fd, sector * SECTOR, os.SEEK_SET)
    data = b''
    while len(data) < count * SECTOR:
        part = os.read(fd, count * SECTOR - len(data))
        if not part:
            break
        data += part
    return data


def dump_session(fd, number):
    """Header sector + records up to the first one that is not this session's. None if absent."""
    start = FIRST_SESSION + (number % MAX_SESSIONS) * SESSION_SECTORS
    header = read_sectors(fd, start, 1)
    if len(header) < SECTOR or header[:8] != b'ETVCLOG2' or struct.unpack_from('<I', header, 8)[0] != number:
        return None   # never written, or overwritten by a newer session
    tag = number & 0xFF
    out = bytearray(header)
    sector = start + 1
    end = start + SESSION_SECTORS
    while sector < end:
        count = min(CHUNK_SECTORS, end - sector)
        chunk = read_sectors(fd, sector, count)
        for offset in range(0, len(chunk) - RECORD + 1, RECORD):
            if chunk[offset + 31] != MARKER or chunk[offset + 28] != tag:
                out += chunk[:offset]
                return bytes(out)
        out += chunk
        sector += count
    return bytes(out)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    parser.add_argument('device')
    parser.add_argument('out_dir')
    group = parser.add_mutually_exclusive_group()
    group.add_argument('--session', type=int)
    group.add_argument('--all', action='store_true')
    args = parser.parse_args()

    result = {'ok': False, 'sessions': [], 'error': None}
    try:
        fd = os.open(args.device, os.O_RDONLY)
        try:
            directory = read_sectors(fd, DIR_SECTOR, 1)
            if directory[:8] != b'ETVCDIR1':
                result['error'] = 'nodir'
            else:
                next_session = struct.unpack_from('<I', directory, 8)[0]
                result['next'] = next_session
                available = list(range(max(0, next_session - MAX_SESSIONS), next_session))
                wanted = available if args.all else [args.session if args.session is not None else next_session - 1]
                for number in wanted:
                    if number < 0:
                        continue
                    data = dump_session(fd, number)
                    if data is None:
                        continue
                    path = os.path.join(args.out_dir, f'session_{number}.bin')
                    with open(path, 'wb') as f:
                        f.write(data)
                    os.chmod(path, 0o644)
                    result['sessions'].append({'session': number, 'file': path, 'bytes': len(data)})
                result['ok'] = True
        finally:
            os.close(fd)
    except OSError as error:
        result['error'] = f'{error.strerror or error} ({args.device})'
    path = os.path.join(args.out_dir, 'result.json')
    with open(path, 'w') as f:
        json.dump(result, f)
    os.chmod(path, 0o644)
    return 0 if result['ok'] else 1


if __name__ == '__main__':
    sys.exit(main())
