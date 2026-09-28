#!/usr/bin/env python3
"""Inspect, upload and independently read back an STM32F405 image through USB DFU.

No hardware is modified without the explicit `flash` subcommand and matching SHA-256.
This utility verifies bytes, not whether an image is appropriate or safe for a board.
"""
import argparse
import hashlib
from pathlib import Path
import re
import shutil
import struct
import subprocess
import tempfile

FLASH_BASE = 0x08000000
FLASH_SIZE = 1024 * 1024


def inspect_image(path):
    image = Path(path).resolve()
    data = image.read_bytes()
    if not 8 <= len(data) <= FLASH_SIZE:
        raise ValueError("A raw binary image must contain 8 bytes to 1 MiB.")
    stack, reset = struct.unpack_from("<II", data)
    if stack % 8 or not 0x20000000 < stack <= 0x20020000:
        raise ValueError("Initial stack is not aligned within STM32F405 general SRAM.")
    if reset & 1 == 0 or not FLASH_BASE + 8 <= (reset & ~1) < FLASH_BASE + len(data):
        raise ValueError("Reset vector is not Thumb code within an image linked at 0x08000000.")
    return image, data, hashlib.sha256(data).hexdigest()


def run(args, timeout=120):
    result = subprocess.run(args, capture_output=True, text=True, timeout=timeout)
    if result.returncode:
        raise RuntimeError((result.stdout + result.stderr).strip() or "dfu-util failed")
    return result.stdout + result.stderr


def discover(output):
    # Only the STM32 ROM DFU VID/PID, alt 0 internal flash; reject ambiguous boards.
    devices = []
    for line in output.splitlines():
        if "Found DFU: [0483:df11]" not in line or 'alt=0,' not in line: continue
        name = re.search(r'name="([^"]+)"', line)
        serial = re.search(r'serial="([^"]+)"', line)
        path = re.search(r'path="([^"]+)"', line)
        if not name or not name.group(1).startswith("@Internal Flash /0x08000000/"): continue
        if serial and path: devices.append({"serial": serial.group(1), "path": path.group(1), "memory": name.group(1)})
    return devices


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    sub.add_parser("list", help="Read-only DFU device discovery")
    inspect = sub.add_parser("inspect", help="Validate vectors and show image SHA-256 without USB access")
    inspect.add_argument("image")
    flash = sub.add_parser("flash", help="Write an explicitly reviewed image, then compare a USB read-back")
    flash.add_argument("image")
    flash.add_argument("--sha256", required=True, help="Exact hash of the image you reviewed")
    flash.add_argument("--board", required=True, choices=["easytvc-v0.1"], help="Your physical board; not auto-detected")
    flash.add_argument("--serial", help="Required when more than one STM32 DFU device is attached")
    flash.add_argument("--allow-blank", action="store_true", help="Acknowledge that safe_blank disables all application operation")
    args = parser.parse_args()
    try:
        if args.command in ("inspect", "flash"):
            image, data, digest = inspect_image(args.image)
            print(f"Image: {image}\nSize: {len(data)} bytes\nAddress: 0x{FLASH_BASE:08x}\nSHA-256: {digest}")
            print("Vector validation does not establish board compatibility or flight readiness.")
            if args.command == "inspect": return
            if not re.fullmatch(r"[0-9a-fA-F]{64}", args.sha256) or args.sha256.lower() != digest:
                raise ValueError("Hash mismatch. No device has been modified.")
            if "safe_blank" in image.name and not args.allow_blank:
                raise ValueError("This image only sleeps; it does not run TVC. Use --allow-blank only for deliberate bring-up.")
        executable = shutil.which("dfu-util")
        if not executable: raise RuntimeError("Install dfu-util with: brew install dfu-util")
        listing = run([executable, "-l"], timeout=10)
        if args.command == "list": print(listing); return
        devices = discover(listing)
        if args.serial: devices = [d for d in devices if d["serial"] == args.serial]
        if len(devices) != 1:
            raise ValueError("Select exactly one STM32 ROM DFU device with internal flash using --serial. No write attempted.")
        device = devices[0]
        if not device["serial"] or device["serial"] == "UNKNOWN":
            raise ValueError("A stable device serial is required. No write attempted.")
        base = [executable, "-d", "0483:df11", "-S", device["serial"], "-p", device["path"], "-a", "0"]
        print(f"Selected {args.board}, USB serial {device['serial']}. Downloading without starting the application.")
        with tempfile.TemporaryDirectory(prefix="easytvc-dfu-") as directory:
            # Write immutable reviewed bytes; never reopen a potentially changed source during transfer.
            staged = Path(directory) / "reviewed.bin"
            staged.write_bytes(data)
            try:
                print(run(base + ["-s", f"0x{FLASH_BASE:08x}", "-D", str(staged)]))
                readback = Path(directory) / "readback.bin"
                print(run(base + ["-s", f"0x{FLASH_BASE:08x}:{len(data)}", "-U", str(readback)]))
                if readback.read_bytes() != data:
                    raise RuntimeError("Read-back mismatch. Keep the board in DFU; application integrity is unverified.")
            except (RuntimeError, OSError, subprocess.TimeoutExpired) as error:
                raise RuntimeError(f"Programming or verification failed; flash may be partial. Keep the board in DFU. {error}") from error
        print("Verified: every downloaded byte matches. Board remains in DFU. Reset manually when ready for your bench test.")
    except (ValueError, OSError, RuntimeError, subprocess.TimeoutExpired) as error:
        parser.exit(1, f"{error}\n")


if __name__ == "__main__": main()
