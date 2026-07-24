#!/usr/bin/env python3
"""Capture an OpenNextion screenshot from UART1 and save it as PNG.

Connect USB-UART TX to GPIO12, RX to GPIO13, and connect GND.  UART settings
are 921600 baud, 8N1, no flow control.
"""

import argparse
import binascii
import struct
import sys
import time
from pathlib import Path

try:
    import serial
    from PIL import Image
except ImportError as exc:
    raise SystemExit("Install dependencies first: pip install pyserial Pillow") from exc


MAGIC = b"ONXS"
HEADER_FORMAT = "<4sBBHHHII"
HEADER_SIZE = struct.calcsize(HEADER_FORMAT)


def read_exact(port, size, deadline):
    result = bytearray()
    while len(result) < size:
        if time.monotonic() >= deadline:
            raise TimeoutError(f"received {len(result)} of {size} bytes")
        result.extend(port.read(size - len(result)))
    return bytes(result)


def read_header(port, deadline):
    window = bytearray()
    while time.monotonic() < deadline:
        window.extend(port.read(1))
        if len(window) > len(MAGIC):
            del window[0]
        if bytes(window) != MAGIC:
            continue
        values = struct.unpack(HEADER_FORMAT, MAGIC + read_exact(port, HEADER_SIZE - 4, deadline))
        _, version, pixel_format, header_size, width, height, data_size, crc32 = values
        if version != 1 or header_size != HEADER_SIZE:
            window.clear()
            continue
        if pixel_format != 2 or not width or not height or data_size != width * height * 2:
            raise ValueError("invalid screenshot header")
        return width, height, data_size, crc32
    raise TimeoutError("no ONXS screenshot header received")


def save_png(pixels, width, height, output):
    rgb = bytearray(width * height * 3)
    for index, (pixel,) in enumerate(struct.iter_unpack(">H", pixels)):
        offset = index * 3
        rgb[offset] = ((pixel >> 11) & 0x1F) * 255 // 31
        rgb[offset + 1] = ((pixel >> 5) & 0x3F) * 255 // 63
        rgb[offset + 2] = (pixel & 0x1F) * 255 // 31
    Image.frombytes("RGB", (width, height), bytes(rgb)).save(output, "PNG")


def main():
    parser = argparse.ArgumentParser(description="Capture an OpenNextion RGB565 screenshot over UART1.")
    parser.add_argument("--port", required=True, help="for example COM5 or /dev/ttyUSB0")
    parser.add_argument("--output", default="screenshot.png")
    parser.add_argument("--baud", type=int, default=460800)
    parser.add_argument("--timeout", type=float, default=15)
    parser.add_argument("--no-command", action="store_true", help="only receive; do not transmit save\\n")
    args = parser.parse_args()

    with serial.Serial(args.port, args.baud, timeout=0.1) as port:
        port.reset_input_buffer()
        if not args.no_command:
            port.write(b"save\n")
            port.flush()
        deadline = time.monotonic() + args.timeout
        width, height, data_size, expected_crc = read_header(port, deadline)
        pixels = read_exact(port, data_size, deadline)

    actual_crc = binascii.crc32(pixels) & 0xFFFFFFFF
    if actual_crc != expected_crc:
        raise ValueError(f"CRC mismatch: expected {expected_crc:08X}, got {actual_crc:08X}")
    output = Path(args.output)
    save_png(pixels, width, height, output)
    print(f"Saved {width}x{height} screenshot to {output}")


if __name__ == "__main__":
    try:
        main()
    except (serial.SerialException, TimeoutError, ValueError) as exc:
        sys.exit(f"Capture failed: {exc}")
