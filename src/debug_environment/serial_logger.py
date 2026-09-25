#!/usr/bin/env python3
"""Record Compass serial output until Ctrl+C.

Install dependencies:
    python -m pip install pyserial

Run:
    python serial_logger.py --port /dev/cu.usbmodem101

Every line received from the board is written to the output file and echoed
to the terminal. Press Ctrl+C to stop recording.
"""

from __future__ import annotations

import argparse
from datetime import datetime
from pathlib import Path

try:
    import serial
except ImportError as error:
    raise SystemExit("Install the dependency with: python -m pip install pyserial") from error


def default_output_path() -> Path:
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return Path(f"serial_recording_{timestamp}.txt")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--port",
        default="/dev/cu.usbmodem101",
        help="serial port (default: /dev/cu.usbmodem101)",
    )
    parser.add_argument(
        "--baud",
        type=int,
        default=115200,
        help="serial baud rate (default: 115200)",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=None,
        help="CSV output path (default: timestamped file)",
    )
    return parser.parse_args()


def read_recording(port: str, baud: int, output_path: Path) -> None:
    print(f"Opening {port} at {baud} baud...")
    print(f"Recording serial output to {output_path}")
    print("Press Ctrl+C to stop.")

    with serial.Serial(port, baudrate=baud, timeout=1) as connection:
        with output_path.open("w", encoding="utf-8") as output_file:
            try:
                while True:
                    raw_line = connection.readline()
                    if not raw_line:
                        continue

                    line = raw_line.decode("utf-8", errors="replace")
                    print(line, end="", flush=True)
                    output_file.write(line)
                    output_file.flush()
            except KeyboardInterrupt:
                print(f"\nSaved serial recording: {output_path}")


def main() -> None:
    args = parse_args()
    output_path = args.output or default_output_path()

    try:
        read_recording(args.port, args.baud, output_path)
    except serial.SerialException as error:
        raise SystemExit(f"Could not open serial port {args.port}: {error}") from error


if __name__ == "__main__":
    main()
