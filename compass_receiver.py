#!/usr/bin/env python3
"""Auto-connecting BLE receiver for the Bluefruit Compass firmware.

Run it and it will scan for the board, connect, and print heading + quaternion
samples. If the board resets, goes out of range, or isn't powered on yet, the
script keeps scanning and reconnects by itself. Press Ctrl+C to quit.

Setup (Windows 10+, macOS, Linux, Raspberry Pi):
    python -m pip install bleak

Platform notes:
    Linux   BlueZ must be running (systemctl status bluetooth).
    macOS   Allow Bluetooth for your terminal app when prompted
            (System Settings > Privacy & Security > Bluetooth).
    Windows Turn Bluetooth on; no pairing is needed.

Firmware line format (25 Hz):  heading,qw,qx,qy,qz\\n
"""

from __future__ import annotations

import argparse
import asyncio
import sys
import time

try:
    from bleak import BleakClient, BleakScanner
    from bleak.exc import BleakError
except ImportError:
    sys.exit("This script needs the 'bleak' package")

# Nordic UART Service, as exposed by Adafruit's BLEUart
UART_SERVICE_UUID = "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
UART_TX_UUID = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"  # board -> host (notify)

STALL_SECONDS = 5.0     # warn if connected but silent this long
MAX_BACKOFF_S = 10.0    # longest wait between reconnect attempts


# ------------------------------------------------------------------ parsing --

def parse_line(line: bytes) -> tuple[float, tuple[float, float, float, float]]:
    """Decode 'heading,qw,qx,qy,qz' into (heading_deg, (w, x, y, z))."""
    try:
        values = [float(v) for v in line.decode("ascii").strip().split(",")]
    except (UnicodeDecodeError, ValueError) as error:
        raise ValueError(f"unrecognized packet {line!r}") from error
    if len(values) != 5:
        raise ValueError(f"expected 5 values, got {len(values)}: {line!r}")
    return values[0], (values[1], values[2], values[3], values[4])


def handle_sample(heading: float, quat: tuple[float, float, float, float]) -> None:
    """Called once per decoded sample. Edit this to log, plot, forward, etc."""
    w, x, y, z = quat
    print(f"heading={heading:6.1f}  w={w:7.3f}  x={x:7.3f}  y={y:7.3f}  z={z:7.3f}",
          flush=True)


class LineReceiver:
    """Reassembles newline-terminated lines from arbitrarily split notifications."""

    MAX_BUFFER = 256

    def __init__(self) -> None:
        self.buffer = bytearray()
        self.synced = False            # first line after connect may be partial
        self.packets = 0
        self.last_rx = time.monotonic()

    def feed(self, data: bytes) -> list[bytes]:
        self.last_rx = time.monotonic()
        self.buffer.extend(data)
        lines = []
        while True:
            end = self.buffer.find(b"\n")
            if end < 0:
                break
            line = bytes(self.buffer[:end])
            del self.buffer[:end + 1]
            if not self.synced:        # drop the possibly truncated first line
                self.synced = True
                continue
            if line.strip():
                lines.append(line)
        if len(self.buffer) > self.MAX_BUFFER:   # no newline in sight: resync
            self.buffer.clear()
        return lines


# ---------------------------------------------------------------- BLE logic --

async def discover(args: argparse.Namespace):
    """Return a BLEDevice, or None if nothing suitable was seen in time."""
    if args.address:
        # Works with MAC addresses (Windows/Linux) and CoreBluetooth UUIDs (macOS)
        return await BleakScanner.find_device_by_address(
            args.address, timeout=args.scan_timeout)

    service = args.service.lower()
    wanted_name = args.name

    def matches(device, adv) -> bool:
        if wanted_name is not None:
            return (adv.local_name or device.name) == wanted_name
        return service in [u.lower() for u in adv.service_uuids]

    return await BleakScanner.find_device_by_filter(
        matches, timeout=args.scan_timeout)


def pick_notify_characteristic(client: BleakClient, args: argparse.Namespace):
    """Use --characteristic if given, else find the UART service's notify char."""
    if args.characteristic:
        return args.characteristic
    service = client.services.get_service(args.service)
    if service is not None:
        for char in service.characteristics:
            if "notify" in char.properties:
                return char
    return UART_TX_UUID


async def run_session(device, args: argparse.Namespace) -> bool:
    """Connect and stream until disconnected. Returns True if data was received."""
    loop = asyncio.get_running_loop()
    disconnected = asyncio.Event()
    receiver = LineReceiver()

    def on_disconnect(_client) -> None:
        loop.call_soon_threadsafe(disconnected.set)

    def on_data(_sender, data: bytearray) -> None:
        for line in receiver.feed(data):
            try:
                heading, quat = parse_line(line)
            except ValueError as error:
                print(f"Skipping bad packet: {error}")
                continue
            receiver.packets += 1
            handle_sample(heading, quat)

    label = device.name or device.address
    print(f"Connecting to {label} ({device.address})...")
    async with BleakClient(device, disconnected_callback=on_disconnect,
                           timeout=args.connect_timeout) as client:
        char = pick_notify_characteristic(client, args)
        await client.start_notify(char, on_data)
        receiver.last_rx = time.monotonic()
        print("Connected. Streaming; press Ctrl+C to stop.")

        warned = False
        try:
            while not disconnected.is_set():
                try:
                    await asyncio.wait_for(disconnected.wait(), timeout=1.0)
                except asyncio.TimeoutError:
                    pass
                silent = time.monotonic() - receiver.last_rx
                if silent > STALL_SECONDS and not warned:
                    warned = True
                    print(f"Connected but no data for {STALL_SECONDS:.0f} s. If the "
                          "board is plugged into a computer with a serial monitor "
                          "open, the firmware is in calibration mode and pauses "
                          "the BLE stream. Close the monitor to resume.")
                elif silent <= STALL_SECONDS:
                    warned = False
        finally:
            if client.is_connected:
                try:
                    await client.stop_notify(char)
                except Exception:
                    pass   # link may already be going down

    return receiver.packets > 0


async def run(args: argparse.Namespace) -> None:
    backoff = 1.0
    failed_scans = 0
    print("Scanning for the compass..." if not args.address
          else f"Looking for {args.address}...")

    while True:
        got_data = False
        device = None
        try:
            device = await discover(args)
            if device is None:
                failed_scans += 1
                if failed_scans == 1:
                    print("Not found yet. Check that the board is powered on and "
                          "not already connected to another device (only one "
                          "connection is allowed). Still searching...")
            else:
                failed_scans = 0
                got_data = await run_session(device, args)
                print("Disconnected." if got_data else "Connection ended before any data.")
                if not args.no_reconnect:
                    print("Reconnecting...")
        except (BleakError, asyncio.TimeoutError, OSError) as error:
            print(f"Bluetooth problem: {error or type(error).__name__}. "
                  "Is Bluetooth turned on and permitted for this app?")

        if args.no_reconnect and (got_data or device is not None):
            return
        backoff = 1.0 if got_data else min(backoff * 2, MAX_BACKOFF_S)
        await asyncio.sleep(backoff)


# --------------------------------------------------------------------- main --

def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Auto-connecting Compass BLE receiver.")
    p.add_argument("--name",
                   help="connect to the device advertising this exact name "
                        "instead of matching the UART service")
    p.add_argument("--address",
                   help="skip scanning by service; connect to this BLE address "
                        "(macOS: the CoreBluetooth UUID)")
    p.add_argument("--service", default=UART_SERVICE_UUID,
                   help="service UUID used to recognise the board")
    p.add_argument("--characteristic",
                   help="notify characteristic UUID (default: auto-detect)")
    p.add_argument("--scan-timeout", type=float, default=8.0,
                   help="seconds per scan attempt (default 8)")
    p.add_argument("--connect-timeout", type=float, default=15.0,
                   help="seconds to wait for a connection (default 15)")
    p.add_argument("--no-reconnect", action="store_true",
                   help="exit after the first session instead of reconnecting")
    return p.parse_args()


def main() -> None:
    args = parse_args()
    try:
        asyncio.run(run(args))
    except KeyboardInterrupt:
        print("\nStopped.")


if __name__ == "__main__":
    main()
