#!/usr/bin/env python3
"""Receive and decode quaternion notifications from the Compass firmware."""

import argparse
import asyncio
import struct

from bleak import BleakClient, BleakScanner
import asyncio; from bleak import BleakScanner; devices = asyncio.run(BleakScanner.discover(timeout=8)); print("\n".join(f"{d.name!r} | {d.address}" for d in devices))

DEFAULT_DEVICE_NAME = "Bluefruit Compass"
DEFAULT_CHARACTERISTIC_UUID = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"

def decode_quaternion(data: bytearray) -> tuple[float, float, float, float]:
    """Decode the firmware's 16-byte little-endian float payload."""
    if len(data) != 16:
        raise ValueError(f"expected 16 bytes, received {len(data)}")
    return struct.unpack("<4f", data)


def print_quaternion(data: bytearray) -> None:
    try:
        w, x, y, z = decode_quaternion(data)
    except ValueError as error:
        print(f"Invalid quaternion packet: {error}")
        return
    print(f"w={w:.6f}, x={x:.6f}, y={y:.6f}, z={z:.6f}")


async def find_device(name: str):
    print(f"Scanning for {name!r}...")
    for device in await BleakScanner.discover():
        if device.name == name:
            return device
    return None


async def receive_quaternions(
    device_name: str,
    address: str | None,
    characteristic_uuid: str,
) -> None:
    device = await find_device(device_name) if address is None else address
    if device is None:
        raise RuntimeError(f"could not find BLE device {device_name!r}")

    device_address = device if isinstance(device, str) else device.address
    print(f"Connecting to {device_address}...")

    async with BleakClient(device_address) as client:
        print("Connected. Waiting for notifications; press Ctrl+C to stop.")

        def handle_notification(_sender, data: bytearray) -> None:
            print_quaternion(data)

        await client.start_notify(characteristic_uuid, handle_notification)
        print("Initial quaternion:")
        print_quaternion(await client.read_gatt_char(characteristic_uuid))
        try:
            await asyncio.Event().wait()
        finally:
            await client.stop_notify(characteristic_uuid)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--name", default=DEFAULT_DEVICE_NAME)
    parser.add_argument("--address", help="BLE address or macOS device UUID")
    parser.add_argument(
        "--characteristic",
        default=DEFAULT_CHARACTERISTIC_UUID,
        help="quaternion notification characteristic UUID",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    try:
        asyncio.run(receive_quaternions(args.name, args.address, args.characteristic))
    except KeyboardInterrupt:
        print("\nDisconnected.")


if __name__ == "__main__":
    main()