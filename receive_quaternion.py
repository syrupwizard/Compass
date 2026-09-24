#!/usr/bin/env python3
"""Receive and decode quaternion notifications from the Compass firmware."""

import argparse
import asyncio
import struct

from bleak import BleakClient, BleakScanner

UART_SERVICE_UUID = "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
DEFAULT_CHARACTERISTIC_UUID = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"

def decode_quaternion(data: bytearray) -> tuple[float, float, float, float]:
    """Decode binary quaternion data or CSV data from the Compass firmware."""
    if len(data) == 16:
        return struct.unpack("<4f", data)

    try:
        values = [float(value) for value in data.decode().strip().split(",")]
    except (UnicodeDecodeError, ValueError) as error:
        raise ValueError(f"unrecognized quaternion packet: {data!r}") from error

    if len(values) == 5:
        return tuple(values[1:])
    if len(values) == 4:
        return tuple(values)
    raise ValueError(f"expected 4 or 5 CSV values, received {len(values)}")


def print_quaternion(data: bytearray) -> None:
    try:
        w, x, y, z = decode_quaternion(data)
    except ValueError as error:
        print(f"Invalid quaternion packet: {error}")
        return
    print(f"w={w:.6f}, x={x:.6f}, y={y:.6f}, z={z:.6f}")


async def find_device(name: str | None, service_uuid: str):
    print(f"Scanning for devices advertising {service_uuid}...")
    discovered = await BleakScanner.discover(return_adv=True)
    service_uuid = service_uuid.lower()
    for device, advertisement in discovered.values():
        advertised_services = {
            uuid.lower() for uuid in advertisement.service_uuids
        }
        if service_uuid in advertised_services and (
            name is None or device.name == name
        ):
            return device
    return None


async def receive_quaternions(
    device_name: str,
    address: str | None,
    service_uuid: str,
    characteristic_uuid: str,
) -> None:
    device = (
        await find_device(device_name, service_uuid)
        if address is None
        else address
    )
    if device is None:
        raise RuntimeError(f"could not find a device advertising {service_uuid!r}")

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
    parser.add_argument(
        "--name",
        default=None,
        help="optional BLE device name filter",
    )
    parser.add_argument("--address", help="BLE address or macOS device UUID")
    parser.add_argument(
        "--service",
        default=UART_SERVICE_UUID,
        help="BLE service UUID used to identify compatible devices",
    )
    parser.add_argument(
        "--characteristic",
        default=DEFAULT_CHARACTERISTIC_UUID,
        help="quaternion notification characteristic UUID",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    try:
        asyncio.run(
            receive_quaternions(
                args.name,
                args.address,
                args.service,
                args.characteristic,
            )
        )
    except KeyboardInterrupt:
        print("\nDisconnected.")


if __name__ == "__main__":
    main()