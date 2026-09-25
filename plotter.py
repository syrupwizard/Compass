"""Plot Compass sensor data live from the serial port."""

from __future__ import annotations

import argparse
import threading
from collections import deque

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import serial


CSV_COLUMNS = ["timestamp_ms", "qw", "qx", "qy", "qz", "yaw", "pitch", "roll"]
PLOT_GROUPS = {
    "Quaternion": ["qw", "qx", "qy", "qz"],
    "Yaw / Pitch / Roll": ["yaw", "pitch", "roll"],
}
Y_LIMITS = {
    "Quaternion": (-1.1, 1.1),
    "Yaw / Pitch / Roll": (-360, 360),
}
FIGURE_HEIGHT = 8

# Global flag to stop the serial thread gracefully
is_running = True

def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="/dev/cu.usbmodem101")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--window", type=int, default=300)
    return parser.parse_args()


def parse_sample(line: str) -> dict[str, float] | None:
    values = line.split(",")
    if len(values) == len(CSV_COLUMNS):
        try:
            return dict(zip(CSV_COLUMNS, (float(value) for value in values)))
        except ValueError:
            return None
    return None


def serial_reader(port: str, baud: int, samples: deque) -> None:
    """Runs in a background thread to read serial data without blocking the GUI."""
    global is_running
    buffer = ""
    
    try:
        with serial.Serial(port, baudrate=baud, timeout=1) as connection:
            while is_running:
                raw_data = connection.read(256)
                if not raw_data:
                    continue

                buffer += raw_data.decode("utf-8", errors="replace")
                values = buffer.split(",")
                buffer = values.pop()

                while len(values) >= len(CSV_COLUMNS):
                    parsed_sample = parse_sample(",".join(values[:len(CSV_COLUMNS)]))
                    values = values[len(CSV_COLUMNS):]
                    if parsed_sample is not None:
                        samples.append(parsed_sample)
                    
    except serial.SerialException as error:
        print(f"\nSerial error: {error}")
        is_running = False


def main() -> None:
    global is_running
    args = parse_args()
    
    # 1. Setup Data Structures
    samples = deque(maxlen=args.window)

    # 2. Start Background Serial Thread
    print(f"Opening {args.port} at {args.baud} baud. Close the plot window to stop.")
    thread = threading.Thread(
        target=serial_reader, 
        args=(args.port, args.baud, samples), 
        daemon=True
    )
    thread.start()

    # 3. Setup the Plot
    figure, axes = plt.subplots(
        len(PLOT_GROUPS), 1, sharex=True, 
        figsize=(13, FIGURE_HEIGHT), constrained_layout=True
    )
    
    lines = {}
    for axis, (title, columns) in zip(axes, PLOT_GROUPS.items()):
        axis.set_title(title)
        axis.set_ylabel("value")
        axis.set_ylim(*Y_LIMITS[title])
        axis.set_xlim(0, args.window)
        axis.grid(True, alpha=0.3)
        lines[title] = {
            column: axis.plot([], [], label=column)[0] for column in columns
        }
        axis.legend(loc="upper right")
        
    axes[-1].set_xlabel("Sample number")
    figure.suptitle("Compass Live Serial Data")

    # 4. Animation Update Function
    def update_plot(frame):
        if not samples:
            return
            
        # Snapshot the deque to avoid size changing while iterating
        current_samples = list(samples)
        sample_numbers = range(len(current_samples))
        
        for title, columns in PLOT_GROUPS.items():
            for column in columns:
                lines[title][column].set_data(
                    sample_numbers, [sample[column] for sample in current_samples]
                )

    # Update the plot every 50ms (~20 FPS). This frees up the CPU immensely.
    ani = FuncAnimation(figure, update_plot, interval=50, cache_frame_data=False)

    # plt.show() blocks the main thread until the window is closed
    try:
        plt.show()
    except KeyboardInterrupt:
        pass
    finally:
        # Signal the background thread to shut down
        is_running = False
        print("\nStopped live plot.")


if __name__ == "__main__":
    main()