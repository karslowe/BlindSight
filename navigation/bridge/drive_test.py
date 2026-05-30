"""Make the car move: stream a scripted DRV sequence and print the TEL coming back.

This is the bring-up tool for the drive path. It needs no SLAM, no mapping, no brain
logic - just the serial link. Because DRV is plain ASCII, this same tool drives:
  - the fake car (dev, no hardware): bridge/fake_car.py prints a device path, pass it here.
  - the REAL Elegoo (once the firmware parses DRV and drives the motors): pass /dev/ttyACM0.

Usage:
    # Terminal 1: start the fake car (dev)
    python bridge/fake_car.py            # prints e.g. /dev/ttys012

    # Terminal 2: drive it
    python bridge/drive_test.py --port /dev/ttys012

    # Or drive the real car:
    python bridge/drive_test.py --port /dev/ttyACM0

The car needs a continuous command stream (the firmware watchdog brakes on silence), so
each step is streamed at --rate Hz for its duration, then a stop is sent at the end.
"""

from __future__ import annotations

import argparse
import sys
import threading
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "shared"))
from schemas.schemas import DriveCommand  # noqa: E402

try:
    from car_link import CarLink  # when run as: python bridge/drive_test.py
except ImportError:  # pragma: no cover
    from bridge.car_link import CarLink

# A short canned drive: (DriveCommand, seconds). Edit freely for your own test runs.
SCRIPT = [
    (DriveCommand(0.20, 0.0), 1.5),       # forward
    (DriveCommand(0.0, 0.8), 1.0),        # turn left in place
    (DriveCommand(0.20, 0.0), 1.5),       # forward again
    (DriveCommand(0.0, 0.0, stop=1), 0.5),  # stop
]


def _telemetry_printer(link: CarLink, stop_event: threading.Event) -> None:
    """Background reader: print the car's telemetry a few times a second."""
    last_print = 0.0
    while not stop_event.is_set():
        tel = link.read_telemetry()
        now = time.monotonic()
        if tel is not None and now - last_print >= 0.25:
            last_print = now
            print(
                f"  TEL  ultrasonic={tel.ultrasonic_distance:6.2f} m  "
                f"bumper={tel.bumper}  line={tel.line_left}{tel.line_center}{tel.line_right}"
            )


def run(port: str, rate: float = 20.0) -> None:
    link = CarLink(port=port)
    link.connect()
    print(f"[drive-test] connected to {port}; streaming the script at {rate:.0f} Hz")

    stop_event = threading.Event()
    reader = threading.Thread(target=_telemetry_printer, args=(link, stop_event), daemon=True)
    reader.start()

    period = 1.0 / rate
    try:
        for cmd, seconds in SCRIPT:
            label = "STOP" if cmd.stop else f"v={cmd.linear_velocity:+.2f} w={cmd.angular_velocity:+.2f}"
            print(f"[drive-test] {label}  for {seconds:.1f}s")
            end = time.monotonic() + seconds
            while time.monotonic() < end:
                link.send_drive(cmd)
                time.sleep(period)
    finally:
        link.send_drive(DriveCommand(0.0, 0.0, stop=1))  # always leave it stopped
        stop_event.set()
        time.sleep(0.1)
        link.close()
        print("[drive-test] done; car stopped")


def main() -> None:
    parser = argparse.ArgumentParser(description="Stream a scripted DRV sequence to the car.")
    parser.add_argument("--port", required=True, help="serial device (fake car PTY path or /dev/ttyACM0)")
    parser.add_argument("--rate", type=float, default=20.0, help="command stream rate, Hz")
    args = parser.parse_args()
    run(args.port, args.rate)


if __name__ == "__main__":
    main()
