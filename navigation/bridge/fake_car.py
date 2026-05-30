"""A DEV-ONLY fake Elegoo car over a virtual serial port.

Stands in for the physical Elegoo UNO R3 so the brain side (bridge/car_link.py and the
mapping logic) can be developed and tested with no hardware. It is the serial-side mirror
of the browser's fake map feed.

What it does:
  - Opens a pseudo-terminal (PTY) and prints a real serial device path. Point the brain at
    it: CarLink(port="<that path>"). The brain talks to it with ordinary pyserial, exactly
    as it would to the real car.
  - Accepts DRV lines ("DRV <linear> <angular> <stop>") and drives a simulated rover in a
    virtual square room.
  - Emits TEL lines ("TEL <ultrasonic> <bumper> <line_left> <line_center> <line_right>
    <timestamp>") at a fixed cadence, with sensor values that respond to the driving:
    the ultrasonic distance shrinks as the rover approaches a wall, the bumper trips on
    contact, and the line sensors fire near an edge.

Message contract: ../../docs/message-schemas.md (DriveCommand, CarTelemetry).

Usage:
    cd navigation
    python bridge/fake_car.py
    # note the printed device path, e.g. /dev/ttys012
    # then in the brain: CarLink(port="/dev/ttys012")

The simulation (FakeCar) is decoupled from the transport so it can be unit-tested without
a PTY. INTEGRATION: this whole file is dev-only; delete it once the real car is in the loop.
"""

from __future__ import annotations

import math
import os
import pty
import random
import select
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "shared"))
from schemas.schemas import CarTelemetry, DriveCommand  # noqa: E402

# Virtual room: axis-aligned square with walls at +/- ROOM_HALF meters, rover starts centered.
ROOM_HALF = 2.0
SENSOR_MAX_M = 2.5  # ultrasonic max range; beyond this it reports -1 (no echo)
EDGE_MARGIN_M = 0.12  # line sensors trip when this close to a wall (edge detection)


def _ray_to_walls(x: float, y: float, theta: float, half: float):
    """Distance from (x, y) heading theta to the first wall of the square room.

    Returns the positive distance, or None if the ray somehow hits nothing.
    """
    dx, dy = math.cos(theta), math.sin(theta)
    ts = []
    if dx > 1e-9:
        ts.append((half - x) / dx)
    elif dx < -1e-9:
        ts.append((-half - x) / dx)
    if dy > 1e-9:
        ts.append((half - y) / dy)
    elif dy < -1e-9:
        ts.append((-half - y) / dy)
    ts = [t for t in ts if t > 0]
    return min(ts) if ts else None


class FakeCar:
    """Simulated rover state and sensor model. Pure logic, no transport."""

    def __init__(self) -> None:
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0  # heading, radians
        self.v = 0.0  # commanded linear velocity, m/s
        self.w = 0.0  # commanded angular velocity, rad/s
        self.bumper = 0
        self._t0 = time.monotonic()

    def apply_drive_line(self, line: str) -> None:
        """Parse one DRV line and update the commanded velocities.

        Non-DRV or malformed lines are ignored (the real car would do the same).
        """
        try:
            cmd = DriveCommand.from_serial_line(line)
        except (ValueError, IndexError):
            return
        if cmd.stop:
            self.v = 0.0
            self.w = 0.0
        else:
            self.v = cmd.linear_velocity
            self.w = cmd.angular_velocity

    def step(self, dt: float) -> None:
        """Advance the simulated rover by dt seconds (differential-drive integration)."""
        if dt <= 0:
            return
        self.theta += self.w * dt
        nx = self.x + self.v * math.cos(self.theta) * dt
        ny = self.y + self.v * math.sin(self.theta) * dt
        self.bumper = 0
        if abs(nx) > ROOM_HALF:
            nx = max(-ROOM_HALF, min(ROOM_HALF, nx))
            self.bumper = 1
        if abs(ny) > ROOM_HALF:
            ny = max(-ROOM_HALF, min(ROOM_HALF, ny))
            self.bumper = 1
        self.x, self.y = nx, ny

    def _ultrasonic(self) -> float:
        d = _ray_to_walls(self.x, self.y, self.theta, ROOM_HALF)
        if d is None or d > SENSOR_MAX_M:
            return -1.0
        return round(max(0.0, d + random.uniform(-0.01, 0.01)), 3)  # a little noise

    def _line_flags(self):
        # Edge detection: fire when the rover is near a wall. All three for simplicity.
        near = ROOM_HALF - max(abs(self.x), abs(self.y))
        edge = 1 if near < EDGE_MARGIN_M else 0
        return edge, edge, edge

    def telemetry(self) -> CarTelemetry:
        """Build the current CarTelemetry reading."""
        left, center, right = self._line_flags()
        return CarTelemetry(
            ultrasonic_distance=self._ultrasonic(),
            bumper=self.bumper,
            line_left=left,
            line_center=center,
            line_right=right,
            timestamp=round(time.monotonic() - self._t0, 3),
        )

    def telemetry_line(self) -> str:
        """The current reading as a TEL wire line (no trailing newline)."""
        return self.telemetry().to_serial_line()


def open_virtual_port():
    """Create a PTY pair. Returns (master_fd, slave_fd, slave_name).

    The car loop reads/writes the master; the brain opens slave_name with pyserial.
    """
    master, slave = pty.openpty()
    slave_name = os.ttyname(slave)
    os.set_blocking(master, False)
    return master, slave, slave_name


def serve(master: int, hz: float = 20.0, stop_event=None, car: "FakeCar | None" = None) -> None:
    """Run the car loop on an open master fd: read DRV lines, emit TEL at `hz`.

    Runs until stop_event is set (if given) or the port closes. Pass a FakeCar to inspect
    its state from outside (used in tests).
    """
    car = car or FakeCar()
    period = 1.0 / hz
    buf = b""
    last_step = time.monotonic()
    last_tel = last_step

    while stop_event is None or not stop_event.is_set():
        r, _, _ = select.select([master], [], [], 0.005)
        if r:
            try:
                data = os.read(master, 4096)
            except OSError:
                data = b""
            if data:
                buf += data
                while b"\n" in buf:
                    line, buf = buf.split(b"\n", 1)
                    car.apply_drive_line(line.decode(errors="ignore"))

        now = time.monotonic()
        car.step(now - last_step)
        last_step = now

        if now - last_tel >= period:
            last_tel = now
            try:
                os.write(master, (car.telemetry_line() + "\n").encode())
            except OSError:
                break  # the brain closed the port


def run_pty(hz: float = 20.0) -> None:
    """Open a virtual serial port and run the car loop on it until interrupted."""
    master, slave, slave_name = open_virtual_port()
    print(f"[fake-car] virtual serial port ready: {slave_name}")
    print(f"[fake-car] point the brain at it:      CarLink(port='{slave_name}')")
    print(f"[fake-car] emitting TEL at {hz:.0f} Hz, accepting DRV. Ctrl-C to stop.")
    try:
        serve(master, hz)
    except KeyboardInterrupt:
        print("\n[fake-car] stopped")
    finally:
        os.close(master)
        os.close(slave)


if __name__ == "__main__":
    run_pty()
