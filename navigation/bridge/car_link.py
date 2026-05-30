"""Serial link to the Elegoo UNO car firmware.

Speaks the line-based ASCII protocol from message-schemas.md: sends DriveCommand
("DRV ...") and receives CarTelemetry ("TEL ..."). Works against the real Elegoo over
USB, or against bridge/fake_car.py over a virtual serial port (same code, just a
different port path).

pyserial is imported lazily so this module stays importable before deps are installed;
connect() raises a clear error if it is missing.
"""

from __future__ import annotations

import sys
from pathlib import Path
from typing import Optional

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "shared"))
from schemas.schemas import CarTelemetry, DriveCommand  # noqa: E402

try:
    import serial  # pyserial
except ImportError:  # pragma: no cover - exercised only without deps
    serial = None  # type: ignore


class CarLink:
    """Owns the pyserial connection to the car MCU."""

    def __init__(self, port: str = "/dev/ttyACM0", baud: int = 115200, timeout: float = 0.1) -> None:
        """Inputs: port, the serial device for the Elegoo UNO (or the fake car's PTY path);
        baud, the link speed (match the firmware, 115200); timeout, the read timeout in
        seconds (keeps read_telemetry from blocking the run loop).
        """
        self.port = port
        self.baud = baud
        self.timeout = timeout
        self._serial = None

    def connect(self) -> None:
        """Open the serial port. Raises RuntimeError if pyserial is not installed."""
        if serial is None:
            raise RuntimeError("pyserial not installed; run: pip install -r requirements.txt")
        self._serial = serial.Serial(self.port, self.baud, timeout=self.timeout)

    def close(self) -> None:
        """Close the serial port if open."""
        if self._serial is not None:
            self._serial.close()
            self._serial = None

    def send_drive(self, cmd: DriveCommand) -> None:
        """Send a DriveCommand to the car as a 'DRV ...' line.

        Inputs: cmd, the DriveCommand to send. Output: none.
        """
        if self._serial is None:
            raise RuntimeError("call connect() first")
        self._serial.write((cmd.to_serial_line() + "\n").encode())

    def read_telemetry(self) -> Optional[CarTelemetry]:
        """Read one CarTelemetry line if one is available within the read timeout.

        Output: a CarTelemetry, or None if no complete 'TEL ...' line arrived. Non-TEL or
        malformed lines are ignored (returns None).
        """
        if self._serial is None:
            raise RuntimeError("call connect() first")
        raw = self._serial.readline()
        if not raw:
            return None
        line = raw.decode(errors="ignore").strip()
        if not line.startswith("TEL"):
            return None
        try:
            return CarTelemetry.from_serial_line(line)
        except (ValueError, IndexError):
            return None
