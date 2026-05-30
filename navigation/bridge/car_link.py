"""Serial link to the Elegoo UNO car firmware.

Interface stub only. Speaks the line-based ASCII protocol from message-schemas.md:
sends DriveCommand ("DRV ...") and receives CarTelemetry ("TEL ...").
"""

from __future__ import annotations

import sys
from pathlib import Path
from typing import Optional

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "shared"))
from schemas.schemas import CarTelemetry, DriveCommand  # noqa: E402


class CarLink:
    """Owns the pyserial connection to the car MCU."""

    def __init__(self, port: str = "/dev/ttyACM0", baud: int = 115200) -> None:
        """Inputs: port, the serial device for the Elegoo UNO; baud, the link speed.

        TODO: store config; do not open the port until connect() is called.
        """
        self.port = port
        self.baud = baud
        self._serial = None  # TODO: serial.Serial handle once connected

    def connect(self) -> None:
        """Open the serial port.

        TODO: self._serial = serial.Serial(self.port, self.baud, timeout=...).
        """
        raise NotImplementedError("car link connect not implemented yet")

    def send_drive(self, cmd: DriveCommand) -> None:
        """Send a DriveCommand to the car.

        Inputs: cmd, the DriveCommand to send.
        Output: none.
        TODO: write cmd.to_serial_line() + "\n" to the serial port.
        """
        raise NotImplementedError("send_drive not implemented yet")

    def read_telemetry(self) -> Optional[CarTelemetry]:
        """Read the latest CarTelemetry line if one is available.

        Output: a CarTelemetry, or None if no complete line is waiting.
        TODO: read a line; if it starts with "TEL", parse via
              CarTelemetry.from_serial_line(line). Ignore other lines.
        """
        raise NotImplementedError("read_telemetry not implemented yet")
