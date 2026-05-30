"""Modulino sensor I/O over Qwiic / I2C.

Interface stub only. Reads the Modulino Movement (IMU) and Modulino Distance (ToF),
which connect to the UNO Q over Qwiic / I2C. Produces ImuSample for the SLAM frontend.
"""

from __future__ import annotations

import sys
from pathlib import Path
from typing import Optional

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "shared"))
from schemas.schemas import ImuSample  # noqa: E402


class ModulinoIO:
    """Owns the I2C bus handles to the Modulino IMU and ToF sensors."""

    def __init__(self, i2c_bus: int = 1) -> None:
        """Inputs: i2c_bus, the I2C bus index on the Dragonwing.

        TODO: store config; open the bus in connect(). Candidate drivers: the Arduino
              Modulino Python library, or a smbus2-based driver against the sensor regs.
        """
        self.i2c_bus = i2c_bus
        self._bus = None

    def connect(self) -> None:
        """Open the I2C bus and probe the IMU and ToF addresses.

        TODO: open smbus2.SMBus(self.i2c_bus); verify the Modulino device addresses.
        """
        raise NotImplementedError("modulino connect not implemented yet")

    def read_imu(self) -> Optional[ImuSample]:
        """Read one IMU sample from the Modulino Movement sensor.

        Output: an ImuSample (accel[3] in m/s^2, gyro[3] in rad/s, timestamp), or None.
        TODO: read accel and gyro registers, convert to SI units, stamp with time.time().
        """
        raise NotImplementedError("read_imu not implemented yet")

    def read_distance(self) -> Optional[float]:
        """Read one range value from the Modulino Distance (ToF) sensor.

        Output: distance in meters, or None if no valid reading.
        TODO: read the ToF register and convert to meters.
        """
        raise NotImplementedError("read_distance not implemented yet")
