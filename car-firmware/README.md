# car-firmware

Role: the dedicated motor controller. Parses drive commands from the brain, drives the
motors, and streams sensor telemetry back.

Runs on: the Elegoo Smart Robot Car V4's onboard Arduino UNO R3 + TB6612 motor driver.
This MCU stays as the low-level real-time controller. It has a servo-mounted ultrasonic
sensor and line sensors, and no wheel encoders.

Tech stack: Arduino C++ (the `.ino` sketch plus `src/` modules). Build with the Arduino
IDE or `arduino-cli`.

## Build and run

Using the Arduino IDE:

1. Open [car-firmware.ino](car-firmware.ino).
2. Select board "Arduino UNO" and the correct serial port.
3. Upload. The serial link to the brain runs at 115200 baud.

Using `arduino-cli`:

```bash
arduino-cli compile --fqbn arduino:avr:uno car-firmware
arduino-cli upload  --fqbn arduino:avr:uno -p /dev/ttyACM0 car-firmware
```

## Modules

- [src/motor_control.h](src/motor_control.h) / `.cpp`: `setSpeed(left, right)`, `stop()`.
- [src/serial_protocol.h](src/serial_protocol.h) / `.cpp`: parse the line-based command
  protocol from the brain, format telemetry lines back.
- [src/sensors.h](src/sensors.h) / `.cpp`: ultrasonic read, line sensor read.

## Hardware pins: what's connected and where

The firmware logic is implemented (DRV parsing, kinematics, PWM control, ultrasonic timing,
TEL formatting). The pin constants are now filled with the **Elegoo Smart Car V4 defaults**.
The car has two sets of sensors/actuators wired to this MCU:

> **Sensing is currently phone (LiDAR) + side ToF only.** The ultrasonic and line sensors are
> **not read** right now - their pins below are documentation, and `sensors.cpp` reports
> sentinels for them (the TEL message keeps the fields). Restore the reads from git history to
> bring them back.

| What | Pins (V4 default) | Notes |
| --- | --- | --- |
| Motors (TB6612) | PWMA `5`, AIN1 `7`, PWMB `6`, BIN1 `8`, STBY `3` | V4 uses ONE direction pin per motor + PWM + shared standby (not IN1+IN2). |
| Ultrasonic (HC-SR04) | TRIG `13`, ECHO `12` | On the front servo. *Not read now.* Verify these if re-enabling - they vary by revision. |
| Line sensors (ITR20001 x3) | left `A2`, center `A1`, right `A0` | **Analog** (`analogRead` + threshold). *Not read now.* |
| Servo (ultrasonic pan) | `10` | Not driven (ultrasonic removed for now). |

The phone (camera + LiDAR + IMU) is the rover's *perception* and connects to the UNO Q's
Linux side, not here. This MCU only handles the motors and the car's own ultrasonic/line
sensors.

**Verify before flashing:** these are the documented V4 values, but copy the exact numbers
from the `DeviceDriverSet_xxx0.h` in your kit's tutorial download to be sure (the ultrasonic
pins especially). Then calibrate on the bench:

- [src/motor_control.cpp](src/motor_control.cpp): `LEFT_INVERT` / `RIGHT_INVERT` (flip a
  backwards wheel), `SWAP_LEFT_RIGHT` (if left/right are reversed).
- [src/sensors.cpp](src/sensors.cpp): `LINE_THRESHOLD` and `LINE_ABOVE_IS_ON` (print the raw
  `analogRead` values over your floor vs an edge, pick a midpoint and the right polarity).
- [car-firmware.ino](car-firmware.ino): `TRACK_WIDTH_M` (measure the wheel spacing, ~0.15 m)
  and `MAX_WHEEL_SPEED_MPS` (time a full-speed run to calibrate m/s -> PWM).

**UNO Q caveat (3.3V vs 5V):** these pins are the UNO-header pins the shield uses, valid on
a 5V UNO R3. If you flash this to the UNO Q's microcontroller instead, (a) confirm the UNO Q
maps the same header pins with Arduino numbering, and (b) **level-shift the 5V sensor
outputs** (the ultrasonic ECHO line, and the line sensors if powered at 5V) before they
reach the 3.3V GPIO, or you can damage a pin.

Bring-up order: flash, then drive it from a laptop with
`navigation/bridge/drive_test.py --port /dev/cu.usbmodemXXXX` and adjust the invert/swap
flags until it drives correctly. The DRV parser and TEL formatter are hardware-independent
and already verified against the Python contract.

## Message schemas

Defined in [../docs/message-schemas.md](../docs/message-schemas.md). Field names match the
Python and JS sides exactly.

- Consumes: `DriveCommand` over USB serial. Wire format
  `DRV <linear_velocity> <angular_velocity> <stop>`.
- Produces: `CarTelemetry` over USB serial. Wire format
  `TEL <ultrasonic_distance> <bumper> <line_left> <line_center> <line_right> <timestamp>`.

## Notes

- No wheel encoders: this firmware does not produce odometry. Odometry is visual-inertial
  on the brain. The car only reports range and line and bumper flags.
- Safety watchdog: if no `DriveCommand` arrives within a timeout, the firmware brakes so a
  brain crash never leaves the motors running. See the `TODO` in the sketch.
