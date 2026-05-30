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
