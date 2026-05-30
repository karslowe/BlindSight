/*
 * Recon Rover - car firmware
 *
 * Runs on the Elegoo Smart Robot Car V4's Arduino UNO R3 + TB6612 motor driver.
 * Role: parse DriveCommand lines from the brain, drive the motors, and stream
 * CarTelemetry lines back.
 *
 * Message contract: ../docs/message-schemas.md
 *   Consumes: "DRV <linear_velocity> <angular_velocity> <stop>"
 *   Produces: "TEL <ultrasonic_distance> <bumper> <line_left> <line_center> <line_right> <timestamp>"
 *
 * This is a scaffold. The module bodies in src/ are stubs with TODO markers.
 */

#include "src/motor_control.h"
#include "src/serial_protocol.h"
#include "src/sensors.h"

// Serial baud to the brain (UNO Q).
static const unsigned long SERIAL_BAUD = 115200;

// Safety watchdog: brake if no DriveCommand has arrived within this many ms.
static const unsigned long COMMAND_TIMEOUT_MS = 500;

// How often to publish telemetry back to the brain.
static const unsigned long TELEMETRY_PERIOD_MS = 50;

static unsigned long last_command_ms = 0;
static unsigned long last_telemetry_ms = 0;

void setup() {
  Serial.begin(SERIAL_BAUD);
  motor_control_init();
  sensors_init();
  // TODO: initialize the ultrasonic servo to its forward-facing home position.
}

void loop() {
  // 1. Parse any incoming DriveCommand line and apply it to the motors.
  DriveCommand cmd;
  if (serial_protocol_poll(&cmd)) {
    last_command_ms = millis();
    if (cmd.stop) {
      motor_control_stop();
    } else {
      // TODO: convert (linear_velocity, angular_velocity) into left/right wheel
      //       speeds (differential drive kinematics) and call setSpeed().
      //       left  = linear - angular * HALF_TRACK_WIDTH
      //       right = linear + angular * HALF_TRACK_WIDTH
      //       then map m/s to the TB6612 PWM range.
      motor_control_set_speed(0, 0);  // placeholder until kinematics are filled in
    }
  }

  // 2. Safety watchdog: brake if the brain has gone quiet.
  if (millis() - last_command_ms > COMMAND_TIMEOUT_MS) {
    motor_control_stop();
  }

  // 3. Publish telemetry on a fixed cadence.
  if (millis() - last_telemetry_ms >= TELEMETRY_PERIOD_MS) {
    last_telemetry_ms = millis();
    CarTelemetry tel = sensors_read();
    serial_protocol_send_telemetry(&tel);
  }
}
