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

// ===================== SET FROM THE SMART CAR V4 (calibration) =====================
// Distance between the two drive wheels, in meters. Measure it with a ruler.
static const float TRACK_WIDTH_M = 0.15f;        // TODO: measure
// Wheel linear speed at full PWM (255), in meters/second. Calibrate by driving at full
// speed over a known distance and timing it, then v = distance / time.
static const float MAX_WHEEL_SPEED_MPS = 0.6f;   // TODO: calibrate
// ==================================================================================

static unsigned long last_command_ms = 0;
static unsigned long last_telemetry_ms = 0;

// Convert a body velocity command into left/right motor PWM and drive the motors.
static void drive_from_velocity(const DriveCommand& cmd) {
  // Differential drive: split body (linear, angular) into per-wheel linear speeds.
  const float half_track = TRACK_WIDTH_M * 0.5f;
  float v_left = cmd.linear_velocity - cmd.angular_velocity * half_track;
  float v_right = cmd.linear_velocity + cmd.angular_velocity * half_track;
  // Map wheel speed (m/s) to the TB6612 PWM range (-255..255). Clamping happens in
  // motor_control_set_speed / drive_channel.
  int16_t pwm_left = (int16_t)(v_left / MAX_WHEEL_SPEED_MPS * 255.0f);
  int16_t pwm_right = (int16_t)(v_right / MAX_WHEEL_SPEED_MPS * 255.0f);
  motor_control_set_speed(pwm_left, pwm_right);
}

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
      drive_from_velocity(cmd);
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
