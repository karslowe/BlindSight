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

#include <Servo.h>

#include "src/motor_control.h"
#include "src/serial_protocol.h"
#include "src/sensors.h"

// Serial baud to the brain (UNO Q).
static const unsigned long SERIAL_BAUD = 115200;

// Safety watchdog: brake if no DriveCommand has arrived within this many ms.
static const unsigned long COMMAND_TIMEOUT_MS = 500;

// How often to publish telemetry back to the brain.
static const unsigned long TELEMETRY_PERIOD_MS = 50;

// Ultrasonic safety reflex: if the forward-facing ultrasonic reads closer than this, kill
// the forward part of the motion. This runs ON THE CAR MCU, independent of the brain and
// the phone, so it still protects the rover if the perception pipeline stalls.
static const float SAFE_STOP_DISTANCE_M = 0.20f;
// Speed above which a command counts as "driving forward". Turning in place (near-zero
// linear) is left alone so the rover can still pivot away from an obstacle ahead.
static const float FORWARD_EPS_MPS = 0.02f;

// The Elegoo V4 ultrasonic sits on a pan servo (pin 10). We home it forward once and never
// sweep it, so the range reading is always the straight-ahead direction.
static const uint8_t PIN_ULTRASONIC_SERVO = 10;
static Servo ultrasonic_servo;

// ===================== SET FROM THE SMART CAR V4 (calibration) =====================
// Distance between the two drive wheels, in meters. Measure it with a ruler.
static const float TRACK_WIDTH_M = 0.15f;        // TODO: measure
// Wheel linear speed at full PWM (255), in meters/second. Calibrate by driving at full
// speed over a known distance and timing it, then v = distance / time.
static const float MAX_WHEEL_SPEED_MPS = 0.6f;   // TODO: calibrate
// ==================================================================================

static unsigned long last_command_ms = 0;
static unsigned long last_telemetry_ms = 0;
static DriveCommand last_cmd = {0.0f, 0.0f, 1};  // most recent command; start braked
static float last_ultrasonic_m = -1.0f;          // cached forward range; <0 = no echo/unknown

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
  // Home the ultrasonic forward (90 deg = straight ahead) and leave it there, so its range
  // reading is always the forward direction the reflex below assumes.
  ultrasonic_servo.attach(PIN_ULTRASONIC_SERVO);
  ultrasonic_servo.write(90);
}

void loop() {
  // 1. Parse any incoming DriveCommand line and remember it (we apply it below, after the
  //    safety overrides, so the reflex can veto it).
  DriveCommand cmd;
  if (serial_protocol_poll(&cmd)) {
    last_cmd = cmd;
    last_command_ms = millis();
  }

  // 2. Refresh telemetry on a fixed cadence and cache the forward range for the reflex.
  if (millis() - last_telemetry_ms >= TELEMETRY_PERIOD_MS) {
    last_telemetry_ms = millis();
    CarTelemetry tel = sensors_read();
    last_ultrasonic_m = tel.ultrasonic_distance;
    serial_protocol_send_telemetry(&tel);
  }

  // 3. Apply motors from the latest command plus two independent safety overrides:
  const bool watchdog_expired = (millis() - last_command_ms > COMMAND_TIMEOUT_MS);
  const bool obstacle_ahead =
      (last_ultrasonic_m > 0.0f && last_ultrasonic_m < SAFE_STOP_DISTANCE_M);

  if (watchdog_expired || last_cmd.stop) {
    // Brain went quiet, or it explicitly asked to brake.
    motor_control_stop();
  } else if (obstacle_ahead && last_cmd.linear_velocity > FORWARD_EPS_MPS) {
    // Ultrasonic reflex: something is close ahead. Cancel the forward motion but keep any
    // turn, so the rover can still pivot away while the brain re-plans from the LiDAR map.
    DriveCommand evade = last_cmd;
    evade.linear_velocity = 0.0f;
    drive_from_velocity(evade);
  } else {
    drive_from_velocity(last_cmd);
  }
}
