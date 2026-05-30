/*
 * motor_control - TB6612 motor driver control for the Elegoo Smart Car V4.
 * See motor_control.h for the interface contract.
 *
 * IMPORTANT: the V4 wires the TB6612 with ONE direction pin per motor (not IN1+IN2). Each
 * motor: a direction pin (HIGH = forward / LOW = reverse) + a PWM speed pin, plus a shared
 * STBY enable. This matches Elegoo's own DeviceDriverSet code.
 *
 * The pin values below are the Elegoo Smart Car V4 defaults. VERIFY them against the
 * DeviceDriverSet_xxx0.h in your kit's tutorial download before flashing, and note the
 * voltage caveat in the README if you run this on the UNO Q (3.3V) instead of an R3 (5V).
 */
#include "motor_control.h"

#include <Arduino.h>

// ===== Elegoo Smart Car V4 pin map (VERIFY against DeviceDriverSet_xxx0.h) =====
static const uint8_t PIN_PWMA = 5;  // left motor speed (PWM)
static const uint8_t PIN_AIN1 = 7;  // left motor direction
static const uint8_t PIN_PWMB = 6;  // right motor speed (PWM)
static const uint8_t PIN_BIN1 = 8;  // right motor direction
static const uint8_t PIN_STBY = 3;  // TB6612 standby (driven HIGH to enable)

// Bench calibration: flip if a wheel spins the wrong way; swap if the channels are reversed.
static const bool LEFT_INVERT = false;
static const bool RIGHT_INVERT = false;
static const bool SWAP_LEFT_RIGHT = false;
// ==============================================================================

// Drive one motor from a signed speed (-255..255): sign = direction via the single dir pin.
static void drive_channel(uint8_t dir_pin, uint8_t pwm_pin, int16_t speed, bool invert) {
  if (invert) {
    speed = -speed;
  }
  int16_t mag = (speed < 0) ? -speed : speed;
  if (mag > 255) {
    mag = 255;
  }
  digitalWrite(dir_pin, (speed >= 0) ? HIGH : LOW);  // HIGH = forward (flip the invert flag if not)
  analogWrite(pwm_pin, mag);
}

void motor_control_init() {
  pinMode(PIN_PWMA, OUTPUT);
  pinMode(PIN_AIN1, OUTPUT);
  pinMode(PIN_PWMB, OUTPUT);
  pinMode(PIN_BIN1, OUTPUT);
  pinMode(PIN_STBY, OUTPUT);
  digitalWrite(PIN_STBY, HIGH);  // enable the driver
  motor_control_stop();
}

void motor_control_set_speed(int16_t left, int16_t right) {
  if (SWAP_LEFT_RIGHT) {
    int16_t tmp = left;
    left = right;
    right = tmp;
  }
  drive_channel(PIN_AIN1, PIN_PWMA, left, LEFT_INVERT);
  drive_channel(PIN_BIN1, PIN_PWMB, right, RIGHT_INVERT);
}

void motor_control_stop() {
  // Coast to a stop (PWM 0). The watchdog calls this when commands stop arriving.
  analogWrite(PIN_PWMA, 0);
  analogWrite(PIN_PWMB, 0);
}
