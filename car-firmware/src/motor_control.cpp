/*
 * motor_control - TB6612 motor driver control for the Elegoo Smart Car V4.
 * See motor_control.h for the interface contract.
 *
 * The logic is complete. You only need to fill in the hardware constants in the marked
 * block below from the V4 pinout, then calibrate the invert flags on the bench.
 */
#include "motor_control.h"

#include <Arduino.h>

// ===================== SET FROM THE SMART CAR V4 HARDWARE =====================
// These default to 0, which is WRONG (pin 0 is the serial RX line). The firmware will
// not drive anything until you set the real TB6612 pins from the V4 schematic or the
// Elegoo example sketch. Use PWM-capable pins (~ marked on the UNO) for PWMA/PWMB.
static const uint8_t PIN_AIN1 = 0;  // TODO: motor A direction 1
static const uint8_t PIN_AIN2 = 0;  // TODO: motor A direction 2
static const uint8_t PIN_PWMA = 0;  // TODO: motor A speed (PWM)
static const uint8_t PIN_BIN1 = 0;  // TODO: motor B direction 1
static const uint8_t PIN_BIN2 = 0;  // TODO: motor B direction 2
static const uint8_t PIN_PWMB = 0;  // TODO: motor B speed (PWM)
static const uint8_t PIN_STBY = 0;  // TODO: TB6612 standby (driven HIGH to enable)

// Bench calibration: if a wheel spins the wrong way, flip its invert flag.
static const bool LEFT_INVERT = false;
static const bool RIGHT_INVERT = false;
// Set true if motor A is actually the RIGHT wheel on your wiring (swaps the channels).
static const bool SWAP_LEFT_RIGHT = false;
// =============================================================================

// Drive one TB6612 channel from a signed speed (-255..255): sign = direction.
static void drive_channel(uint8_t in1, uint8_t in2, uint8_t pwm, int16_t speed, bool invert) {
  if (invert) {
    speed = -speed;
  }
  int16_t mag = (speed < 0) ? -speed : speed;
  if (mag > 255) {
    mag = 255;
  }
  bool forward = (speed >= 0);
  digitalWrite(in1, forward ? HIGH : LOW);
  digitalWrite(in2, forward ? LOW : HIGH);
  analogWrite(pwm, mag);
}

void motor_control_init() {
  pinMode(PIN_AIN1, OUTPUT);
  pinMode(PIN_AIN2, OUTPUT);
  pinMode(PIN_PWMA, OUTPUT);
  pinMode(PIN_BIN1, OUTPUT);
  pinMode(PIN_BIN2, OUTPUT);
  pinMode(PIN_PWMB, OUTPUT);
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
  // Channel A = left wheel, channel B = right wheel (adjust SWAP_LEFT_RIGHT if not).
  drive_channel(PIN_AIN1, PIN_AIN2, PIN_PWMA, left, LEFT_INVERT);
  drive_channel(PIN_BIN1, PIN_BIN2, PIN_PWMB, right, RIGHT_INVERT);
}

void motor_control_stop() {
  // Coast to a stop (PWM 0). The watchdog calls this when commands stop arriving.
  analogWrite(PIN_PWMA, 0);
  analogWrite(PIN_PWMB, 0);
}
