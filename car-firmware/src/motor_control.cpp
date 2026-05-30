/*
 * motor_control - stub implementation.
 * See motor_control.h for the interface contract.
 */
#include "motor_control.h"

#include <Arduino.h>

// TODO: set these to the actual TB6612 pin assignments on the Elegoo Smart Car V4.
// static const uint8_t PIN_AIN1 = 0;
// static const uint8_t PIN_AIN2 = 0;
// static const uint8_t PIN_PWMA = 0;
// static const uint8_t PIN_BIN1 = 0;
// static const uint8_t PIN_BIN2 = 0;
// static const uint8_t PIN_PWMB = 0;
// static const uint8_t PIN_STBY = 0;

void motor_control_init() {
  // TODO: pinMode() each driver pin to OUTPUT and pull STBY high to enable the driver.
}

void motor_control_set_speed(int16_t left, int16_t right) {
  // TODO: clamp to -255..255, set direction pins from the sign, and analogWrite the PWM.
  (void)left;
  (void)right;
}

void motor_control_stop() {
  // TODO: put both channels into the short-brake state (both IN pins high, PWM high).
}
