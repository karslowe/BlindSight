/*
 * motor_control - thin wrapper over the TB6612 motor driver on the Elegoo V4.
 *
 * Stub. Bodies live in motor_control.cpp with TODO markers.
 */
#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <stdint.h>

// Initialize motor driver pins. Call once from setup().
void motor_control_init();

/*
 * Set wheel speeds.
 *
 * Inputs:
 *   left, right: signed speed for each side, range -255..255 (TB6612 PWM, sign = direction).
 * Output: none. Drives the motors immediately.
 * TODO: map the signed range onto the TB6612 IN1/IN2/PWM pins for each channel.
 */
void motor_control_set_speed(int16_t left, int16_t right);

/*
 * Brake both motors immediately.
 * Inputs: none. Output: none.
 * TODO: drive both channels into the TB6612 short-brake state.
 */
void motor_control_stop();

#endif  // MOTOR_CONTROL_H
