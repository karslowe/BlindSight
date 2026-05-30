/*
 * sensors - stub implementation.
 * See sensors.h for the interface contract.
 */
#include "sensors.h"

#include <Arduino.h>

// TODO: set these to the actual pin assignments on the Elegoo Smart Car V4.
// static const uint8_t PIN_ULTRASONIC_TRIG = 0;
// static const uint8_t PIN_ULTRASONIC_ECHO = 0;
// static const uint8_t PIN_LINE_LEFT = 0;
// static const uint8_t PIN_LINE_CENTER = 0;
// static const uint8_t PIN_LINE_RIGHT = 0;

void sensors_init() {
  // TODO: pinMode the ultrasonic trig (OUTPUT), echo (INPUT), and line pins (INPUT).
}

CarTelemetry sensors_read() {
  CarTelemetry tel;
  // TODO: trigger the ultrasonic, measure echo with pulseIn, convert to meters.
  tel.ultrasonic_distance = -1.0f;  // placeholder: no echo
  // TODO: read bumper/stall and the three line sensors (digital or thresholded analog).
  tel.bumper = 0;
  tel.line_left = 0;
  tel.line_center = 0;
  tel.line_right = 0;
  tel.timestamp = millis() / 1000.0f;
  return tel;
}
