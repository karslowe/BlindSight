/*
 * sensors - read the Elegoo V4 onboard sensors into a CarTelemetry record.
 * See sensors.h for the interface contract.
 *
 * Sensing is now phone (LiDAR) + side ToF only. The ULTRASONIC and LINE sensors are no
 * longer read. The CarTelemetry message keeps their fields (contract unchanged) but they
 * report sentinels: ultrasonic_distance = -1 (no echo), line_* = 0. Only the bumper/stall
 * flag and timestamp are produced. To bring those sensors back, restore the reads from git
 * history.
 */
#include "sensors.h"

#include <Arduino.h>

// Optional stall/bumper switch. Stock V4 has none; leave 0 to always report bumper=0.
static const uint8_t PIN_BUMPER = 0;

void sensors_init() {
  if (PIN_BUMPER != 0) {
    pinMode(PIN_BUMPER, INPUT_PULLUP);
  }
}

CarTelemetry sensors_read() {
  CarTelemetry tel;
  // Ultrasonic + line sensors removed for now: report sentinels (message shape unchanged).
  tel.ultrasonic_distance = -1.0f;  // no echo / sensor not present
  tel.line_left = 0;
  tel.line_center = 0;
  tel.line_right = 0;
  tel.bumper = (PIN_BUMPER != 0 && digitalRead(PIN_BUMPER) == LOW) ? 1 : 0;
  tel.timestamp = millis() / 1000.0f;
  return tel;
}
