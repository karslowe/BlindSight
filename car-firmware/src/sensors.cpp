/*
 * sensors - read the Elegoo V4 onboard sensors into a CarTelemetry record.
 * See sensors.h for the interface contract.
 *
 * The V4's three line-tracking sensors (ITR20001) are ANALOG: read with analogRead and
 * threshold, not digitalRead. The ultrasonic is an HC-SR04 on the front servo.
 *
 * Pin values are the Elegoo Smart Car V4 defaults. The line pins (A0-A2) are well known;
 * VERIFY the ultrasonic Trig/Echo against the DeviceDriverSet_xxx0.h in your kit's tutorial
 * (they vary by revision). On the UNO Q (3.3V) the 5V echo line needs level shifting - see
 * the README.
 */
#include "sensors.h"

#include <Arduino.h>

// ===== Elegoo Smart Car V4 pin map (VERIFY against DeviceDriverSet_xxx0.h) =====
// Line tracking (ITR20001) - ANALOG sensors:
static const uint8_t PIN_LINE_LEFT = A2;
static const uint8_t PIN_LINE_CENTER = A1;
static const uint8_t PIN_LINE_RIGHT = A0;
// Ultrasonic HC-SR04 on the front servo. *** CONFIRM these two in your V4 code. ***
static const uint8_t PIN_ULTRASONIC_TRIG = 13;
static const uint8_t PIN_ULTRASONIC_ECHO = 12;
// Optional stall/bumper switch. Stock V4 has none; leave 0 to always report bumper=0.
static const uint8_t PIN_BUMPER = 0;

// Line sensor analog threshold (0..1023). Above it counts as "line/edge seen". Tune on the
// bench: print the raw analogRead values over your floor vs an edge and pick a midpoint.
static const int LINE_THRESHOLD = 500;
static const bool LINE_ABOVE_IS_ON = true;  // flip if your sensors read the other way
// Ultrasonic echo timeout (us). ~30000 us is roughly a 5 m max range.
static const unsigned long ECHO_TIMEOUT_US = 30000UL;
// ==============================================================================

void sensors_init() {
  pinMode(PIN_ULTRASONIC_TRIG, OUTPUT);
  pinMode(PIN_ULTRASONIC_ECHO, INPUT);
  // Analog line pins need no pinMode for analogRead().
  if (PIN_BUMPER != 0) {
    pinMode(PIN_BUMPER, INPUT_PULLUP);
  }
  digitalWrite(PIN_ULTRASONIC_TRIG, LOW);
}

// Trigger the ultrasonic and convert the echo to meters. Returns -1 if no echo (out of range).
static float read_ultrasonic_m() {
  digitalWrite(PIN_ULTRASONIC_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_ULTRASONIC_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_ULTRASONIC_TRIG, LOW);
  unsigned long dur = pulseIn(PIN_ULTRASONIC_ECHO, HIGH, ECHO_TIMEOUT_US);
  if (dur == 0) {
    return -1.0f;  // no echo within the timeout window
  }
  // Round-trip time -> distance: speed of sound 343 m/s, divide by 2, us -> s.
  return dur * 0.0001715f;
}

// Read one analog line sensor and threshold it to 0/1.
static int read_line(uint8_t pin) {
  int v = analogRead(pin);
  bool on = LINE_ABOVE_IS_ON ? (v > LINE_THRESHOLD) : (v < LINE_THRESHOLD);
  return on ? 1 : 0;
}

CarTelemetry sensors_read() {
  CarTelemetry tel;
  tel.ultrasonic_distance = read_ultrasonic_m();
  tel.bumper = (PIN_BUMPER != 0 && digitalRead(PIN_BUMPER) == LOW) ? 1 : 0;
  tel.line_left = read_line(PIN_LINE_LEFT);
  tel.line_center = read_line(PIN_LINE_CENTER);
  tel.line_right = read_line(PIN_LINE_RIGHT);
  tel.timestamp = millis() / 1000.0f;
  return tel;
}
