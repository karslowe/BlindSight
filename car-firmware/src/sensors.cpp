/*
 * sensors - read the Elegoo V4 onboard sensors into a CarTelemetry record.
 * See sensors.h for the interface contract.
 *
 * The logic is complete. Fill in the pin constants below from the V4 pinout, then check
 * the LINE_ACTIVE_HIGH polarity on the bench (drive over a line and watch the flags).
 */
#include "sensors.h"

#include <Arduino.h>

// ===================== SET FROM THE SMART CAR V4 HARDWARE =====================
// Default 0 is WRONG (pin 0 is serial RX). Set the real pins before flashing.
static const uint8_t PIN_ULTRASONIC_TRIG = 0;  // TODO
static const uint8_t PIN_ULTRASONIC_ECHO = 0;  // TODO
static const uint8_t PIN_LINE_LEFT = 0;        // TODO
static const uint8_t PIN_LINE_CENTER = 0;      // TODO
static const uint8_t PIN_LINE_RIGHT = 0;       // TODO
// Optional stall/bumper switch. The stock V4 has none; leave 0 to always report bumper=0.
static const uint8_t PIN_BUMPER = 0;           // 0 = not wired

// Polarity: set false if the line sensors read LOW (instead of HIGH) when over a line.
static const bool LINE_ACTIVE_HIGH = true;
// Ultrasonic echo timeout in microseconds. ~30000 us is roughly a 5 m max range.
static const unsigned long ECHO_TIMEOUT_US = 30000UL;
// =============================================================================

void sensors_init() {
  pinMode(PIN_ULTRASONIC_TRIG, OUTPUT);
  pinMode(PIN_ULTRASONIC_ECHO, INPUT);
  pinMode(PIN_LINE_LEFT, INPUT);
  pinMode(PIN_LINE_CENTER, INPUT);
  pinMode(PIN_LINE_RIGHT, INPUT);
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

static int read_line(uint8_t pin) {
  int v = digitalRead(pin);
  bool on = LINE_ACTIVE_HIGH ? (v == HIGH) : (v == LOW);
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
