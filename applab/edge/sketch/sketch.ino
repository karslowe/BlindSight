/*
 * BlindSight — UNO Q MCU sketch (App Lab Bridge side of the car link).
 *
 * Runs on the UNO Q's microcontroller. Replaces the old USB-serial car link: the Python
 * brain (python/main.py) sends drive commands over the App Lab Bridge, and this sketch
 * drives the Elegoo Smart Car V4 motors and streams telemetry back.
 *
 *   Python -> sketch:  Bridge.call("drive", linear_velocity, angular_velocity, stop)
 *   sketch -> Python:  Bridge.notify("car_telemetry", ultrasonic, bumper, ll, lc, lr, t)
 *
 * Bridge API confirmed against the Arduino examples (color-your-leds, real-time-
 * accelerometer): Bridge.begin() in setup(), Bridge.provide(name, fn) to expose a function
 * to Python, Bridge.notify(name, args...) to push data to a Python-provided handler.
 *
 * PINS + MOTOR TOPOLOGY are the Elegoo Smart Car V4 defaults, taken from
 * BlindSight/car-firmware (which mirrors Elegoo's DeviceDriverSet). The V4 uses ONE
 * direction pin per motor (not IN1+IN2) + a PWM pin + a shared STBY.
 *
 * ⚠️ STILL NEEDS A BENCH PASS (TASK C), because these were written for the R3 (5 V):
 *   - The UNO Q is 3.3 V logic — the 5 V ultrasonic ECHO line needs level-shifting
 *     (see car-firmware/README.md). TB6612 logic inputs are fine at 3.3 V.
 *   - Verify Trig/Echo against your kit's DeviceDriverSet_xxx0.h (they vary by revision).
 *   - Confirm a "drive" command spins the wheels the right way (flip *_INVERT / SWAP below).
 */

#include <Arduino_RouterBridge.h>

// ---- calibration (measure on the real Smart Car V4) -------------------------------
static const float TRACK_WIDTH_M   = 0.15f;  // distance between drive wheels (m) — measure
static const float MAX_WHEEL_SPEED = 0.60f;  // wheel m/s at full PWM (255) — calibrate

// ---- safety / cadence -------------------------------------------------------------
static const unsigned long COMMAND_TIMEOUT_MS  = 500;  // brake if no drive cmd within this
static const unsigned long TELEMETRY_PERIOD_MS = 50;   // telemetry push cadence (20 Hz)

// ---- Elegoo Smart Car V4 pin map (from BlindSight/car-firmware) --------------------
static const int PIN_PWMA = 5;   // left motor speed (PWM)
static const int PIN_DIRA = 7;   // left motor direction (HIGH = forward)
static const int PIN_PWMB = 6;   // right motor speed (PWM)
static const int PIN_DIRB = 8;   // right motor direction (HIGH = forward)
static const int PIN_STBY = 3;   // TB6612 standby (HIGH = enabled)
static const int PIN_US_TRIG = 13;
static const int PIN_US_ECHO = 12;            // ⚠️ 5 V — level-shift on the UNO Q
static const int PIN_LINE_LEFT = A2, PIN_LINE_CENTER = A1, PIN_LINE_RIGHT = A0;  // analog
static const int LINE_THRESHOLD = 500;        // analogRead above this = line/edge seen
// Flip if a wheel runs backwards / channels are swapped (bench calibration).
static const bool LEFT_INVERT = false, RIGHT_INVERT = false, SWAP_LEFT_RIGHT = false;

static unsigned long last_command_ms = 0;
static unsigned long last_telemetry_ms = 0;

// ---- motor HAL (TB6612, single direction pin per motor) ---------------------------
static void drive_channel(int dir_pin, int pwm_pin, int16_t speed, bool invert) {
  if (invert) speed = -speed;
  int mag = abs(speed); if (mag > 255) mag = 255;
  digitalWrite(dir_pin, speed >= 0 ? HIGH : LOW);
  analogWrite(pwm_pin, mag);
}

static void motors_set(int16_t left, int16_t right) {
  if (SWAP_LEFT_RIGHT) { int16_t t = left; left = right; right = t; }
  digitalWrite(PIN_STBY, HIGH);
  drive_channel(PIN_DIRA, PIN_PWMA, left, LEFT_INVERT);
  drive_channel(PIN_DIRB, PIN_PWMB, right, RIGHT_INVERT);
}

static void motors_stop() {
  analogWrite(PIN_PWMA, 0);
  analogWrite(PIN_PWMB, 0);
}

// Differential drive: body (linear, angular) -> per-wheel PWM (from car-firmware.ino).
static void drive(float linear_velocity, float angular_velocity, int stop) {
  last_command_ms = millis();
  if (stop) { motors_stop(); return; }
  float half = TRACK_WIDTH_M * 0.5f;
  float v_left  = linear_velocity - angular_velocity * half;
  float v_right = linear_velocity + angular_velocity * half;
  motors_set((int16_t)(v_left  / MAX_WHEEL_SPEED * 255.0f),
             (int16_t)(v_right / MAX_WHEEL_SPEED * 255.0f));
}

// ---- sensor HAL -------------------------------------------------------------------
static float read_ultrasonic() {  // meters; -1 on no echo
  digitalWrite(PIN_US_TRIG, LOW);  delayMicroseconds(2);
  digitalWrite(PIN_US_TRIG, HIGH); delayMicroseconds(10);
  digitalWrite(PIN_US_TRIG, LOW);
  unsigned long us = pulseIn(PIN_US_ECHO, HIGH, 30000UL);  // ~5 m timeout
  if (us == 0) return -1.0f;
  return us * 0.0001715f;  // round trip, 343 m/s
}

static int read_line(int pin) { return analogRead(pin) > LINE_THRESHOLD ? 1 : 0; }

void setup() {
  pinMode(PIN_PWMA, OUTPUT); pinMode(PIN_DIRA, OUTPUT);
  pinMode(PIN_PWMB, OUTPUT); pinMode(PIN_DIRB, OUTPUT);
  pinMode(PIN_STBY, OUTPUT);
  pinMode(PIN_US_TRIG, OUTPUT); pinMode(PIN_US_ECHO, INPUT);
  digitalWrite(PIN_US_TRIG, LOW);
  motors_stop();

  Bridge.begin();
  Bridge.provide("drive", drive);  // Python: Bridge.call("drive", v, w, stop)
}

void loop() {
  // Safety watchdog: brake if the brain has gone quiet (lost connection / crash).
  if (millis() - last_command_ms > COMMAND_TIMEOUT_MS) {
    motors_stop();
  }

  // Stream telemetry to the brain on a fixed cadence.
  unsigned long now = millis();
  if (now - last_telemetry_ms >= TELEMETRY_PERIOD_MS) {
    last_telemetry_ms = now;
    Bridge.notify("car_telemetry", read_ultrasonic(), 0,
                  read_line(PIN_LINE_LEFT), read_line(PIN_LINE_CENTER), read_line(PIN_LINE_RIGHT),
                  (float)(now / 1000.0));
  }
}
