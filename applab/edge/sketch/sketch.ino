/*
 * BlindSight — UNO Q MCU sketch (App Lab Bridge side of the car link).
 *
 * Runs on the UNO Q's microcontroller. The Python brain (python/main.py) sends drive
 * commands over the App Lab Bridge; this sketch drives the Elegoo Smart Car V4 motors.
 *
 *   Python -> sketch:  Bridge.call("drive", linear_velocity, angular_velocity, stop)
 *
 * Bridge API confirmed against the Arduino examples (color-your-leds): Bridge.begin() in
 * setup(), Bridge.provide(name, fn) exposes a function to Python.
 *
 * NO TELEMETRY: the ultrasonic was removed (no time to level-shift its 5 V echo to the
 * UNO Q's 3.3 V). The car's other onboard sensors (ITR20001 line trackers) share that 5 V
 * issue and aren't used by the brain, so the whole sketch->brain telemetry path is dropped.
 * Re-add it with `Bridge.notify("car_telemetry", ...)` + level-shifting if needed later.
 *
 * PINS + MOTOR TOPOLOGY are the Elegoo Smart Car V4 defaults from BlindSight/car-firmware
 * (the V4 uses ONE direction pin per motor + a PWM pin + a shared STBY). Still needs a bench
 * pass (TASK C): confirm a "drive" command spins the wheels the right way (flip *_INVERT /
 * SWAP below); TB6612 logic inputs are fine at the UNO Q's 3.3 V.
 */

#include <Arduino_RouterBridge.h>

// ---- calibration (measure on the real Smart Car V4) -------------------------------
static const float TRACK_WIDTH_M   = 0.15f;  // distance between drive wheels (m) — measure
static const float MAX_WHEEL_SPEED = 0.60f;  // wheel m/s at full PWM (255) — calibrate

// ---- safety -----------------------------------------------------------------------
static const unsigned long COMMAND_TIMEOUT_MS = 500;  // brake if no drive cmd within this

// ---- Elegoo Smart Car V4 pin map (from BlindSight/car-firmware) --------------------
static const int PIN_PWMA = 5;   // left motor speed (PWM)
static const int PIN_DIRA = 7;   // left motor direction (HIGH = forward)
static const int PIN_PWMB = 6;   // right motor speed (PWM)
static const int PIN_DIRB = 8;   // right motor direction (HIGH = forward)
static const int PIN_STBY = 3;   // TB6612 standby (HIGH = enabled)
// Flip if a wheel runs backwards / channels are swapped (bench calibration).
static const bool LEFT_INVERT = false, RIGHT_INVERT = false, SWAP_LEFT_RIGHT = false;

static unsigned long last_command_ms = 0;

// ---- motor HAL (TB6612, single direction pin per motor) ---------------------------
static void drive_channel(int dir_pin, int pwm_pin, int16_t speed, bool invert) {
  if (invert) speed = -speed;
  int mag = abs(speed); if (mag > 255) mag = 255;
  digitalWrite(dir_pin, speed >= 0 ? HIGH : LOW);
  analogWrite(pwm_pin, mag);
}

static int16_t g_last_left = 0, g_last_right = 0;  // for the serial monitor debug

static void motors_set(int16_t left, int16_t right) {
  if (SWAP_LEFT_RIGHT) { int16_t t = left; left = right; right = t; }
  digitalWrite(PIN_STBY, HIGH);
  drive_channel(PIN_DIRA, PIN_PWMA, left, LEFT_INVERT);
  drive_channel(PIN_DIRB, PIN_PWMB, right, RIGHT_INVERT);
  g_last_left = left; g_last_right = right;
}

static void motors_stop() {
  analogWrite(PIN_PWMA, 0);
  analogWrite(PIN_PWMB, 0);
}

// Differential drive: body (linear, angular) -> per-wheel PWM (from car-firmware.ino).
static void drive(float linear_velocity, float angular_velocity, int stop) {
  last_command_ms = millis();
  if (stop) { motors_stop(); Monitor.println("[drive] STOP"); return; }
  float half = TRACK_WIDTH_M * 0.5f;
  float v_left  = linear_velocity - angular_velocity * half;
  float v_right = linear_velocity + angular_velocity * half;
  motors_set((int16_t)(v_left  / MAX_WHEEL_SPEED * 255.0f),
             (int16_t)(v_right / MAX_WHEEL_SPEED * 255.0f));
  // DEBUG: confirms the MCU received the Bridge call + the PWM it applied. Watch with
  // `arduino-app-cli monitor`. Remove once the car drives.
  Monitor.print("[drive] v="); Monitor.print(linear_velocity, 3);
  Monitor.print(" w="); Monitor.print(angular_velocity, 3);
  Monitor.print(" -> PWM L="); Monitor.print(g_last_left);
  Monitor.print(" R="); Monitor.println(g_last_right);
}

void setup() {
  pinMode(PIN_PWMA, OUTPUT); pinMode(PIN_DIRA, OUTPUT);
  pinMode(PIN_PWMB, OUTPUT); pinMode(PIN_DIRB, OUTPUT);
  pinMode(PIN_STBY, OUTPUT);
  motors_stop();

  Monitor.begin(115200);
  Bridge.begin();
  Bridge.provide("drive", drive);  // Python: Bridge.call("drive", v, w, stop)
  Monitor.println("[sketch] BlindSight drive endpoint up; waiting for drive commands");
}

void loop() {
  // Safety watchdog: brake if the brain has gone quiet (lost connection / crash).
  // DEBUG: announce the brake once per stretch so a quiet Bridge is visible on the monitor.
  static bool braking = false;
  if (millis() - last_command_ms > COMMAND_TIMEOUT_MS) {
    motors_stop();
    if (!braking) { Monitor.println("[watchdog] no drive command in 500ms — braking"); braking = true; }
  } else {
    braking = false;
  }
}
