/*
 * sensors - read the Elegoo V4 onboard sensors into a CarTelemetry record.
 *
 * The car has a servo-mounted ultrasonic sensor and line sensors. It has no wheel
 * encoders, so this firmware produces no odometry.
 *
 * Stub. Bodies live in sensors.cpp with TODO markers.
 */
#ifndef SENSORS_H
#define SENSORS_H

#include "serial_protocol.h"  // for CarTelemetry

// Initialize sensor pins (ultrasonic trigger/echo, line sensor inputs). Call from setup().
void sensors_init();

/*
 * Read all sensors once and pack them into a CarTelemetry record.
 *
 * Inputs: none.
 * Output: a filled CarTelemetry with ultrasonic_distance in meters (-1 if no echo),
 *         bumper and the three line flags as 0/1, and timestamp in seconds.
 * TODO: trigger the ultrasonic and convert echo time to meters; read the line sensors;
 *       set timestamp from millis()/1000.0.
 */
CarTelemetry sensors_read();

#endif  // SENSORS_H
