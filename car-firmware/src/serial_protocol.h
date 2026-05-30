/*
 * serial_protocol - the line-based ASCII contract with the brain.
 *
 * Contract: ../../docs/message-schemas.md
 *   Consumes: "DRV <linear_velocity> <angular_velocity> <stop>"
 *   Produces: "TEL <ultrasonic_distance> <bumper> <line_left> <line_center> <line_right> <timestamp>"
 *
 * Stub. Bodies live in serial_protocol.cpp with TODO markers.
 */
#ifndef SERIAL_PROTOCOL_H
#define SERIAL_PROTOCOL_H

#include <stdbool.h>

// Parsed DriveCommand. Field names match the schema exactly.
struct DriveCommand {
  float linear_velocity;   // m/s, forward positive
  float angular_velocity;  // rad/s, CCW positive
  int   stop;              // 0 or 1; 1 brakes and overrides the velocities
};

// CarTelemetry to send back. Field names match the schema exactly.
struct CarTelemetry {
  float ultrasonic_distance;  // m, -1 if no echo
  int   bumper;               // 0 or 1
  int   line_left;            // 0 or 1
  int   line_center;          // 0 or 1
  int   line_right;           // 0 or 1
  float timestamp;            // s, car-side time
};

/*
 * Non-blocking poll for a complete DriveCommand line.
 *
 * Inputs:
 *   out: pointer to a DriveCommand to fill.
 * Output:
 *   returns true and fills *out when a full valid "DRV ..." line was read this call,
 *   false otherwise (no complete line yet, or a malformed line that was discarded).
 * TODO: accumulate Serial bytes until '\n', then sscanf the "DRV" line into *out.
 */
bool serial_protocol_poll(DriveCommand* out);

/*
 * Format and write a telemetry record as a "TEL ..." line plus newline.
 * Inputs: tel: pointer to the record to send. Output: none.
 * TODO: print the fields in schema order, space separated, terminated by '\n'.
 */
void serial_protocol_send_telemetry(const CarTelemetry* tel);

#endif  // SERIAL_PROTOCOL_H
