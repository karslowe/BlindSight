/*
 * serial_protocol - stub implementation.
 * See serial_protocol.h for the interface contract.
 */
#include "serial_protocol.h"

#include <Arduino.h>

// Line accumulation buffer for incoming DRV commands.
static const size_t LINE_MAX = 64;
static char line_buf[LINE_MAX];
static size_t line_len = 0;

bool serial_protocol_poll(DriveCommand* out) {
  // TODO: read available Serial bytes into line_buf until '\n'. On a full line,
  //       sscanf(line_buf, "DRV %f %f %d", &out->linear_velocity,
  //              &out->angular_velocity, &out->stop) and return true on a match.
  //       Reset line_len after each newline. Guard against overflow of LINE_MAX.
  (void)out;
  (void)line_buf;
  (void)line_len;
  return false;
}

void serial_protocol_send_telemetry(const CarTelemetry* tel) {
  // TODO: emit "TEL <ultrasonic_distance> <bumper> <line_left> <line_center>
  //       <line_right> <timestamp>\n" using Serial.print of each field in order.
  (void)tel;
}
