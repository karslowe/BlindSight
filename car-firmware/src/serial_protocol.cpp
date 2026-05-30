/*
 * serial_protocol - line-based ASCII contract with the brain.
 * See serial_protocol.h for the interface contract.
 *
 * This file is fully implemented and hardware-independent (pure string handling), so it
 * needs no pin numbers or calibration. It mirrors DriveCommand.from_serial_line and
 * CarTelemetry.to_serial_line on the Python side: identical wire format, both ends parse.
 *
 * Note: AVR's sscanf does not support %f by default, so the DRV numbers are parsed with
 * strtok_r + atof rather than sscanf, which works on the UNO out of the box.
 */
#include "serial_protocol.h"

#include <Arduino.h>
#include <stdlib.h>  // atof, atoi
#include <string.h>  // strtok_r, strcmp

// Line accumulation buffer for incoming DRV commands.
static const size_t LINE_MAX = 64;
static char line_buf[LINE_MAX];
static size_t line_len = 0;

// Parse one complete, null-terminated line. Returns true and fills *out on a valid
// "DRV <linear> <angular> [stop]" line. NOTE: this modifies buf (strtok_r).
static bool parse_drv(char* buf, DriveCommand* out) {
  char* save = NULL;
  char* cmd = strtok_r(buf, " \t\r\n", &save);
  if (cmd == NULL || strcmp(cmd, "DRV") != 0) {
    return false;
  }
  char* a = strtok_r(NULL, " \t\r\n", &save);
  char* b = strtok_r(NULL, " \t\r\n", &save);
  char* c = strtok_r(NULL, " \t\r\n", &save);
  if (a == NULL || b == NULL) {
    return false;  // need at least linear and angular
  }
  out->linear_velocity = atof(a);
  out->angular_velocity = atof(b);
  out->stop = (c != NULL) ? atoi(c) : 0;
  return true;
}

bool serial_protocol_poll(DriveCommand* out) {
  while (Serial.available() > 0) {
    char ch = (char)Serial.read();
    if (ch == '\n') {
      line_buf[line_len] = '\0';
      size_t had = line_len;
      line_len = 0;
      if (had > 0 && parse_drv(line_buf, out)) {
        return true;  // a valid DRV command this call
      }
      // Not a valid line: drop it and keep draining any further buffered bytes.
    } else if (ch != '\r') {
      if (line_len < LINE_MAX - 1) {
        line_buf[line_len++] = ch;
      } else {
        line_len = 0;  // overflow guard: discard the oversized line
      }
    }
  }
  return false;
}

void serial_protocol_send_telemetry(const CarTelemetry* tel) {
  // "TEL <ultrasonic> <bumper> <line_left> <line_center> <line_right> <timestamp>\n"
  // Floats use Serial.print(value, digits) because AVR printf lacks %f by default.
  Serial.print("TEL ");
  Serial.print(tel->ultrasonic_distance, 4);
  Serial.print(' ');
  Serial.print(tel->bumper);
  Serial.print(' ');
  Serial.print(tel->line_left);
  Serial.print(' ');
  Serial.print(tel->line_center);
  Serial.print(' ');
  Serial.print(tel->line_right);
  Serial.print(' ');
  Serial.print(tel->timestamp, 4);
  Serial.print('\n');
}
