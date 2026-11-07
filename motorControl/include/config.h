#pragma once
#include <Arduino.h>
// ---------------------------------------------------------------------
//      PIN MAP
// ---------------------------------------------------------------------
struct MotorPins { uint8_t step; uint8_t dir; };

const MotorPins MOTOR_PINS[5][2] = {
  /* A */ { {47, 46}, {45, 44} },
  /* B */ { {43, 42}, {41, 40} },
  /* C */ { {39, 38}, {37, 36} },
  /* D */ { {35, 34}, {33, 32} },
  /* E */ { {31, 30}, {29, 28} },
};
 -- edit to match your wiring
const uint8_t START_BUTTON_PIN = 2;   // wired to GND, uses INPUT_PULLUP
const char    PRE_BYTE   = 'P';       // serial: run PRE_SEQUENCE  (only accepted at 0)
const char    START_BYTE = 'S';       // serial / button: run MAIN_SEQUENCE (only at base)
const char    POST_BYTE  = 'Z';       // serial: run POST_SEQUENCE (only at base)

// ---------------------------------------------------------------------
// MECHANICAL CONFIG (all compile-time)
// ---------------------------------------------------------------------
constexpr float STEPS_PER_MM[5]   = { 13.333f, 13.333f, 13.333f, 13.333f, 13.333f };
constexpr float MAX_SPEED_MM_S[5] = { 100, 100, 100, 100, 100 };
// PLACEHOLDER - tune on the machine. Higher = shorter ramps but more stall
// risk. Rule 3 rejects moves too short for this accel.
constexpr float ACCEL_MM_S2[5]    = { 50, 50, 50, 50, 50 };

// Manual jog (serial 'J' command): cruise speed and per-command distance cap.
// Jogs use ACCEL_MM_S2 for their ramps and never change the tracked position.
constexpr float JOG_SPEED_MM_S = 10;   // keep modest: you are positioning by eye
constexpr float JOG_MAX_MM     = 100;  // reject typos like JA0 5000
constexpr uint32_t JOG_LINE_TIMEOUT_MS = 2000; // max gap between typed chars of a J line

// Per-motor direction sign, for mirrored mounting on a given axis.
const int8_t DIR_SIGN[5][2] = {
  { -1, 1 },
  { 1, -1 },
  { 1, -1 },
  { -1, -1 },
  { 1, -1 },
};
