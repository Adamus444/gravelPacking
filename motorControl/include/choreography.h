#pragma once

#include <Arduino.h>


enum AxisId { A = 0, B = 1, C = 2, D = 3, E = 4 };
#define TIME_MULT 1.5 // so i dont have to rewrite the sequence

struct Move {
  uint32_t start_ms;   // motion begins, ms after this sequence's trigger
  uint32_t end_ms;     // motion ends (axis is at pos_mm), ms after trigger
  AxisId   axis;
  float    pos_mm;     // ABSOLUTE target position, 0 = power-up position
};

// PRE: power-up (0) -> base. The base is whatever this leaves each axis at.
constexpr Move PRE_SEQUENCE[] = {
  { 0, 2000, B, 30 },
  { 0, 2000, C, 30 },
  { 0, 2000, D, 30 },
  { 0, 2000, E, 30 },
};

// MAIN: starts at base, must end at base (rule 4) so it can repeat.
constexpr Move MAIN_SEQUENCE[] = {
    { (uint32_t)(0000*TIME_MULT),  (uint32_t)(6000*mult), A,  270 },
    { (uint32_t)(7000*TIME_MULT),  (uint32_t)(13000*mult), A,  0 },
    
    { (uint32_t)(8000*mult), (uint32_t)(9000*mult), B, -1},
    { (uint32_t)(8000*mult), (uint32_t)(9000*mult), C, -1},
    
    { (uint32_t)(11000*mult), (uint32_t)(12000*mult), B, 30},
    { (uint32_t)(11000*mult), (uint32_t)(12000*mult), C, 30},

    { (uint32_t)(6000*mult), (uint32_t)(7000*mult), D, -4},
    { (uint32_t)(6000*mult), (uint32_t)(7000*mult), E, 0},
    { (uint32_t)(13000*mult), (uint32_t)(14000*mult), D, 30},
    { (uint32_t)(13000*mult), (uint32_t)(14000*mult), E, 30},
};

// POST: base -> 0 on every axis (rule 4). Axes already at 0 need no row.
constexpr Move POST_SEQUENCE[] = {
  { 0, 2000, B, 0 },
  { 0, 2000, C, 0 },
  { 0, 2000, D, 0 },
  { 0, 2000, E, 0 },
};

constexpr int PRE_LEN  = sizeof(PRE_SEQUENCE)  / sizeof(PRE_SEQUENCE[0]);
constexpr int MAIN_LEN = sizeof(MAIN_SEQUENCE) / sizeof(MAIN_SEQUENCE[0]);
constexpr int POST_LEN = sizeof(POST_SEQUENCE) / sizeof(POST_SEQUENCE[0]);

// Sequence ids used throughout: 0 = PRE, 1 = MAIN, 2 = POST
constexpr const Move* rowsOf(int id) {
  return id == 0 ? PRE_SEQUENCE : id == 1 ? MAIN_SEQUENCE : POST_SEQUENCE;
}
constexpr int lenOf(int id) { return id == 0 ? PRE_LEN : id == 1 ? MAIN_LEN : POST_LEN; }
constexpr int MAX_LEN =
  (PRE_LEN > MAIN_LEN ? (PRE_LEN > POST_LEN ? PRE_LEN : POST_LEN)
                      : (MAIN_LEN > POST_LEN ? MAIN_LEN : POST_LEN));
