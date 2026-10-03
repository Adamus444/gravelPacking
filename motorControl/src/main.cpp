/*
 * Fixed choreography sequencer, ISR-driven stepping (grbl-style).
 * Arduino Mega 2560, 5 dual-motor axes (A-E) on TB6600 drivers.
 *
 * Design:
 *  - Step/dir pins are resolved to raw AVR port register + bitmask ONCE at
 *    setup(). The hot path uses direct register writes (*reg |= mask),
 *    not digitalWrite(), which is the main source of loop-speed limits.
 *  - All step pulses are generated inside a Timer1 Compare-Match ISR that
 *    fires at a fixed rate (ISR_FREQ_HZ). Each axis has its own Bresenham/
 *    DDA accumulator so multiple axes can run at different speeds off the
 *    same shared timer tick - this is the same mechanism grbl uses.
 *  - loop() never touches a motor pin. It only (a) waits for a start
 *    trigger (serial byte or button), then (b) compares millis() against
 *    the compile-time SEQUENCE[] table to decide when to arm a move.
 *    Because stepping lives entirely in the ISR, loop() can spend
 *    arbitrary time on serial parsing without dropping or slowing steps.
 *
 * No AccelStepper/MultiStepper dependency - this file is self-contained
 * beyond avr/io.h, which Arduino.h already pulls in.
 *
 * NOTE: constant speed per move, no accel/decel ramp. Fine for light
 * loads / short moves; if motors stall on start, ask for a trapezoidal
 * ramp version - same accumulator idea, with stepRate changing over time.
 */

#include <Arduino.h>

// ---------------------------------------------------------------------
// 1. PIN MAP -- edit to match your wiring
// ---------------------------------------------------------------------
struct MotorPins { uint8_t step; uint8_t dir; };

const MotorPins MOTOR_PINS[5][2] = {
  /* A */ { {47, 46}, {45, 44} },
  /* B */ { {43, 42}, {41, 40} },
  /* C */ { {39, 38}, {37, 36} },
  /* D */ { {35, 34}, {33, 32} },
  /* E */ { {31, 30}, {29, 28} },
};

const uint8_t START_BUTTON_PIN = 2;   // wired to GND, uses INPUT_PULLUP
const char    START_BYTE = 'S';       // any serial byte equal to this starts the sequence

// ---------------------------------------------------------------------
// 2. MECHANICAL CONFIG
// ---------------------------------------------------------------------
const float STEPS_PER_MM[5]    = { 13.333, 13.333, 13.333, 13.333, 13.333 };
const float MAX_SPEED_MM_S[5]  = { 100, 100, 100, 100, 100 };

// Per-motor direction sign, for mirrored mounting on a given axis.
const int8_t DIR_SIGN[5][2] = {
  { -1, 1 },
  { 1, 1 },
  { 1, 1 },
  { -1, -1 },
  { -1, -1 },
};

// ---------------------------------------------------------------------
// 3. CHOREOGRAPHY -- edit this table for a new routine
// ---------------------------------------------------------------------
enum AxisId { A = 0, B = 1, C = 2, D = 3, E = 4 };

struct Move {
  uint32_t time_ms;     // offset from trigger, ms
  AxisId   axis;
  float    distance_mm; // signed, relative
  float    speed_mm_s;
};

const Move SEQUENCE[] = {
  { 0, A, 100.0f, 10.0f },
  { 1000, B, 50, 25},
  { 1000, D, 50, 25},
  { 10000, B, -50, 25},
  { 10000, D, -50, 25},

  { 1000, C, 50, 25},
  { 1000, E, 50, 25},
  { 10000, C, -50, 25},
  { 10000, E, -50, 25},
  { 15000, A, -100.0f, 10.0f },
};
const size_t SEQUENCE_LEN = sizeof(SEQUENCE) / sizeof(SEQUENCE[0]);

// ---------------------------------------------------------------------
// 4. TIMER / ISR CONFIG
// ---------------------------------------------------------------------
// ISR fires twice per step (once to raise STEP, once to lower it), so
// max achievable step rate per axis is ISR_FREQ_HZ / 2. Keep this well
// above MAX_SPEED_MM_S * STEPS_PER_MM for every axis.
const uint32_t ISR_FREQ_HZ = 20000; // 50us tick -> 10kHz max step rate/axis

// ---------------------------------------------------------------------
// 5. RUNTIME STATE
// ---------------------------------------------------------------------
volatile uint8_t *stepReg[5][2];
uint8_t stepMask[5][2];
volatile uint8_t *dirReg[5][2];
uint8_t dirMask[5][2];

struct AxisState {
  volatile int32_t  stepsRemaining;
  volatile int32_t  accum;
  volatile uint16_t stepRateInt; // steps/sec, integer
  volatile bool     pulseHigh;
  volatile bool     active;
};
volatile AxisState axisState[5];

bool moveTriggered[SEQUENCE_LEN];
bool sequenceStarted = false;
uint32_t sequenceStart = 0;

// ---------------------------------------------------------------------
// Fast pin resolution: bind once at setup(), use raw register writes after.
// ---------------------------------------------------------------------
void resolveFastPin(uint8_t pin, volatile uint8_t *&reg, uint8_t &mask) {
  reg  = portOutputRegister(digitalPinToPort(pin));
  mask = digitalPinToBitMask(pin);
}

void setup() {
  Serial.begin(115200);
  pinMode(START_BUTTON_PIN, INPUT_PULLUP);

  for (uint8_t axis = 0; axis < 5; axis++) {
    for (uint8_t m = 0; m < 2; m++) {
      pinMode(MOTOR_PINS[axis][m].step, OUTPUT);
      pinMode(MOTOR_PINS[axis][m].dir, OUTPUT);
      resolveFastPin(MOTOR_PINS[axis][m].step, stepReg[axis][m], stepMask[axis][m]);
      resolveFastPin(MOTOR_PINS[axis][m].dir,  dirReg[axis][m],  dirMask[axis][m]);
    }
    axisState[axis].stepsRemaining = 0;
    axisState[axis].accum = 0;
    axisState[axis].stepRateInt = 0;
    axisState[axis].pulseHigh = false;
    axisState[axis].active = false;
  }

  for (size_t i = 0; i < SEQUENCE_LEN; i++) moveTriggered[i] = false;

  // Timer1 CTC, prescaler 8: OCR1A = F_CPU/8/ISR_FREQ_HZ - 1
  noInterrupts();
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1  = 0;
  OCR1A  = (F_CPU / 8UL / ISR_FREQ_HZ) - 1;
  TCCR1B |= (1 << WGM12);  // CTC mode
  TCCR1B |= (1 << CS11);   // prescaler 8
  TIMSK1 |= (1 << OCIE1A);
  interrupts();
}

// ---------------------------------------------------------------------
// Step ISR -- keep this short. Direct register access only, no math
// beyond integer add/compare.
// ---------------------------------------------------------------------
ISR(TIMER1_COMPA_vect) {
  for (uint8_t axis = 0; axis < 5; axis++) {
    if (!axisState[axis].active) continue;

    if (axisState[axis].pulseHigh) {
      *stepReg[axis][0] &= ~stepMask[axis][0];
      *stepReg[axis][1] &= ~stepMask[axis][1];
      axisState[axis].pulseHigh = false;
      if (axisState[axis].stepsRemaining == 0) {
        axisState[axis].active = false; // move fully complete
      }
      continue;
    }

    if (axisState[axis].stepsRemaining > 0) {
      axisState[axis].accum += axisState[axis].stepRateInt;
      if (axisState[axis].accum >= (int32_t)ISR_FREQ_HZ) {
        axisState[axis].accum -= ISR_FREQ_HZ;
        *stepReg[axis][0] |= stepMask[axis][0];
        *stepReg[axis][1] |= stepMask[axis][1];
        axisState[axis].pulseHigh = true;
        axisState[axis].stepsRemaining--;
      }
    }
  }
}

// ---------------------------------------------------------------------
// Arm a move for one axis. Called from loop(), briefly guards the
// shared state with interrupts off so the ISR never sees a half-written
// struct.
// ---------------------------------------------------------------------
void startAxisMove(AxisId axis, float distance_mm, float speed_mm_s) {
  int8_t  dir = (distance_mm >= 0) ? 1 : -1;
  int32_t steps = lround(fabs(distance_mm) * STEPS_PER_MM[axis]);
  float   speed = min(speed_mm_s, MAX_SPEED_MM_S[axis]);
  uint16_t rate = (uint16_t)lround(speed * STEPS_PER_MM[axis]);

  for (uint8_t m = 0; m < 2; m++) {
    int8_t motorDir = dir * DIR_SIGN[axis][m];
    if (motorDir >= 0) *dirReg[axis][m] |= dirMask[axis][m];
    else                *dirReg[axis][m] &= ~dirMask[axis][m];
  }

  noInterrupts();
  axisState[axis].stepsRemaining = steps;
  axisState[axis].accum = 0;
  axisState[axis].stepRateInt = rate;
  axisState[axis].pulseHigh = false;
  axisState[axis].active = true;
  interrupts();
}

// ---------------------------------------------------------------------
// Non-blocking start trigger: serial byte OR debounced button.
// ---------------------------------------------------------------------
bool checkStartTrigger() {
  if (Serial.available() > 0) {
    char c = Serial.read();
    if (c == START_BYTE) return true;
  }

  static uint32_t lowSince = 0;
  bool pressed = (digitalRead(START_BUTTON_PIN) == LOW);
  if (pressed) {
    if (lowSince == 0) lowSince = millis();
    else if (millis() - lowSince > 25) return true; // debounced
  } else {
    lowSince = 0;
  }
  return false;
}

void loop() {
  if (!sequenceStarted) {
    if (checkStartTrigger()) {
      sequenceStarted = true;
      sequenceStart = millis();
    }
    return;
  }

  uint32_t elapsed = millis() - sequenceStart;
  for (size_t i = 0; i < SEQUENCE_LEN; i++) {
    if (!moveTriggered[i] && elapsed >= SEQUENCE[i].time_ms) {
      startAxisMove(SEQUENCE[i].axis, SEQUENCE[i].distance_mm, SEQUENCE[i].speed_mm_s);
      moveTriggered[i] = true;
    }
  }

  // Stepping happens entirely in the ISR now - loop() is free to do
  // whatever else you need (status prints, a "sequence done" check by
  // scanning axisState[*].active, etc.) without affecting step timing.
}
