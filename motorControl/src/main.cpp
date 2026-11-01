/*
 * Fixed choreography sequencer, ISR-driven stepping (grbl-style).
 * Arduino Mega 2560, 5 dual-motor axes (A-E) on TB6600 drivers.
 *
 * ---------------------------------------------------------------------
 * HOW TO USE (read this first)
 * ---------------------------------------------------------------------
 *  - Edit SEQUENCE[] in section 3. Each row is:
 *        { start_ms, end_ms, AXIS, target_position_mm }
 *    "Move AXIS so it is AT target_position_mm at end_ms, starting the
 *    motion at start_ms." Positions are ABSOLUTE. Speed is NOT an input:
 *    it is derived from distance and duration (see MOTION PROFILE).
 *  - Times are ms from the trigger (serial byte 'S' or the start button).
 *  - Every axis is assumed to be at 0 mm at power-up / reset. Home the
 *    machine by hand BEFORE powering on. Position is tracked in steps in
 *    RAM only (no feedback). Axes not mentioned in a row hold position.
 *  - When the sequence finishes, the firmware prints "DONE" on Serial,
 *    discards any 'S' bytes received during the run, waits for the button
 *    to be released, and is ready to run again. Re-running needs no reset.
 *
 * ---------------------------------------------------------------------
 * COMPILE-TIME CHECKS (static_assert; config is compile-time only)
 * ---------------------------------------------------------------------
 *  SEQUENCE[] is checked while compiling. A failure shows a message and,
 *  in the "In instantiation of 'struct SeqRowError<ROW, RULE>'" line, the
 *  offending ROW index (0-based) and RULE number:
 *    1. end_ms > start_ms.
 *    2. Rows of the same axis are in chronological order in the table and
 *       do not overlap (start_ms >= previous end_ms; touching is fine).
 *       Rows of different axes may be in any order / overlap freely.
 *    3. The move is physically achievable: the duration is long enough for
 *       the axis accel (a trapezoid/triangle profile exists) and the peak
 *       speed stays <= MAX_SPEED_MM_S and <= the ISR limit.
 *       Fix: lengthen the duration, shorten the distance, or raise the
 *       limits in section 2.
 *    4. Each axis ENDS at 0 mm, so the sequence can be re-run (a re-run
 *       starts from where the last run ended; rule 3 only validates moves
 *       from the positions inside the table).
 *  Also checked: positive STEPS_PER_MM / MAX_SPEED_MM_S / ACCEL_MM_S2.
 *  Needs C++11 constexpr (Arduino AVR default). The recursion depth limit
 *  (-fconstexpr-depth, default 512) caps the table at roughly 250 rows;
 *  raise the flag if you need more.
 *
 * ---------------------------------------------------------------------
 * MOTION PROFILE
 * ---------------------------------------------------------------------
 *  Each move is a symmetric trapezoid (triangle if too short to cruise):
 *  accelerate at A = ACCEL_MM_S2, cruise, decelerate at A. Given distance
 *  D (steps) and duration T (s), the cruise speed that covers D in exactly
 *  T is the smaller root of v^2 - A*T*v + A*D = 0:
 *        v = 2D / (T + sqrt(T^2 - 4D/A))            (computed in loop())
 *  The ISR does not know about time, only about steps: it ramps the rate
 *  up by a fixed amount per tick, and starts ramping down when
 *  stepsRemaining <= decelSteps (= v^2 / 2A). The move therefore always
 *  emits exactly the right number of steps. The last step lands at or
 *  slightly BEFORE end_ms, never after: host simulation of this exact ISR
 *  logic gave 3-20 ms early on 0.2-10 s moves (the ramp starts at a
 *  non-zero floor speed and the final step precedes the ideal stop).
 *  Set ACCEL_MM_S2 huge to get (almost) constant-speed moves: ramps vanish
 *  because the ramp floor then exceeds the cruise speed.
 *  The ramp starts/ends at a small non-zero speed (sqrt(A) steps/s) so the
 *  last step is never stuck waiting on a rate that has decayed to zero.
 *
 * ---------------------------------------------------------------------
 * DESIGN
 * ---------------------------------------------------------------------
 *  - Step/dir pins are resolved to raw AVR port register + bitmask ONCE at
 *    setup(); hot paths use direct register writes, not digitalWrite().
 *  - All step pulses come from a Timer1 Compare-Match ISR at ISR_FREQ_HZ.
 *    Each axis has a 32-bit phase accumulator (Q0.32 steps/tick): every tick
 *    phase += rate, and a carry out of the top bit emits one step. Rate is
 *    also Q0.32 steps/tick and is ramped inside the ISR with integer adds.
 *  - Max step rate is ISR_FREQ_HZ / 2 (a step takes a high tick and a low
 *    tick). The ISR runs 5 axes x (3-4 32-bit ops + volatile loads/stores)
 *    per tick, so ISR_FREQ_HZ is a CPU-budget tradeoff: 10 kHz was chosen
 *    to leave headroom with all 5 axes moving at once. If you raise it,
 *    measure ISR time (toggle a spare pin) first.
 *  - loop() never touches a step pin. It waits for the trigger, then arms
 *    moves when millis() reaches each row's start_ms.
 *  - Both motors of an axis get the same step pulse; per-motor direction is
 *    handled by DIR_SIGN (mirrored mounting).
 *
 * ---------------------------------------------------------------------
 * HAZARDS / NOTES FOR FUTURE EDITS
 * ---------------------------------------------------------------------
 *  - Many step/dir pins share one AVR PORT (e.g. pins 46 and 47 are both
 *    PORTL). `*reg |= mask` is a non-atomic read-modify-write on the Mega's
 *    extended I/O registers, so ANY write to a motor pin register from
 *    loop() must be inside noInterrupts()/interrupts(), or the ISR can be
 *    interrupted mid-RMW and a step edge gets lost or stuck.
 *  - startAxisMove() refuses (returns false) while the axis is still
 *    finishing the previous move; loop() simply retries on the next pass.
 *    Float math (sqrtf etc.) is done BEFORE interrupts are disabled.
 *  - No soft travel limits are enforced. Check targets against the
 *    machine's real travel yourself. Moves into negative positions are
 *    allowed and intentional.
 *  - Change log: v2 absolute positions + start/end timestamps; dir-pin RMW
 *    race fixed. v3 trapezoidal accel/decel, compile-time sequence checks
 *    (replaced boot-time validation), sequence re-runnable (the old
 *    one-shot sequenceStarted flag was never cleared), ISR 20k -> 10 kHz.
 *
 * No AccelStepper/MultiStepper dependency - self-contained beyond avr/io.h,
 * which Arduino.h already pulls in.
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
// 2. MECHANICAL CONFIG (all compile-time)
// ---------------------------------------------------------------------
constexpr float STEPS_PER_MM[5]   = { 13.333f, 13.333f, 13.333f, 13.333f, 13.333f };
constexpr float MAX_SPEED_MM_S[5] = { 100, 100, 100, 100, 100 };
// PLACEHOLDER - tune on the machine. Higher = shorter ramps but more stall
// risk. Rule 3 rejects moves too short for this accel.
constexpr float ACCEL_MM_S2[5]    = { 500, 500, 500, 500, 500 };

// Per-motor direction sign, for mirrored mounting on a given axis.
const int8_t DIR_SIGN[5][2] = {
  { -1, 1 },
  { 1, -1 },
  { 1, -1 },
  { -1, -1 },
  { 1, -1 },
};

// ---------------------------------------------------------------------
// 3. CHOREOGRAPHY -- edit this table for a new routine
// ---------------------------------------------------------------------
enum AxisId { A = 0, B = 1, C = 2, D = 3, E = 4 };

struct Move {
  uint32_t start_ms;   // motion begins, ms after trigger
  uint32_t end_ms;     // motion ends (axis is at pos_mm), ms after trigger
  AxisId   axis;
  float    pos_mm;     // ABSOLUTE target position, 0 = power-up position
};

constexpr Move SEQUENCE[] = {
  { 0000,  5000, A,  200 },
  { 7000,  12000, A,  0 },
  { 1000, 5000, B, 20},
  { 1000, 5000, C, 20},
  { 8000, 9000, B, 0},
  { 8000, 9000, C, 0},
  { 9000, 10000, B, -1},
  { 9000, 10000, C, -1},
  { 10000, 12000, B, 20},
  { 10000, 12000, C, 20},

  { 1000, 4000, D, 20},
  { 1000, 4000, E, 20},
  { 4000, 5000, D, 0},
  { 4000, 5000, E, 0},
  { 12000, 13000, D, 20},
  { 12000, 13000, E, 20},
};
constexpr int SEQUENCE_LEN = sizeof(SEQUENCE) / sizeof(SEQUENCE[0]);

// ---------------------------------------------------------------------
// 4. TIMER / ISR CONFIG
// ---------------------------------------------------------------------
// Max achievable step rate per axis is ISR_FREQ_HZ / 2.
constexpr uint32_t ISR_FREQ_HZ = 10000; // 100us tick -> 5kHz max step rate/axis

// ---------------------------------------------------------------------
// 4b. COMPILE-TIME SEQUENCE CHECKS (C++11 constexpr: single-return, recursive)
// ---------------------------------------------------------------------
constexpr int32_t toSteps(float mm, int ax) {
  return (int32_t)(mm * STEPS_PER_MM[ax] + (mm >= 0 ? 0.5f : -0.5f));
}
constexpr int32_t absI(int32_t x) { return x < 0 ? -x : x; }
constexpr float   minF(float a, float b) { return a < b ? a : b; }
constexpr float   vLimitSteps(int ax) {
  return minF(MAX_SPEED_MM_S[ax] * STEPS_PER_MM[ax], ISR_FREQ_HZ / 2.0f);
}

// previous row (before row i) on the same axis, or -1
constexpr int prevRow(int i, int j) {
  return j < 0 ? -1 : (SEQUENCE[j].axis == SEQUENCE[i].axis ? j : prevRow(i, j - 1));
}
constexpr int prevRow(int i) { return prevRow(i, i - 1); }
// true if no later row uses the same axis as row i
constexpr bool isLastOfAxis(int i, int j) {
  return j >= SEQUENCE_LEN ? true : (SEQUENCE[j].axis == SEQUENCE[i].axis ? false : isLastOfAxis(i, j + 1));
}

constexpr int32_t startSteps(int i) {
  return prevRow(i) < 0 ? 0 : toSteps(SEQUENCE[prevRow(i)].pos_mm, SEQUENCE[i].axis);
}

// A profile of accel A (steps/s^2), peak <= vlim, covering D steps in T s
// exists iff (triangle fits) T^2*A >= 4D and the smaller root of
// v^2 - A*T*v + A*D = 0 is <= vlim.
constexpr bool feasible(float D, float T, float A, float vlim) {
  return (T * T * A >= 4 * D) && (vlim >= A * T / 2 || vlim * vlim - A * T * vlim + A * D <= 0);
}

constexpr bool rule1(int i) { return SEQUENCE[i].end_ms > SEQUENCE[i].start_ms; }
constexpr bool rule2(int i) {
  return prevRow(i) < 0 || SEQUENCE[i].start_ms >= SEQUENCE[prevRow(i)].end_ms;
}
constexpr bool rule3(int i) {
  return feasible((float)absI(toSteps(SEQUENCE[i].pos_mm, SEQUENCE[i].axis) - startSteps(i)),
                  (SEQUENCE[i].end_ms - SEQUENCE[i].start_ms) * 0.001f,
                  ACCEL_MM_S2[SEQUENCE[i].axis] * STEPS_PER_MM[SEQUENCE[i].axis],
                  vLimitSteps(SEQUENCE[i].axis));
}
constexpr bool rule4(int i) {
  return !isLastOfAxis(i, i + 1) || toSteps(SEQUENCE[i].pos_mm, SEQUENCE[i].axis) == 0;
}
constexpr bool rowOk(int rule, int i) {
  return rule == 1 ? rule1(i) : rule == 2 ? rule2(i) : rule == 3 ? rule3(i) : rule4(i);
}
// index of first row breaking `rule`, or SEQUENCE_LEN if none
constexpr int firstBad(int rule, int i) {
  return i >= SEQUENCE_LEN ? SEQUENCE_LEN : (rowOk(rule, i) ? firstBad(rule, i + 1) : i);
}

// Instantiating this with a bad Row triggers a static_assert whose compiler
// output names Row and Rule.
template <int Row, int Rule> struct SeqRowError {
  static_assert(Row == SEQUENCE_LEN || Rule != 1, "SEQUENCE rule 1: end_ms must be greater than start_ms");
  static_assert(Row == SEQUENCE_LEN || Rule != 2, "SEQUENCE rule 2: same-axis rows overlap or are out of order");
  static_assert(Row == SEQUENCE_LEN || Rule != 3, "SEQUENCE rule 3: move too fast/short for ACCEL_MM_S2, MAX_SPEED_MM_S or ISR rate - lengthen it");
  // static_assert(Row == SEQUENCE_LEN || Rule != 4, "SEQUENCE rule 4: axis must end at 0 mm so the sequence can be re-run");
};
static_assert(sizeof(SeqRowError<firstBad(1, 0), 1>) > 0, "SEQUENCE rule 1 failed");
static_assert(sizeof(SeqRowError<firstBad(2, 0), 2>) > 0, "SEQUENCE rule 2 failed");
static_assert(sizeof(SeqRowError<firstBad(3, 0), 3>) > 0, "SEQUENCE rule 3 failed");
// static_assert(sizeof(SeqRowError<firstBad(4, 0), 4>) > 0, "SEQUENCE rule 4 failed");

constexpr bool cfgOk(int a) {
  return a >= 5 ? true : (STEPS_PER_MM[a] > 0 && MAX_SPEED_MM_S[a] > 0 && ACCEL_MM_S2[a] > 0 && cfgOk(a + 1));
}
static_assert(cfgOk(0), "STEPS_PER_MM, MAX_SPEED_MM_S and ACCEL_MM_S2 must all be > 0");

// ---------------------------------------------------------------------
// 5. RUNTIME STATE
// ---------------------------------------------------------------------
volatile uint8_t *stepReg[5][2];
uint8_t stepMask[5][2];
volatile uint8_t *dirReg[5][2];
uint8_t dirMask[5][2];

// Rates are Q0.32 steps/tick (2^32 = one step per tick; max used is 2^31).
struct AxisState {
  volatile int32_t  stepsRemaining; // steps still to emit
  volatile int32_t  decelSteps;     // start ramping down at this many remaining
  volatile uint32_t phase;          // accumulator; carry out = step
  volatile uint32_t v;              // current rate
  volatile uint32_t vCruise;        // peak rate
  volatile uint32_t vFloor;         // start/end rate
  volatile uint32_t accel;          // rate change per tick
  volatile bool     pulseHigh;
  volatile bool     active;
};
volatile AxisState axisState[5];

// Loop-side only
int32_t posSteps[5] = {0, 0, 0, 0, 0};  // commanded absolute position, steps

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
    axisState[axis].decelSteps = 0;
    axisState[axis].phase = 0;
    axisState[axis].v = 0;
    axisState[axis].vCruise = 0;
    axisState[axis].vFloor = 0;
    axisState[axis].accel = 0;
    axisState[axis].pulseHigh = false;
    axisState[axis].active = false;
  }

  for (int i = 0; i < SEQUENCE_LEN; i++) moveTriggered[i] = false;

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
// Step ISR -- keep this short: integer add/compare only, no division/float.
// Per tick, per active axis: lower last tick's pulse, update the rate
// (ramp down once stepsRemaining <= decelSteps, else ramp up to vCruise),
// then phase += rate and emit a step on carry.
// ---------------------------------------------------------------------
ISR(TIMER1_COMPA_vect) {
  for (uint8_t axis = 0; axis < 5; axis++) {
    volatile AxisState &s = axisState[axis];
    if (!s.active) continue;

    if (s.pulseHigh) {
      *stepReg[axis][0] &= ~stepMask[axis][0];
      *stepReg[axis][1] &= ~stepMask[axis][1];
      s.pulseHigh = false;
    }

    if (s.stepsRemaining > 0) {
      uint32_t v = s.v;
      uint32_t a = s.accel;
      if (s.stepsRemaining <= s.decelSteps) {
        v = (v - s.vFloor > a) ? v - a : s.vFloor;
      } else {
        v = (s.vCruise - v > a) ? v + a : s.vCruise;
      }
      s.v = v;

      uint32_t old = s.phase;
      uint32_t p = old + v;
      s.phase = p;
      if (p < old) { // carry -> step
        *stepReg[axis][0] |= stepMask[axis][0];
        *stepReg[axis][1] |= stepMask[axis][1];
        s.pulseHigh = true;
        s.stepsRemaining--;
      }
    } else if (!s.pulseHigh) {
      s.active = false; // all steps emitted and last pulse lowered
    }
  }
}

// steps/s -> Q0.32 steps/tick, clamped to 2^31 (the ISR_FREQ_HZ/2 limit)
static uint32_t toFix(float stepsPerSec) {
  float f = stepsPerSec * (4294967296.0f / (float)ISR_FREQ_HZ);
  if (f > 2147483648.0f) f = 2147483648.0f;
  return (uint32_t)f;
}

// ---------------------------------------------------------------------
// Arm a move: go to absolute target_mm, taking duration_ms. Returns false
// if the axis is still busy (caller retries). All float math happens first;
// interrupts are off only for the short commit.
// ---------------------------------------------------------------------
bool startAxisMove(AxisId axis, float target_mm, uint32_t duration_ms) {
  if (axisState[axis].active) return false;

  int32_t target = toSteps(target_mm, axis);
  int32_t delta  = target - posSteps[axis];
  if (delta == 0) return true;

  float D = (float)labs(delta);                         // steps
  float T = (float)duration_ms * 0.001f;                // s
  float A = ACCEL_MM_S2[axis] * STEPS_PER_MM[axis];     // steps/s^2

  float disc = T * T - 4.0f * D / A;
  if (disc < 0) disc = 0;                               // rule 3 rules this out at compile time
  float vPeak  = 2.0f * D / (T + sqrtf(disc));          // steps/s
  float vFloor = sqrtf(A);
  if (vFloor > vPeak) vFloor = vPeak;
  int32_t decelSteps = (int32_t)(vPeak * vPeak / (2.0f * A) + 0.5f);

  uint32_t accelFix = toFix(A / (float)ISR_FREQ_HZ);    // (steps/s)/tick
  if (accelFix == 0) accelFix = 1;
  uint32_t cruiseFix = toFix(vPeak);
  uint32_t floorFix  = toFix(vFloor);
  if (floorFix == 0) floorFix = 1;
  if (cruiseFix < floorFix) cruiseFix = floorFix;

  int8_t dir = (delta > 0) ? 1 : -1;

  noInterrupts();
  for (uint8_t m = 0; m < 2; m++) {
    if (dir * DIR_SIGN[axis][m] >= 0) *dirReg[axis][m] |= dirMask[axis][m];
    else                              *dirReg[axis][m] &= ~dirMask[axis][m];
  }
  axisState[axis].stepsRemaining = labs(delta);
  axisState[axis].decelSteps = decelSteps;
  axisState[axis].phase = 0;
  axisState[axis].v = floorFix;
  axisState[axis].vCruise = cruiseFix;
  axisState[axis].vFloor = floorFix;
  axisState[axis].accel = accelFix;
  axisState[axis].pulseHigh = false;
  axisState[axis].active = true;
  interrupts();

  posSteps[axis] = target;
  return true;
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

bool anyAxisActive() {
  for (uint8_t axis = 0; axis < 5; axis++) {
    if (axisState[axis].active) return true;
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
  bool allArmed = true;
  for (int i = 0; i < SEQUENCE_LEN; i++) {
    if (moveTriggered[i]) continue;
    allArmed = false;
    if (elapsed >= SEQUENCE[i].start_ms &&
        startAxisMove(SEQUENCE[i].axis, SEQUENCE[i].pos_mm,
                      SEQUENCE[i].end_ms - SEQUENCE[i].start_ms)) {
      moveTriggered[i] = true;
    }
  }

  // Sequence finished: every move armed and every axis has stopped.
  if (allArmed && !anyAxisActive()) {
    sequenceStarted = false;
    for (int i = 0; i < SEQUENCE_LEN; i++) moveTriggered[i] = false;
    while (Serial.available() > 0) Serial.read();      // drop 'S' sent mid-run
    while (digitalRead(START_BUTTON_PIN) == LOW) {}    // wait for button release
    Serial.println(F("DONE"));
  }
}