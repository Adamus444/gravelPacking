/*
 * Choreography sequencer (PRE / MAIN / POST), ISR-driven stepping (grbl-style).
 * Arduino Mega 2560, 5 dual-motor axes (A-E) on TB6600 drivers.
 *
 * ---------------------------------------------------------------------
 * HOW TO USE (read this first)
 * ---------------------------------------------------------------------
 *  Three tables in section 3, all with the same row format:
 *        { start_ms, end_ms, AXIS, target_position_mm }
 *  "Move AXIS so it is AT target_position_mm at end_ms, starting the
 *  motion at start_ms." Positions are ABSOLUTE. Speed is NOT an input: it
 *  is derived from distance and duration (see MOTION PROFILE). Times are
 *  ms from that sequence's own trigger. Axes not mentioned hold position.
 *
 *    PRE_SEQUENCE   serial 'P'   Power-up (0 mm everywhere) -> production
 *                                start position ("base"). Run once, at the
 *                                very start of production.
 *    MAIN_SEQUENCE  serial 'S'   The show. Repeatable any number of times.
 *                  or button     Starts at base and must END at base.
 *    POST_SEQUENCE  serial 'Z'   base -> 0 mm on every axis (end of
 *                                production / before power-off).
 *
 *  Every axis is assumed to be at 0 mm at power-up / reset. Home the
 *  machine by hand BEFORE powering on. Position is tracked in steps in RAM
 *  only (no feedback), so a reset forgets where the machine is.
 *
 *  Allowed order (a human drives it; anything else is ignored with a
 *  message on Serial @115200):
 *        at 0 --P--> at base --S--> at base --S--> ... --Z--> at 0
 *  P is only accepted at 0, S and Z only at base, and nothing is accepted
 *  while a sequence is running. Bytes received mid-run are discarded when
 *  the run ends (no queued surprise runs). Other bytes (e.g. '\n') are
 *  ignored. Serial prints "START <name>" and "DONE <name>".
 *  The button triggers MAIN only, once per press.
 *
 *  JOG (re-zeroing):  J<axis><motor><mm>   then newline, e.g.
 *        JA0 5      JB1-2.5      Jc0 0.3      JD2 1
 *    Physically steps ONE motor of one axis by a signed distance in mm, at
 *    JOG_SPEED_MM_S, WITHOUT updating the tracked position. axis = A-E,
 *    motor = 0 or 1 (the first / second entry of that axis in MOTOR_PINS),
 *    or 2 for both motors together. Positive = the direction that motor
 *    would turn for a positive position (DIR_SIGN is applied per motor).
 *    A space between the motor digit and the distance is optional.
 *    The firmware still believes the axis is where it was, so the 0 mm
 *    (and base) reference moves by the jogged amount. Use it to nudge
 *    the machine onto its true zero before 'P', or to fine-tune base.
 *    NOTE: jogging a single motor of a dual-motor axis changes the
 *    alignment between its two motors (that is the point, e.g. squaring a
 *    gantry); all later moves keep that offset.
 *    Only accepted while no sequence is running, and only if that axis is
 *    not already moving. Serial prints "JOG <axis><motor> <mm> (<n> steps)"
 *    and "JOG DONE". The line MUST end with a newline (set the Serial
 *    Monitor to "Newline"); it is dropped if more than JOG_LINE_TIMEOUT_MS
 *    (2 s) passes between typed characters. Moves are limited to
 *    +/- JOG_MAX_MM per command; send several for longer jogs.
 *    While any axis is jogging, P/S/Z/button are ignored.
 *  There is NO abort/e-stop in firmware: a running sequence always
 *  finishes. Cut driver power for an emergency.
 *  Each table needs at least one row. For a no-op PRE/POST, use a
 *  zero-distance row, e.g. { 0, 1000, A, 0 } (a move to where it already is
 *  does nothing).
 *
 * ---------------------------------------------------------------------
 * COMPILE-TIME CHECKS (static_assert; config is compile-time only)
 * ---------------------------------------------------------------------
 *  "base" is derived, not configured: base = where PRE leaves each axis
 *  (0 for axes PRE doesn't touch). All three tables are checked while
 *  compiling. A failure shows a message and, in the
 *  "In instantiation of 'struct SeqError<INDEX, RULE, SEQ>'" line, the
 *  INDEX (row, or axis number 0-4 for rule 4), the RULE and SEQ
 *  (0 = PRE, 1 = MAIN, 2 = POST):
 *    1. end_ms > start_ms.
 *    2. Rows of the same axis are in chronological order in the table and
 *       do not overlap (start_ms >= previous end_ms; touching is fine).
 *       Rows of different axes may be in any order / overlap freely.
 *    3. The move is physically achievable from the position the axis is at
 *       when the row starts: the duration is long enough for the axis
 *       accel (a trapezoid/triangle profile exists) and the peak speed
 *       stays <= MAX_SPEED_MM_S and <= the ISR limit.
 *       Fix: lengthen the duration, shorten the distance, or raise the
 *       limits in section 2.
 *    4. End positions: MAIN must end every axis where MAIN started it
 *       (= base), so it can repeat. POST must end every axis at 0 mm.
 *       (PRE has no end rule.)
 *  Also checked: positive STEPS_PER_MM / MAX_SPEED_MM_S / ACCEL_MM_S2.
 *  Needs C++11 constexpr (Arduino AVR default). The recursion depth limit
 *  (-fconstexpr-depth, default 512) caps a table at roughly 200 rows;
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
 *  - loop() never touches a step pin. It reads commands, then arms moves
 *    when millis() reaches each row's start_ms.
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
 *    (replaced boot-time validation), re-runnable sequence, ISR 20k ->
 *    10 kHz. v4 PRE/MAIN/POST sequences with serial commands P/S/Z and
 *    position-state gating; MAIN no longer has to end at 0 (it must end at
 *    base); button is now edge-triggered (one trigger per press). v5 serial
 *    JOG command ('JA0 5', 'JB1-2.5'): steps one motor (or both) of an axis
 *    without updating the tracked position, shifting the machine's zero;
 *    startAxisMove split into commitMove() + wrapper; per-motor step mask in
 *    the ISR (AxisState.motorMask); jog line timeout 2 s.
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
const char    PRE_BYTE   = 'P';       // serial: run PRE_SEQUENCE  (only accepted at 0)
const char    START_BYTE = 'S';       // serial / button: run MAIN_SEQUENCE (only at base)
const char    POST_BYTE  = 'Z';       // serial: run POST_SEQUENCE (only at base)

// ---------------------------------------------------------------------
// 2. MECHANICAL CONFIG (all compile-time)
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

// ---------------------------------------------------------------------
// 3. CHOREOGRAPHY -- edit these tables for a new routine
// ---------------------------------------------------------------------
enum AxisId { A = 0, B = 1, C = 2, D = 3, E = 4 };

struct Move {
  uint32_t start_ms;   // motion begins, ms after this sequence's trigger
  uint32_t end_ms;     // motion ends (axis is at pos_mm), ms after trigger
  AxisId   axis;
  float    pos_mm;     // ABSOLUTE target position, 0 = power-up position
};

// PLACEHOLDER VALUES - replace with the real production positions.
// PRE: power-up (0) -> base. The base is whatever this leaves each axis at.
constexpr Move PRE_SEQUENCE[] = {
  { 0, 2000, B, 30 },
  { 0, 2000, C, 30 },
  { 0, 2000, D, 30 },
  { 0, 2000, E, 30 },
};

// MAIN: starts at base, must end at base (rule 4) so it can repeat.
constexpr float mult = 1.5;
constexpr Move MAIN_SEQUENCE[] = {
    { (uint32_t)(0000*mult),  (uint32_t)(6000*mult), A,  270 },
    { (uint32_t)(7000*mult),  (uint32_t)(13000*mult), A,  0 },
    
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

// previous row (before row i) in sequence id on the same axis, or -1
constexpr int prevRow(int id, int i, int j) {
  return j < 0 ? -1 : (rowsOf(id)[j].axis == rowsOf(id)[i].axis ? j : prevRow(id, i, j - 1));
}
constexpr int prevRow(int id, int i) { return prevRow(id, i, i - 1); }
// last row in sequence id using axis ax, or -1
constexpr int lastRow(int id, int ax, int j) {
  return j < 0 ? -1 : (rowsOf(id)[j].axis == ax ? j : lastRow(id, ax, j - 1));
}

// Where each axis is when a sequence starts / ends, in steps.
// PRE starts at 0; MAIN starts where PRE ended; POST starts where MAIN ended.
constexpr int32_t baseSteps(int id, int ax);  // defined below (mutual recursion)
constexpr int32_t endSteps(int id, int ax) {
  return lastRow(id, ax, lenOf(id) - 1) < 0
           ? baseSteps(id, ax)
           : toSteps(rowsOf(id)[lastRow(id, ax, lenOf(id) - 1)].pos_mm, ax);
}
constexpr int32_t baseSteps(int id, int ax) { return id == 0 ? 0 : endSteps(id - 1, ax); }

// where the axis of row i is when row i starts
constexpr int32_t startSteps(int id, int i) {
  return prevRow(id, i) < 0
           ? baseSteps(id, rowsOf(id)[i].axis)
           : toSteps(rowsOf(id)[prevRow(id, i)].pos_mm, rowsOf(id)[i].axis);
}

// A profile of accel A (steps/s^2), peak <= vlim, covering D steps in T s
// exists iff (triangle fits) T^2*A >= 4D and the smaller root of
// v^2 - A*T*v + A*D = 0 is <= vlim.
constexpr bool feasible(float D, float T, float A, float vlim) {
  return (T * T * A >= 4 * D) && (vlim >= A * T / 2 || vlim * vlim - A * T * vlim + A * D <= 0);
}

constexpr bool rule1(int id, int i) { return rowsOf(id)[i].end_ms > rowsOf(id)[i].start_ms; }
constexpr bool rule2(int id, int i) {
  return prevRow(id, i) < 0 || rowsOf(id)[i].start_ms >= rowsOf(id)[prevRow(id, i)].end_ms;
}
constexpr bool rule3(int id, int i) {
  return feasible((float)absI(toSteps(rowsOf(id)[i].pos_mm, rowsOf(id)[i].axis) - startSteps(id, i)),
                  (rowsOf(id)[i].end_ms - rowsOf(id)[i].start_ms) * 0.001f,
                  ACCEL_MM_S2[rowsOf(id)[i].axis] * STEPS_PER_MM[rowsOf(id)[i].axis],
                  vLimitSteps(rowsOf(id)[i].axis));
}
// per axis: PRE free; MAIN ends where it began; POST ends at 0
constexpr bool rule4(int id, int ax) {
  return id == 0 ? true
       : id == 1 ? endSteps(1, ax) == baseSteps(1, ax)
                 : endSteps(2, ax) == 0;
}
constexpr bool rowOk(int rule, int id, int i) {
  return rule == 1 ? rule1(id, i) : rule == 2 ? rule2(id, i) : rule3(id, i);
}
// first row of sequence id breaking rule 1-3, or -1
constexpr int firstBadRow(int id, int rule, int i) {
  return i >= lenOf(id) ? -1 : (rowOk(rule, id, i) ? firstBadRow(id, rule, i + 1) : i);
}
// first axis breaking rule 4, or -1
constexpr int firstBadAxis(int id, int ax) {
  return ax >= 5 ? -1 : (rule4(id, ax) ? firstBadAxis(id, ax + 1) : ax);
}

// Instantiating this with a bad Index triggers a static_assert whose compiler
// output names Index (row, or axis for rule 4), Rule and Seq (0=PRE 1=MAIN 2=POST).
template <int Index, int Rule, int Seq> struct SeqError {
  static_assert(Index < 0 || Rule != 1, "SEQUENCE rule 1: end_ms must be greater than start_ms (see <row, rule, seq> in the instantiation line; seq 0=PRE 1=MAIN 2=POST)");
  static_assert(Index < 0 || Rule != 2, "SEQUENCE rule 2: same-axis rows overlap or are out of order (see <row, rule, seq>; seq 0=PRE 1=MAIN 2=POST)");
  static_assert(Index < 0 || Rule != 3, "SEQUENCE rule 3: move too fast/short for ACCEL_MM_S2, MAX_SPEED_MM_S or ISR rate - lengthen it (see <row, rule, seq>; seq 0=PRE 1=MAIN 2=POST)");
  static_assert(Index < 0 || Rule != 4 || Seq != 1, "SEQUENCE rule 4: MAIN_SEQUENCE must end this axis (index = axis 0-4) where it started, i.e. at the PRE end position, so it can repeat");
  static_assert(Index < 0 || Rule != 4 || Seq != 2, "SEQUENCE rule 4: POST_SEQUENCE must end this axis (index = axis 0-4) at 0 mm");
};
#define SEQ_CHECK(id)                                                                  \
  static_assert(sizeof(SeqError<firstBadRow(id, 1, 0), 1, id>) > 0, "rule 1 failed"); \
  static_assert(sizeof(SeqError<firstBadRow(id, 2, 0), 2, id>) > 0, "rule 2 failed"); \
  static_assert(sizeof(SeqError<firstBadRow(id, 3, 0), 3, id>) > 0, "rule 3 failed"); \
  static_assert(sizeof(SeqError<firstBadAxis(id, 0),   4, id>) > 0, "rule 4 failed")
SEQ_CHECK(0);
SEQ_CHECK(1);
SEQ_CHECK(2);

constexpr bool cfgOk(int a) {
  return a >= 5 ? true : (STEPS_PER_MM[a] > 0 && MAX_SPEED_MM_S[a] > 0 && ACCEL_MM_S2[a] > 0 && cfgOk(a + 1));
}
static_assert(cfgOk(0), "STEPS_PER_MM, MAX_SPEED_MM_S and ACCEL_MM_S2 must all be > 0");

constexpr bool jogOk(int a) {
  return a >= 5 ? true : (JOG_SPEED_MM_S * STEPS_PER_MM[a] <= vLimitSteps(a) && jogOk(a + 1));
}
static_assert(JOG_SPEED_MM_S > 0 && JOG_MAX_MM > 0, "JOG_SPEED_MM_S and JOG_MAX_MM must be > 0");
static_assert(jogOk(0), "JOG_SPEED_MM_S exceeds MAX_SPEED_MM_S or the ISR step-rate limit on some axis");

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
  volatile uint8_t  motorMask;      // bit0 = motor 0, bit1 = motor 1: which get step pulses
  volatile bool     pulseHigh;
  volatile bool     active;
};
volatile AxisState axisState[5];

// Loop-side only
int32_t posSteps[5] = {0, 0, 0, 0, 0};  // commanded absolute position, steps

// Sequencer state
bool     moveTriggered[MAX_LEN];
bool     running = false;     // a sequence is in progress
int      curId = 0;           // which one: 0 PRE, 1 MAIN, 2 POST
bool     atBase = false;      // false = at 0 (power-up / after POST), true = at base
uint32_t sequenceStart = 0;
const char* const SEQ_NAME[3] = { "PRE", "MAIN", "POST" };

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
    axisState[axis].motorMask = 3;
    axisState[axis].pulseHigh = false;
    axisState[axis].active = false;
  }

  for (int i = 0; i < MAX_LEN; i++) moveTriggered[i] = false;

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
        uint8_t mm = s.motorMask;
        if (mm & 1) *stepReg[axis][0] |= stepMask[axis][0];
        if (mm & 2) *stepReg[axis][1] |= stepMask[axis][1];
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
// Shared arming code: emit |delta| steps on one axis (sign = direction) with
// a trapezoid/triangle of peak vPeak (steps/s) and accel A (steps/s^2).
// motorMask selects the motors that move (bit0 = motor 0, bit1 = motor 1);
// the other motor's step AND dir pins are left alone.
// Does NOT touch posSteps -- callers decide whether the move is "remembered".
// Float math first; interrupts are off only for the short commit.
// Caller guarantees the axis is idle and delta != 0.
// ---------------------------------------------------------------------
static void commitMove(uint8_t axis, uint8_t motorMask, int32_t delta, float vPeak, float A) {
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
    if (!(motorMask & (1 << m))) continue;
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
  axisState[axis].motorMask = motorMask;
  axisState[axis].pulseHigh = false;
  axisState[axis].active = true;
  interrupts();
}

// ---------------------------------------------------------------------
// Arm a move: go to absolute target_mm, taking duration_ms. Returns false
// if the axis is still busy (caller retries). Updates the tracked position.
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

  commitMove(axis, 3, delta, vPeak, A);               // both motors
  posSteps[axis] = target;
  return true;
}

// ---------------------------------------------------------------------
// JOG: step one motor (motorMask 1 = motor 0, 2 = motor 1) or both (3) of an axis by a relative
// distance WITHOUT updating posSteps, so the firmware's idea of "where the
// axis is" stays put while the metal moves. Net effect: the machine's 0 /
// base reference shifts by the jogged amount.
// Returns false if the axis is busy. Constant JOG_SPEED_MM_S, shortened to a
// triangle profile for tiny jogs.
// ---------------------------------------------------------------------
bool jogAxis(uint8_t axis, uint8_t motorMask, float mm, int32_t &stepsOut) {
  stepsOut = 0;
  if (axisState[axis].active) return false;

  int32_t delta = toSteps(mm, axis);
  stepsOut = delta;
  if (delta == 0) return true;

  float D = (float)labs(delta);
  float A = ACCEL_MM_S2[axis] * STEPS_PER_MM[axis];
  float vPeak = JOG_SPEED_MM_S * STEPS_PER_MM[axis];
  float vTri  = sqrtf(A * D);                           // peak of a pure triangle over D
  if (vTri < vPeak) vPeak = vTri;

  commitMove(axis, motorMask, delta, vPeak, A);         // posSteps deliberately NOT updated
  return true;
}

// Parse the text after 'J': "<axis A-E><signed mm>", e.g. "A5", "b-2.5".
bool jogPending = false;     // a jog was started; announce JOG DONE when it ends

void handleJogLine(const char *p) {
  while (*p == ' ') p++;
  char ax = (char)toupper((unsigned char)*p);
  if (ax < 'A' || ax > 'E') {
    Serial.println(F("JOG ERR: expected J<axis A-E><motor 0-2><mm>, e.g. JA0 5 or JB1-2.5"));
    return;
  }
  p++;
  while (*p == ' ') p++;
  if (*p < '0' || *p > '2') {
    Serial.println(F("JOG ERR: motor must be 0, 1 or 2 (both), e.g. JA0 5"));
    return;
  }
  uint8_t motor = (uint8_t)(*p - '0');
  uint8_t motorMask = (motor == 2) ? 3 : (uint8_t)(1 << motor);   // 0 -> bit0, 1 -> bit1, 2 -> both
  p++;
  char *end;
  float mm = (float)strtod(p, &end);
  if (end == p) {
    Serial.println(F("JOG ERR: missing distance, e.g. JA0 5 or JB1-2.5"));
    return;
  }
  if (!(fabsf(mm) <= JOG_MAX_MM)) {
    Serial.print(F("JOG ERR: |distance| must be <= ")); Serial.print(JOG_MAX_MM, 1); Serial.println(F(" mm"));
    return;
  }

  uint8_t axis = (uint8_t)(ax - 'A');
  int32_t steps;
  if (!jogAxis(axis, motorMask, mm, steps)) {
    Serial.print(F("JOG ERR: axis ")); Serial.print(ax); Serial.println(F(" is busy"));
    return;
  }
  Serial.print(F("JOG ")); Serial.print(ax); Serial.print(motor); Serial.print(' ');
  Serial.print(mm, 3); Serial.print(F(" mm ("));
  Serial.print(steps); Serial.println(F(" steps) - zero reference shifted"));
  if (steps != 0) jogPending = true;
}

// ---------------------------------------------------------------------
// Non-blocking command input: next serial byte, or the start button
// (edge-triggered: one START_BYTE per press, after a 25 ms debounce).
// Returns 0 if nothing arrived.
// ---------------------------------------------------------------------
char readCommand() {
  if (Serial.available() > 0) return (char)Serial.read();

  static uint32_t lowSince = 0;
  static bool fired = false;
  if (digitalRead(START_BUTTON_PIN) == LOW) {
    if (lowSince == 0) lowSince = millis();
    else if (!fired && millis() - lowSince > 25) { fired = true; return START_BYTE; }
  } else {
    lowSince = 0;
    fired = false;
  }
  return 0;
}

bool anyAxisActive() {
  for (uint8_t axis = 0; axis < 5; axis++) {
    if (axisState[axis].active) return true;
  }
  return false;
}

void beginSequence(int id) {
  curId = id;
  for (int i = 0; i < MAX_LEN; i++) moveTriggered[i] = false;
  sequenceStart = millis();
  running = true;
  Serial.print(F("START ")); Serial.println(SEQ_NAME[id]);
}

void loop() {
  if (!running) {
    // --- jog line collection (after a 'J'), serial only ---
    static bool     jogCollecting = false;
    static char     jogBuf[24];
    static uint8_t  jogLen = 0;
    static uint32_t jogLastRx = 0;

    if (jogCollecting) {
      while (Serial.available() > 0) {
        char ch = (char)Serial.read();
        jogLastRx = millis();
        if (ch == '\n' || ch == '\r') {
          jogBuf[jogLen] = 0;
          jogCollecting = false;
          handleJogLine(jogBuf);
          break;
        }
        if (jogLen < sizeof(jogBuf) - 1) jogBuf[jogLen++] = ch;
      }
      if (jogCollecting && millis() - jogLastRx > JOG_LINE_TIMEOUT_MS) {
        jogCollecting = false;
        Serial.println(F("JOG ERR: line not finished in time, dropped"));
      }
      return;
    }

    if (jogPending && !anyAxisActive()) {
      jogPending = false;
      Serial.println(F("JOG DONE"));
    }

    char c = readCommand();
    if (c == 'J' || c == 'j') {
      jogCollecting = true;
      jogLen = 0;
      jogLastRx = millis();
      return;
    }
    if ((c == PRE_BYTE || c == START_BYTE || c == POST_BYTE) && anyAxisActive()) {
      Serial.println(F("IGNORED: a jog is still moving"));
      return;
    }
    if (c == PRE_BYTE) {
      if (atBase) Serial.println(F("IGNORED P: already at base, send Z first"));
      else        beginSequence(0);
    } else if (c == START_BYTE) {
      if (!atBase) Serial.println(F("IGNORED S: not at base, send P first"));
      else         beginSequence(1);
    } else if (c == POST_BYTE) {
      if (!atBase) Serial.println(F("IGNORED Z: already at 0"));
      else         beginSequence(2);
    }
    return;
  }

  const Move *rows = rowsOf(curId);
  int len = lenOf(curId);
  uint32_t elapsed = millis() - sequenceStart;
  bool allArmed = true;
  for (int i = 0; i < len; i++) {
    if (moveTriggered[i]) continue;
    allArmed = false;
    if (elapsed >= rows[i].start_ms &&
        startAxisMove(rows[i].axis, rows[i].pos_mm, rows[i].end_ms - rows[i].start_ms)) {
      moveTriggered[i] = true;
    }
  }

  // Finished: every move armed and every axis has stopped.
  if (allArmed && !anyAxisActive()) {
    running = false;
    atBase = (curId != 2);                              // PRE and MAIN leave us at base, POST at 0
    while (Serial.available() > 0) Serial.read();       // drop commands sent mid-run
    Serial.print(F("DONE ")); Serial.println(SEQ_NAME[curId]);
  }
}