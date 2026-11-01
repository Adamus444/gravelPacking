#include <SPI.h>
#include <math.h>

// ===================== COMPILE-TIME CONFIG =====================
#define NUM_SENSORS   2      // MAX6675 count (1 or 2)
#define NUM_DISPLAYS  2      // 74HC595 display count (1 or 2)
#define NUM_HEATERS   2      // heater count (1 or 2); heater i uses sensor i + pot i

// Pot range: 0 V -> TEMP_MIN, 5 V -> TEMP_MAX (degrees C)
#define TEMP_MIN      80
#define TEMP_MAX      120
#define TEMP_STEP     1      // setpoint resolution in degrees C
#define HEATER_OFF_AT_MIN 1  // 1 = pot fully at minimum switches the heater off

// ---- Heater drive ----
// Average voltage on the heater while it is heating, as a percentage of the
// supply voltage (0 - 100). Set at compile time.
#define HEATER_VOLTAGE_PERCENT 100

// Fast PWM frequency on D9/D10 (Timer1, Uno/Nano). Allowed values:
//   31372, 3921, 490, 122, 30
// Use 31372 or 3921 for smooth "average voltage" behaviour. Make sure your
// MOSFET / gate driver can switch that fast (low-side logic-level MOSFET
// directly on a pin is fine at 490-3921 Hz; use a gate driver for 31 kHz).
#define HEATER_PWM_HZ 31372

// Thermostat behaviour: heater runs at the fixed duty above until the
// temperature reaches the setpoint, then switches fully off. It switches on
// again once the temperature drops HEATER_HYST degrees below the setpoint.
#define HEATER_HYST   1.0f   // degrees C (0 = switch back on as soon as t < setpoint)

// Safety
#define TEMP_CUTOFF   (TEMP_MAX + 20) // heater forced off above this temperature
#define STALL_TIME_S  1200    // 0 disables. Latch fault if heating this long...
#define STALL_MIN_RISE 2.0f   // ...without rising at least this many degrees
#define STALL_MIN_ERR  5.0f   // ...while at least this far below setpoint
                              // (raise STALL_TIME_S if you use a low voltage percent)

#define SETPOINT_SHOW_MS 2000UL
// ================================================================

#if NUM_SENSORS < 1 || NUM_SENSORS > 2
  #error "NUM_SENSORS must be 1 or 2"
#endif
#if NUM_DISPLAYS < 1 || NUM_DISPLAYS > 2
  #error "NUM_DISPLAYS must be 1 or 2"
#endif
#if NUM_HEATERS < 1 || NUM_HEATERS > 2
  #error "NUM_HEATERS must be 1 or 2"
#endif
#if NUM_HEATERS > NUM_SENSORS || NUM_DISPLAYS > NUM_SENSORS
  #error "Heaters and displays each need a matching sensor"
#endif
#if HEATER_VOLTAGE_PERCENT < 0 || HEATER_VOLTAGE_PERCENT > 100
  #error "HEATER_VOLTAGE_PERCENT must be 0 - 100"
#endif

// analogWrite value (0-255) corresponding to the chosen percentage
#define HEATER_PWM_VALUE ((uint8_t)((HEATER_VOLTAGE_PERCENT * 255L + 50) / 100))

// ---------- Pins (Uno/Nano) ----------
// Shared bus: SCK=D13, MISO=D12 (MAX6675 SO), MOSI=D11 (display DIO)
const uint8_t PIN_CS[2]   = {2, 3};     // MAX6675 chip selects
const uint8_t PIN_RCLK[2] = {8, 7};     // display latches
const uint8_t PIN_PWM[2]  = {9, 10};    // heater MOSFET gates (Timer1)
const uint8_t PIN_POT[2]  = {A0, A1};   // setpoint pots

SPISettings maxSettings(1000000, MSBFIRST, SPI_MODE0);
SPISettings dispSettings(2000000, MSBFIRST, SPI_MODE0);

// ---------- Display data (common anode) ----------
const uint8_t SEG_DIGITS[10] = {
  0xC0, 0xF9, 0xA4, 0xB0, 0x99, 0x92, 0x82, 0xF8, 0x80, 0x90
};
const uint8_t SEG_BLANK = 0xFF;
const uint8_t SEG_DASH  = 0xBF;
const uint8_t SEG_E     = 0x86;
const uint8_t SEG_R     = 0xAF;
const uint8_t DP_MASK   = 0x7F;
const uint8_t DIGIT_SEL[4] = {0x08, 0x04, 0x02, 0x01};

uint8_t displayBuf[NUM_DISPLAYS][4];

// ---------- State ----------
float temperature[NUM_SENSORS];
float setpoint[NUM_HEATERS];
float potFilt[NUM_HEATERS];
unsigned long showSpUntil[NUM_HEATERS];

bool  heating[NUM_HEATERS];     // thermostat state (true = heater driven)
float duty[NUM_HEATERS];        // for status print only
bool  latched[NUM_HEATERS];
unsigned long fullSince[NUM_HEATERS];
float fullTemp[NUM_HEATERS];

unsigned long lastRead = 0;
const unsigned long READ_INTERVAL = 500;   // ms; MAX6675 needs >= 220 ms

// ---------- MAX6675 ----------
float readThermocoupleC(uint8_t idx) {
  uint16_t raw;

  SPI.beginTransaction(maxSettings);
  digitalWrite(PIN_CS[idx], LOW);
  delayMicroseconds(10);
  raw  = SPI.transfer(0x00) << 8;
  raw |= SPI.transfer(0x00);
  digitalWrite(PIN_CS[idx], HIGH);
  SPI.endTransaction();

  if (raw & 0x0004) return NAN;   // thermocouple open
  raw >>= 3;
  return raw * 0.25f;
}

// ---------- Display ----------
void setDisplayError(uint8_t d) {
  for (uint8_t i = 0; i < 4; i++) displayBuf[d][i] = SEG_E;
  displayBuf[0][2] = SEG_DIGITS[0];
  displayBuf[1][2] = SEG_DIGITS[1];
}

void setDisplayFault(uint8_t d) {          // "Err "
  displayBuf[d][0] = SEG_E;
  displayBuf[d][1] = SEG_R;
  displayBuf[d][2] = SEG_R;
  displayBuf[d][3] = SEG_BLANK;
}

void setDisplayTemp(uint8_t d, float tempC) {
  int v = (int)(tempC * 10.0f + 0.5f);     // tenths of a degree
  if (v < 1000) {                          // XX.X
    displayBuf[d][0] = (v >= 100) ? SEG_DIGITS[(v / 100) % 10] : SEG_BLANK;
    displayBuf[d][1] = SEG_DIGITS[(v / 10) % 10] & DP_MASK;
    displayBuf[d][2] = SEG_DIGITS[v % 10];
    displayBuf[d][3] = SEG_BLANK;
  } else {                                 // whole degrees
    v = (int)(tempC + 0.5f);
    displayBuf[d][0] = (v >= 1000) ? SEG_DIGITS[(v / 1000) % 10] : SEG_BLANK;
    displayBuf[d][1] = SEG_DIGITS[(v / 100) % 10];
    displayBuf[d][2] = SEG_DIGITS[(v / 10) % 10];
    displayBuf[d][3] = SEG_DIGITS[v % 10];
  }
}

void updateDisplayBuffers() {
  unsigned long now = millis();
  for (uint8_t d = 0; d < NUM_DISPLAYS; d++) {
    if (d < NUM_HEATERS && latched[d]) {
      setDisplayFault(d);
    } else if (d < NUM_HEATERS && (long)(showSpUntil[d] - now) > 0) {
      setDisplayTemp(d, setpoint[d]);
    } else if (isnan(temperature[d])) {
      setDisplayError(d);
    } else {
      setDisplayTemp(d, temperature[d]);
    }
  }
}

void sendDisplay(uint8_t d, uint8_t seg, uint8_t sel) {
  SPI.beginTransaction(dispSettings);
  digitalWrite(PIN_RCLK[d], LOW);
  SPI.transfer(seg);
  SPI.transfer(sel);
  digitalWrite(PIN_RCLK[d], HIGH);
  SPI.endTransaction();
}

void refreshDisplays() {
  for (uint8_t i = 0; i < 4; i++) {
    for (uint8_t d = 0; d < NUM_DISPLAYS; d++) {
      sendDisplay(d, displayBuf[d][i], DIGIT_SEL[i]);
    }
    delay(2);
  }
  for (uint8_t d = 0; d < NUM_DISPLAYS; d++) sendDisplay(d, SEG_BLANK, 0x00);
}

// ---------- Setpoint pots ----------
float quantizeSetpoint(float filt) {
  float c = TEMP_MIN + (filt / 1023.0f) * (TEMP_MAX - TEMP_MIN);
  float q = roundf(c / TEMP_STEP) * TEMP_STEP;
  return constrain(q, (float)TEMP_MIN, (float)TEMP_MAX);
}

void updateSetpoints() {
  unsigned long now = millis();
  for (uint8_t i = 0; i < NUM_HEATERS; i++) {
    potFilt[i] += (analogRead(PIN_POT[i]) - potFilt[i]) * 0.1f;   // smooth
    float c = TEMP_MIN + (potFilt[i] / 1023.0f) * (TEMP_MAX - TEMP_MIN);
    float q = quantizeSetpoint(potFilt[i]);
    bool atEnd = (q <= TEMP_MIN || q >= TEMP_MAX);
    // Hysteresis so the value doesn't flicker between two steps
    if (q != setpoint[i] && (atEnd || fabsf(c - setpoint[i]) > TEMP_STEP * 0.75f)) {
      setpoint[i] = q;
      showSpUntil[i] = now + SETPOINT_SHOW_MS;
    }
  }
}

// ---------- Heater control ----------
void heaterOff(uint8_t i) {
  heating[i] = false;
  duty[i] = 0;
  fullSince[i] = 0;
  analogWrite(PIN_PWM[i], 0);
}

void heaterOn(uint8_t i) {
  heating[i] = true;
  duty[i] = HEATER_VOLTAGE_PERCENT / 100.0f;
  analogWrite(PIN_PWM[i], HEATER_PWM_VALUE);
}

void controlHeater(uint8_t i, float t, unsigned long now) {
  // Fail-safe cases: latched fault, sensor open, over-temperature
  if (latched[i] || isnan(t) || t >= TEMP_CUTOFF) {
    heaterOff(i);
    return;
  }

  float sp = setpoint[i];
#if HEATER_OFF_AT_MIN
  if (sp <= TEMP_MIN) {
    heaterOff(i);
    return;
  }
#endif

  // Thermostat: off at/above setpoint, on again below (setpoint - hysteresis)
  if (t >= sp) {
    heaterOff(i);
    return;
  }
  if (!heating[i] && t <= sp - HEATER_HYST) {
    heaterOn(i);
  } else if (heating[i]) {
    heaterOn(i);            // keep driving at the fixed duty
  } else {
    return;                 // inside hysteresis band, stay off
  }

#if STALL_TIME_S > 0
  // Heating for a long time with no temperature rise = broken sensor/heater
  if (sp - t > STALL_MIN_ERR) {
    if (fullSince[i] == 0) {
      fullSince[i] = now;
      fullTemp[i] = t;
    } else if (now - fullSince[i] >= STALL_TIME_S * 1000UL) {
      if (t - fullTemp[i] < STALL_MIN_RISE) {
        latched[i] = true;
        heaterOff(i);
        Serial.print(F("FAULT: heater "));
        Serial.print(i + 1);
        Serial.println(F(" not heating, latched off (reset to clear)"));
        return;
      }
      fullSince[i] = now;
      fullTemp[i] = t;
    }
  } else {
    fullSince[i] = 0;
  }
#endif
}

// ---------- Serial status ----------
void printStatus() {
  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    Serial.print(F("S")); Serial.print(i + 1); Serial.print(F(": "));
    if (isnan(temperature[i])) Serial.print(F("open"));
    else { Serial.print(temperature[i], 2); Serial.print(F(" C")); }
    if (i < NUM_HEATERS) {
      Serial.print(F(" | set ")); Serial.print(setpoint[i], 0);
      Serial.print(F(" C | duty ")); Serial.print(duty[i] * 100.0f, 0);
      Serial.print(F("%"));
      if (latched[i]) Serial.print(F(" | FAULT"));
    }
    Serial.println();
  }
}

// ---------- Setup / loop ----------
void setup() {
  Serial.begin(115200);

  // Heaters first, so outputs are driven low as early as possible
  for (uint8_t i = 0; i < 2; i++) {
    pinMode(PIN_PWM[i], OUTPUT);
    digitalWrite(PIN_PWM[i], LOW);
  }

#if defined(__AVR_ATmega328P__)
  // Timer1 (D9/D10) prescaler: analogWrite() keeps working, only the frequency changes
  // (8-bit phase-correct PWM: f = 16 MHz / (prescaler * 510))
  #if HEATER_PWM_HZ == 31372
    TCCR1B = (TCCR1B & 0xF8) | 0x01;   // /1    -> ~31.4 kHz
  #elif HEATER_PWM_HZ == 3921
    TCCR1B = (TCCR1B & 0xF8) | 0x02;   // /8    -> ~3.9 kHz
  #elif HEATER_PWM_HZ == 490
    TCCR1B = (TCCR1B & 0xF8) | 0x03;   // /64   -> ~490 Hz
  #elif HEATER_PWM_HZ == 122
    TCCR1B = (TCCR1B & 0xF8) | 0x04;   // /256  -> ~122 Hz
  #elif HEATER_PWM_HZ == 30
    TCCR1B = (TCCR1B & 0xF8) | 0x05;   // /1024 -> ~30.6 Hz
  #else
    #error "HEATER_PWM_HZ must be 31372, 3921, 490, 122 or 30"
  #endif
#endif

  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    pinMode(PIN_CS[i], OUTPUT);
    digitalWrite(PIN_CS[i], HIGH);
    temperature[i] = NAN;
  }
  for (uint8_t i = 0; i < NUM_DISPLAYS; i++) {
    pinMode(PIN_RCLK[i], OUTPUT);
    digitalWrite(PIN_RCLK[i], HIGH);
    setDisplayError(i);
  }
  for (uint8_t i = 0; i < NUM_HEATERS; i++) {
    potFilt[i] = analogRead(PIN_POT[i]);
    setpoint[i] = quantizeSetpoint(potFilt[i]);
    heaterOff(i);
  }

  SPI.begin();
  Serial.println(F("MAX6675 heater controller (fast PWM thermostat)"));
  delay(500);   // let the MAX6675 stabilize
}

void loop() {
  unsigned long now = millis();

  if (now - lastRead >= READ_INTERVAL) {
    lastRead = now;
    for (uint8_t i = 0; i < NUM_SENSORS; i++) temperature[i] = readThermocoupleC(i);
    for (uint8_t i = 0; i < NUM_HEATERS; i++) controlHeater(i, temperature[i], now);
    printStatus();
  }

  updateSetpoints();
  updateDisplayBuffers();
  refreshDisplays();
}