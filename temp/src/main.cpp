#include <SPI.h>

// ---------- MAX6675 (hardware SPI) ----------
// SCK -> D13, SO -> D12, CS -> D10
const uint8_t PIN_CS = 10;
SPISettings max6675Settings(4000000, MSBFIRST, SPI_MODE0);

// ---------- 74HC595 4-digit display module ----------
const uint8_t PIN_RCLK = 4;
const uint8_t PIN_SCLK = 7;
const uint8_t PIN_DIO  = 8;

// Common-anode segment codes (0 = segment ON), bit order: dp g f e d c b a
const uint8_t SEG_DIGITS[10] = {
  0xC0, 0xF9, 0xA4, 0xB0, 0x99, 0x92, 0x82, 0xF8, 0x80, 0x90
};
const uint8_t SEG_BLANK = 0xFF;
const uint8_t SEG_DASH  = 0xBF;
const uint8_t DP_MASK   = 0x7F;

// Reversed to match your module: index 0 = leftmost digit
const uint8_t DIGIT_SEL[4] = {0x08, 0x04, 0x02, 0x01};

uint8_t displayBuf[4] = {SEG_BLANK, SEG_BLANK, SEG_BLANK, SEG_BLANK};

unsigned long lastRead = 0;
const unsigned long READ_INTERVAL = 500;

float readThermocoupleC() {
  uint16_t raw;

  SPI.beginTransaction(max6675Settings);
  digitalWrite(PIN_CS, LOW);
  delayMicroseconds(10);
  raw  = SPI.transfer(0x00) << 8;
  raw |= SPI.transfer(0x00);
  digitalWrite(PIN_CS, HIGH);
  SPI.endTransaction();

  if (raw & 0x0004) return NAN;   // thermocouple open

  raw >>= 3;
  return raw * 0.25;
}

void setDisplayError() {
  for (uint8_t i = 0; i < 4; i++) displayBuf[i] = SEG_DASH;
}

void setDisplayTemp(float tempC) {
  if (tempC < 100.0) {
    int v = (int)(tempC * 10.0 + 0.5);                      // tenths of a degree
    displayBuf[0] = (v >= 100) ? SEG_DIGITS[(v / 100) % 10] : SEG_BLANK;
    displayBuf[1] = SEG_DIGITS[(v / 10) % 10] & DP_MASK;    // decimal point
    displayBuf[2] = SEG_DIGITS[v % 10];
    displayBuf[3] = SEG_BLANK;
  } else {
    int v = (int)(tempC + 0.5);
    displayBuf[0] = (v >= 1000) ? SEG_DIGITS[(v / 1000) % 10] : SEG_BLANK;
    displayBuf[1] = SEG_DIGITS[(v / 100) % 10];
    displayBuf[2] = SEG_DIGITS[(v / 10) % 10];
    displayBuf[3] = SEG_DIGITS[v % 10];
  }
}

void refreshDisplay() {
  for (uint8_t i = 0; i < 4; i++) {
    digitalWrite(PIN_RCLK, LOW);
    shiftOut(PIN_DIO, PIN_SCLK, MSBFIRST, displayBuf[i]);   // segments
    shiftOut(PIN_DIO, PIN_SCLK, MSBFIRST, DIGIT_SEL[i]);    // digit select
    digitalWrite(PIN_RCLK, HIGH);
    delay(2);
  }
  // Blank to avoid ghosting
  digitalWrite(PIN_RCLK, LOW);
  shiftOut(PIN_DIO, PIN_SCLK, MSBFIRST, SEG_BLANK);
  shiftOut(PIN_DIO, PIN_SCLK, MSBFIRST, 0x00);
  digitalWrite(PIN_RCLK, HIGH);
}

void setup() {
  Serial.begin(9600);

  pinMode(PIN_CS, OUTPUT);
  digitalWrite(PIN_CS, HIGH);
  SPI.begin();

  pinMode(PIN_RCLK, OUTPUT);
  pinMode(PIN_SCLK, OUTPUT);
  pinMode(PIN_DIO, OUTPUT);

  Serial.println("MAX6675 + 74HC595 display");
  delay(500);
}

void loop() {
  unsigned long now = millis();

  if (now - lastRead >= READ_INTERVAL) {
    lastRead = now;
    float tempC = readThermocoupleC();

    if (isnan(tempC)) {
      Serial.println("Error: thermocouple not connected");
      setDisplayError();
    } else {
      Serial.print("Temperature: ");
      Serial.print(tempC, 2);
      Serial.print(" C  (");
      Serial.print(tempC * 9.0 / 5.0 + 32.0, 2);
      Serial.println(" F)");
      setDisplayTemp(tempC);
    }
  }

  refreshDisplay();
}
