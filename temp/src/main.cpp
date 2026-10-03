#include <Arduino.h>
#include <OneWire.h>
#include <DallasTemperature.h>

// Pin, do którego podłączony jest czujnik danych
#define ONE_WIRE_BUS 2

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

void setup() 
{
  Serial.begin(115200);
  sensors.begin();
}

void loop() 
{
  sensors.requestTemperatures(); // Wysyłanie żądania odczytu temperatury
  float tempC = sensors.getTempCByIndex(0); // Pobranie temperatury pierwszego czujnika

  Serial.print("Temperatura: ");
  Serial.print(tempC);
  Serial.println(" °C");

  delay(1000); // Czekaj 1 sekundę
}