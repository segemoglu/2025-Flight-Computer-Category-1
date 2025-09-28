#include "Arduino.h"
#include "LoRa_E22.h"
#include <Wire.h>
#include <MS5611.h>

MS5611 ms5611;

// LoRa pinleri: TX, RX, M0, M1, AUX
LoRa_E22 e22(3, 4, 5, 6, 7);

float sensorValue = 0.0;

unsigned long previousMillis = 0;
const unsigned long interval = 1000;

void setup() {
  Serial.begin(9600);
  e22.begin();
  Wire.begin();
  delay(500);

  if (!ms5611.begin()) {
    Serial.println("MS5611 sensörü bulunamadı!");
    while (1);
  }

  ms5611.setOversampling(OSR_ULTRA_HIGH);
  Serial.println("MS5611 başlatıldı.");
}

void loop() {
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {
    previousMillis += interval;

    ms5611.read();
    sensorValue = ms5611.getTemperature();

    String message = String("#IOT1:") + String(sensorValue, 2);
    ResponseStatus rs = e22.sendFixedMessage(0, 2, 21, message.c_str(), message.length());

    Serial.print("Gönderilen veri: ");
    Serial.println(message);
    Serial.println(rs.getResponseDescription());
  }
}
