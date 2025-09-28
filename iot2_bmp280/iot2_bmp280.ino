#include "Arduino.h"
#include "LoRa_E22.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>

// LoRa pinleri: TX, RX, M0, M1, AUX
LoRa_E22 e22(3, 4, 5, 6, 7);

Adafruit_BMP280 bmp; // BMP280 sensörü için nesne

float sensorValue = 0.0;

unsigned long previousMillis = 0;
const unsigned long interval = 1000;

void setup() {
  Serial.begin(9600);
  e22.begin();
  Wire.begin();
  delay(500);

  if (!bmp.begin(0x76)) {  // BMP280 I2C adresi 0x76 veya 0x77 olabilir, kartına göre kontrol et
    Serial.println("BMP280 sensörü bulunamadı!");
    while (1);
  }

  // İsteğe bağlı ayarlar
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                  Adafruit_BMP280::SAMPLING_X2,
                  Adafruit_BMP280::SAMPLING_X16,
                  Adafruit_BMP280::FILTER_X16,
                  Adafruit_BMP280::STANDBY_MS_500);

  Serial.println("BMP280 başlatıldı.");
}

void loop() {
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {
    previousMillis += interval;

    // Sıcaklık sensörden oku
    sensorValue = bmp.readTemperature();

    // Float değeri stringe çevir ve IOT1 etiketi ekle
    String message = String("#IOT2:") + String(sensorValue, 2);
    ResponseStatus rs = e22.sendFixedMessage(0, 2, 21, message.c_str(), message.length());

    Serial.print("Gönderilen veri: ");
    Serial.println(message);
    Serial.println(rs.getResponseDescription());
  }

  // Buraya başka işler eklenebilir
}
