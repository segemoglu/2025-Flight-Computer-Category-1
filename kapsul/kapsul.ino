#include "Arduino.h"
#include "LoRa_E22.h"
#include <SoftwareSerial.h>
#include <Wire.h>

LoRa_E22 e22(3, 4, 5, 7, 6); // TX, RX, M0, M1, AUX

float sensorValue = 1903.1903;

unsigned long previousMillis = 0;
const unsigned long interval = 1000;

// MS5611 I2C adresi
#define MS5611_ADDRESS 0x77

// MS5611 komutları
#define MS5611_RESET 0x1E
#define MS5611_CONVERT_D1_OSR_4096 0x48
#define MS5611_CONVERT_D2_OSR_4096 0x58
#define MS5611_ADC_READ 0x00
#define MS5611_PROM_BASE_ADDR 0xA0

// Kalibrasyon katsayıları
uint16_t C[8];
uint32_t D1, D2;
int32_t TEMP;
int64_t OFF, SENS;
int32_t P;


void setup() {
    Serial.begin(9600);
    e22.begin();
    Wire.begin();

    pinMode(12,INPUT_PULLUP);


  // Sensörü resetle
  resetSensor();
    delay(500);
  
  // Kalibrasyon verilerini oku
  readCalibrationData();
  
  Serial.println("MS5611 Sensörü Hazır!");


}

void loop() {
  unsigned long currentMillis = millis();
  int pinDurumu = digitalRead(12);


  if (pinDurumu == LOW){
    Serial.println("Kapsül Birleşik");
  }else{
    if (currentMillis - previousMillis >= interval) {
    previousMillis += interval;

    // Sıcaklık ve basınç oku
    readTemperatureAndPressure();

    // Sonuçları yazdır
    Serial.print("Sıcaklık: ");
    Serial.print(TEMP / 100.0);
    Serial.println(" °C");
    
    Serial.print("Basınç: ");
    Serial.print(P / 100.0);sensorValue = P/ 100.0;
    Serial.println(" hPa");
    
    // Yükseklik hesapla (deniz seviyesi: 1013.25 hPa)
    float altitude = 44330 * (1.0 - pow((P/100.0) / 1013.25, 0.1903));
    Serial.print("Yükseklik: ");
    Serial.print(altitude);
    Serial.println(" m");
    
    Serial.println("---");

    // Float değeri stringe çevir ve IOT2 etiketi ekle
    String message = String("#KAP:") + String(sensorValue, 2);  // 2 ondalık basamakla
    ResponseStatus rs = e22.sendFixedMessage(0, 2,21, message.c_str(), message.length());

    Serial.print("Gönderilen veri: ");
    Serial.println(message);
    Serial.println(rs.getResponseDescription());
    }
  }


}


void resetSensor() {
  Wire.beginTransmission(MS5611_ADDRESS);
  Wire.write(MS5611_RESET);
  Wire.endTransmission();
}

void readCalibrationData() {
  for (uint8_t i = 0; i < 8; i++) {
    Wire.beginTransmission(MS5611_ADDRESS);
    Wire.write(MS5611_PROM_BASE_ADDR + (i * 2));
    Wire.endTransmission();
    
    Wire.requestFrom(MS5611_ADDRESS, 2);
    if (Wire.available() >= 2) {
      C[i] = (Wire.read() << 8) | Wire.read();
    }
  }
}

void readTemperatureAndPressure() {
  // D1 (basınç) oku
  Wire.beginTransmission(MS5611_ADDRESS);
  Wire.write(MS5611_CONVERT_D1_OSR_4096);
  Wire.endTransmission();
  delay(10); // Dönüşüm için bekle
  
  D1 = readADC();
  
  // D2 (sıcaklık) oku
  Wire.beginTransmission(MS5611_ADDRESS);
  Wire.write(MS5611_CONVERT_D2_OSR_4096);
  Wire.endTransmission();
  delay(10); // Dönüşüm için bekle
  
  D2 = readADC();
  
  // Sıcaklık hesapla
  int32_t dT = D2 - ((uint32_t)C[5] << 8);
  TEMP = 2000 + ((int64_t)dT * C[6]) / 8388608;
  
  // Basınç hesapla
  OFF = ((int64_t)C[2] << 16) + (((int64_t)C[4] * dT) >> 7);
  SENS = ((int64_t)C[1] << 15) + (((int64_t)C[3] * dT) >> 8);
  
  P = (((D1 * SENS) >> 21) - OFF) >> 15;
}

uint32_t readADC() {
  Wire.beginTransmission(MS5611_ADDRESS);
  Wire.write(MS5611_ADC_READ);
  Wire.endTransmission();
  
  Wire.requestFrom(MS5611_ADDRESS, 3);
  if (Wire.available() >= 3) {
    uint32_t result = Wire.read();
    result = (result << 8) | Wire.read();
    result = (result << 8) | Wire.read();
    return result;
  }
  return 0;
}