#include "Arduino.h"
#include "LoRa_E22.h"
#include <SoftwareSerial.h>
#include <Wire.h>

#define MS5611_ADDRESS 0x77
#define CMD_RESET 0x1E
#define CMD_CONV_D1 0x48  // Basınç dönüşümü
#define CMD_CONV_D2 0x58  // Sıcaklık dönüşümü
#define CMD_ADC_READ 0x00
#define CMD_PROM_READ 0xA2
#define STANDARD_PRESSURE 101325

float altitude_offset = 0;
float previous_altitude = -1;
unsigned long previous_time = 0;
float vertical_speed = 0;

struct CalibrationData {
  uint16_t C1, C2, C3, C4, C5, C6;
};

CalibrationData calibration;

LoRa_E22 e22(3, 4, 5, 7, 6); // TX, RX, M0, M1, AUX

float sensorValue = 1903.1903;

unsigned long previousMillis = 0;
unsigned long previousMillis2 = 0;
const unsigned long interval = 1000;
const unsigned long interval2 = 250;

void setup() {
    Serial.begin(9600);
    e22.begin();
    Wire.begin();
    reset();
    calibration = read_calibration_data();
    
    // İlk yükseklik verisini al ve ofset olarak ayarla
    initialize_altitude_offset();
    delay(500);
}

void loop() {
    unsigned long currentMillis = millis();
/*
    if (currentMillis - previousMillis >= interval) {
      previousMillis += interval;

      // Float değeri stringe çevir ve IOT2 etiketi ekle
      String message = String("#KAP:") + String(sensorValue, 2);  // 2 ondalık basamakla
      ResponseStatus rs = e22.sendFixedMessage(0, 2, 6, message.c_str(), message.length());

      Serial.print("Gönderilen veri: ");
      Serial.println(message);
      Serial.println(rs.getResponseDescription());
    }*/

    if (currentMillis - previousMillis2 >= interval2) {
      previousMillis2 += interval2;


      float temperature, pressure;
      
      // Sıcaklık ve basınç verilerini oku
      read_temperature_and_pressure(temperature, pressure);
      
      // Yükseklik hesapla
      float altitude = get_adjusted_altitude(pressure);
      
      // Dikey hız hesapla
      vertical_speed = 0;
      if (previous_altitude != -1) {
        float delta_altitude = altitude - previous_altitude;
        unsigned long current_time = millis();
        float delta_time = (current_time - previous_time) / 1000.0;  // saniye
        vertical_speed = delta_altitude / delta_time;
      }

      previous_altitude = altitude;
      previous_time = millis();

      // Seri port üzerinden veriyi gönder
      Serial.print("Sıcaklık: ");
      Serial.print(temperature);
      Serial.print(" °C, Basınç: ");
      Serial.print(pressure);
      Serial.print(" hPa, Yükseklik: ");
      Serial.print(altitude);
      Serial.print(" m, Dikey Hız: ");
      Serial.print(vertical_speed);
      Serial.println(" m/s");

    }
}


void reset() {
  Wire.beginTransmission(MS5611_ADDRESS);
  Wire.write(CMD_RESET);
  Wire.endTransmission();
  delay(100);  // Sıfırlama süresi
}

CalibrationData read_calibration_data() {
  CalibrationData calData;
  for (uint8_t i = 0; i < 6; i++) {
    Wire.beginTransmission(MS5611_ADDRESS);
    Wire.write(CMD_PROM_READ + (i * 2));
    Wire.endTransmission();
    Wire.requestFrom(MS5611_ADDRESS, 2);
    uint8_t highByte = Wire.read();
    uint8_t lowByte = Wire.read();
    uint16_t value = (highByte << 8) | lowByte;
    
    switch (i) {
      case 0: calData.C1 = value; break;
      case 1: calData.C2 = value; break;
      case 2: calData.C3 = value; break;
      case 3: calData.C4 = value; break;
      case 4: calData.C5 = value; break;
      case 5: calData.C6 = value; break;
    }
  }
  return calData;
}

void start_conversion(uint8_t command) {
  Wire.beginTransmission(MS5611_ADDRESS);
  Wire.write(command);
  Wire.endTransmission();
  delay(10);  // Dönüşüm süresi
}

uint32_t read_adc() {
  Wire.beginTransmission(MS5611_ADDRESS);
  Wire.write(CMD_ADC_READ);
  Wire.endTransmission();
  Wire.requestFrom(MS5611_ADDRESS, 3);
  
  uint32_t adc = 0;
  adc |= (Wire.read() << 16);
  adc |= (Wire.read() << 8);
  adc |= Wire.read();
  
  return adc;
}

void read_temperature_and_pressure(float &temperature, float &pressure) {
  start_conversion(CMD_CONV_D1);  // Basınç dönüşümü
  delay(10);  // Basınç dönüşüm süresi
  uint32_t D1 = read_adc();

  start_conversion(CMD_CONV_D2);  // Sıcaklık dönüşümü
  delay(10);  // Sıcaklık dönüşüm süresi
  uint32_t D2 = read_adc();

  // Kalibrasyon verilerini oku
  int32_t dT = D2 - (int32_t)calibration.C5 * 256;

  // Sıcaklık hesaplama (100 ile bölme işlemi yapılmaz)
  temperature = 2000 + ((int64_t)dT * calibration.C6) / 8388608;

  // Basınç hesaplama
  int64_t OFF = (int64_t)calibration.C2 * 65536 + (int64_t)(calibration.C4 * dT) / 128;
  int64_t SENS = (int64_t)calibration.C1 * 32768 + (int64_t)(calibration.C3 * dT) / 256;
  pressure = (D1 * SENS / 2097152 - OFF) / 32768;

  // Sıcaklık düzeltmeleri
  if (temperature < 2000) {
    int64_t T2 = ((int64_t)dT * dT) / 2147483648;
    int64_t OFF2 = 5 * ((temperature - 2000) * (temperature - 2000)) / 2;
    int64_t SENS2 = 5 * ((temperature - 2000) * (temperature - 2000)) / 4;
    if (temperature < -1500) {
      OFF2 += 7 * ((temperature + 1500) * (temperature + 1500));
      SENS2 += 11 * ((temperature + 1500) * (temperature + 1500)) / 2;
    }
    temperature -= T2;
    OFF -= OFF2;
    SENS -= SENS2;
  }

  temperature = temperature / 100.0;  // °C cinsine dönüştür
  pressure = pressure / 100.0;  // hPa cinsine dönüştür
}

float calculate_altitude(float pressure) {
  float P_Pa = pressure / 100;  // hPa'dan Pa'ya dönüştür
  return 44330 * (1 - pow(P_Pa / STANDARD_PRESSURE, 0.1903));
}

void set_altitude_offset(float offset) {
  altitude_offset = offset;
}

float get_adjusted_altitude(float pressure) {
  float raw_altitude = calculate_altitude(pressure);
  return raw_altitude - altitude_offset;
}

void initialize_altitude_offset() {
  float temperature, pressure;
  read_temperature_and_pressure(temperature, pressure);
  float initial_altitude = calculate_altitude(pressure);
  set_altitude_offset(initial_altitude);
  previous_altitude = initial_altitude;
  previous_time = millis();
}
