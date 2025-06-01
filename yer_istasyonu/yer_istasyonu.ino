#include "LoRa_E22.h"
#include <Arduino.h>

LoRa_E22 e22(&Serial2, 4, 10, 9); // A-7, C-18
LoRa_E22 e22_2(&Serial3, 5, 12, 11); // A-3, C-6

typedef struct {
  int packet_no;
  int status;
  char error_code[6];
  char time_data[17];
  float pressure1;
  float pressure2;
  float altitude1;
  float altitude2;
  float heigh_difference;
  float vertical_speed;
  float temperature;
  float battery_voltage;
  float gps_latitude;
  float gps_longitude;
  float gps_altitude;
  float pitch;
  float roll;
  float yaw;
  char rhrh[4];
  float iot1;
  float iot2;
  int team_no;
} DataNew;

void setup() {
    Serial.begin(9600);
    Serial2.begin(9600);
    Serial3.begin(9600);
    e22.begin();
    e22_2.begin();

    delay(500);
}

void loop() {
  // Seri porttan veri var mı diye kontrol et
  if (Serial.available()) {
    String incoming = Serial.readStringUntil('\n'); // Satır sonuna kadar oku

    // Mesajın başına "#COM:" etiketi ekle
    String messageToSend = "#COM:" + incoming;

    // Maksimum 58 bayt sınırını aşmadığından emin ol
    if (messageToSend.length() <= 239) {
      ResponseStatus rs = e22_2.sendFixedMessage(0, 2, 6, messageToSend.c_str(), messageToSend.length());
      Serial.println("Gönderildi: " + messageToSend);
      Serial.println(rs.getResponseDescription());
    } else {
      Serial.println("HATA: Mesaj çok uzun! Gönderilmedi.");
    }
  }

  if (e22.available() > 0) {
    ResponseStructContainer rsc = e22.receiveMessage(sizeof(DataNew));
    DataNew receivedMessage = *(DataNew*)rsc.data;

    Serial.print(receivedMessage.packet_no);Serial.print("*");
    Serial.print(receivedMessage.status);Serial.print("*");
    Serial.print(receivedMessage.error_code);Serial.print("*");
    Serial.print(receivedMessage.time_data);Serial.print("*");
    Serial.print(receivedMessage.pressure1);Serial.print("*");
    Serial.print(receivedMessage.pressure2);Serial.print("*");
    Serial.print(receivedMessage.altitude1);Serial.print("*");
    Serial.print(receivedMessage.altitude2);Serial.print("*");
    Serial.print(receivedMessage.heigh_difference);Serial.print("*");
    Serial.print(receivedMessage.vertical_speed);Serial.print("*");
    Serial.print(receivedMessage.temperature);Serial.print("*");
    Serial.print(receivedMessage.battery_voltage);Serial.print("*");
    Serial.print(receivedMessage.gps_latitude);Serial.print("*");
    Serial.print(receivedMessage.gps_longitude);Serial.print("*");
    Serial.print(receivedMessage.gps_altitude);Serial.print("*");
    Serial.print(receivedMessage.pitch);Serial.print("*");
    Serial.print(receivedMessage.roll);Serial.print("*");
    Serial.print(receivedMessage.yaw);Serial.print("*");
    Serial.print(receivedMessage.rhrh);Serial.print("*");
    Serial.print(receivedMessage.iot1);Serial.print("*");
    Serial.print(receivedMessage.iot2);Serial.print("*");
    Serial.println(receivedMessage.team_no);
  }
}

