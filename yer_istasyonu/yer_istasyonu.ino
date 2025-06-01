#include "LoRa_E22.h"
#include <Arduino.h>

LoRa_E22 e22(&Serial2, 4, 10, 9); 

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
    e22.begin();

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
      ResponseStatus rs = e22.sendFixedMessage(0, 1, 6, messageToSend.c_str(), messageToSend.length());
      Serial.println("Gönderildi: " + messageToSend);
      Serial.println(rs.getResponseDescription());
    } else {
      Serial.println("HATA: Mesaj çok uzun! Gönderilmedi.");
    }
  }

  if (e22.available() > 0) {
    ResponseStructContainer rsc = e22.receiveMessage(sizeof(DataNew));
    DataNew receivedMessage = *(DataNew*)rsc.data;

    Serial.print("Packet No: ");
    Serial.println(receivedMessage.packet_no);
  }
}

