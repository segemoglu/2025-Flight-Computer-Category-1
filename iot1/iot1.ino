#include "Arduino.h"
#include "LoRa_E22.h"
#include <SoftwareSerial.h>

LoRa_E22 e22(3, 4, 5, 7, 6); // TX, RX, M0, M1, AUX

float sensorValue = 1903.1903;

unsigned long previousMillis = 0;
const unsigned long interval = 1000;

void setup() {
    Serial.begin(9600);
    e22.begin();
    delay(500);
}

void loop() {
    unsigned long currentMillis = millis();

    if (currentMillis - previousMillis >= interval) {
        previousMillis += interval;

        // Float değeri stringe çevir ve IOT2 etiketi ekle
        String message = String("#IOT1:") + String(sensorValue, 2);  // 2 ondalık basamakla
        ResponseStatus rs = e22.sendFixedMessage(0, 2, 6, message.c_str(), message.length());

        Serial.print("Gönderilen veri: ");
        Serial.println(message);
        Serial.println(rs.getResponseDescription());
    }

    // Diğer işler burada yapılabilir (sensör okuma, vs.)
}
