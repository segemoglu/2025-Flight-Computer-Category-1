#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <RTClib.h>  // RTC kütüphanesi
#include <TinyGPS++.h>
#include "Arduino.h"
#include "LoRa_E22.h"
#include <SD.h>
#include <SPI.h>

LoRa_E22 e22(&Serial2, 4, 10, 9);
LoRa_E22 e22_2(&Serial7, 30, 31, 32);

IntervalTimer rtcTimer;
volatile bool rtcFlag = false;

IntervalTimer bno055Timer;
volatile bool bno055Flag = false;

IntervalTimer ms5611Timer;
volatile bool ms5611Flag = false;

IntervalTimer loraTimer;
volatile bool loraFlag = false;

IntervalTimer sendLoRaPacketTimer;
volatile bool sendLoRaPacketFlag = false;





uint8_t myRxBuffer[1024];
uint8_t myTxBuffer[1024];

bool localFlag = false;

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

DataNew dataSend;

//Veri paketi degiskenleri
// Telemetri Verileri
int paket_numarasi;           // <PAKET NUMARASI>
int uydu_statusu;             // <UYDU STATÜSÜ>
char hata_kodu[6];            // <HATA KODU> - 6 karakter + null karakter
char gonderme_saati[17];      // <GÖNDERME SAATİ> - 17 karakter + null karakter

float basinc1;                // <BASINÇ1>
float basinc2;                // <BASINÇ2>
float yukseklik1;             // <YÜKSEKLİK1>
float yukseklik2;             // <YÜKSEKLİK2>
float irtifa_farki;           // <İRTİFA FARKI>
float inis_hizi;              // <İNİŞ HIZI>
float sicaklik;               // <SICAKLIK>
float pil_gerilimi;           // <PİL GERİLİMİ>

float gps1_latitude;          // <GPS1 LATITUDE>
float gps1_longitude;         // <GPS1 LONGITUDE>
float gps1_altitude;          // <GPS1 ALTITUDE>

float pitch;                  // <PITCH>
float roll;                   // <ROLL>
float yaw;                    // <YAW>

char rhrh[4];                 // <RHRH> - 4 karakter + null karakter
float iot_s1_data;            // <IoT S1 DATA>
float iot_s2_data;            // <IoT S2 DATA>

int takim_no;                 // <TAKIM NO>











#define SD_CARD_PIN BUILTIN_SDCARD  // Teensy 4.1 için dahili SD kart pini

RTC_PCF8523 rtc;  // RTC nesnesi

// MS5611 adres ve komutlar
#define MS5611_ADDRESS 0x77
#define CMD_RESET 0x1E
#define CMD_CONV_D1 0x48  // Basınç dönüşümü
#define CMD_CONV_D2 0x58  // Sıcaklık dönüşümü
#define CMD_ADC_READ 0x00
#define CMD_PROM_READ 0xA2
#define STANDARD_PRESSURE 101325

// MS5611 sensör nesnesi
struct CalibrationData {
  uint16_t C1, C2, C3, C4, C5, C6;
};

CalibrationData calibration;
float altitude_offset = 0;
float previous_altitude = -1;
unsigned long previous_time = 0;
float vertical_speed = 0;
unsigned long previousMillisMS5611 = 0;
const long intervalMS5611 = 1000;  // MS5611 için 1 saniye aralık

// GPS ve BNO055 nesneleri
TinyGPSPlus gps;
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

// Zamanlayıcılar için değişkenler
unsigned long previousMillisPrint = 0;  // Veri yazdırma için zamanlayıcı
const long intervalPrint = 1000;  // 1 saniye aralık ile yazdırma


// MS5611 reset fonksiyonu
void reset() {
  Wire.beginTransmission(MS5611_ADDRESS);
  Wire.write(CMD_RESET);
  Wire.endTransmission();
  delay(100);  // Sıfırlama süresi
}

// MS5611 kalibrasyon verilerini okuma
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

// Basınç ve sıcaklık dönüşümünü başlat
void start_conversion(uint8_t command) {
  Wire.beginTransmission(MS5611_ADDRESS);
  Wire.write(command);
  Wire.endTransmission();
  delay(10);  // Dönüşüm süresi
}

// ADC verisini oku
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

// Sıcaklık ve basınç verilerini oku
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

// Yükseklik hesaplama
float calculate_altitude(float pressure) {
  float P_Pa = pressure / 100;  // hPa'dan Pa'ya dönüştür
  return 44330 * (1 - pow(P_Pa / STANDARD_PRESSURE, 0.1903));
}

// Yükseklik ofsetini ayarlama
void set_altitude_offset(float offset) {
  altitude_offset = offset;
}

// Düzgünleştirilmiş yükseklik verisi
float get_adjusted_altitude(float pressure) {
  float raw_altitude = calculate_altitude(pressure);
  return raw_altitude - altitude_offset;
}

// İlk yükseklik verisini al ve ofset olarak ayarla
void initialize_altitude_offset() {
  float temperature, pressure;
  read_temperature_and_pressure(temperature, pressure);
  float initial_altitude = calculate_altitude(pressure);
  set_altitude_offset(initial_altitude);
  previous_altitude = initial_altitude;
  previous_time = millis();
}

// Dikey hız hesaplaması
float calculate_vertical_speed(float altitude) {
  if (previous_altitude != -1) {
    float delta_altitude = altitude - previous_altitude;
    unsigned long current_time = millis();
    float delta_time = (current_time - previous_time) / 1000.0;  // saniye
    return delta_altitude / delta_time;
  }
  return 0;
}

// MS5611 sensör verilerini oku ve işle
void read_MS5611_data() {
  float temperature, pressure;
  read_temperature_and_pressure(temperature, pressure);
  float altitude = get_adjusted_altitude(pressure);
  vertical_speed = calculate_vertical_speed(altitude);

  previous_altitude = altitude;
  previous_time = millis();
}

void readGPS(){
  while (Serial6.available() > 0) {
    char c = Serial6.read();
    gps.encode(c);
  }


}

void readMS5611(){ 
  read_MS5611_data();  // MS5611 sensöründen veri oku

  float temperature, pressure;
  read_temperature_and_pressure(temperature, pressure);
  float altitude = get_adjusted_altitude(pressure);
  float vertical_speed = calculate_vertical_speed(altitude);

  sicaklik = temperature;
  basinc1 = pressure;
  yukseklik1 = altitude;
  inis_hizi = vertical_speed;



    if (gps.location.isValid() && gps.location.isUpdated()) {
    gps1_latitude = gps.location.lat();
    gps1_longitude = gps.location.lng();
    gps1_altitude = gps.altitude.meters();
  } else {
    gps1_latitude = 0;
    gps1_longitude = 0;
    gps1_altitude = 0;
  }
}

void readBNO055(){
  sensors_event_t event;
  bno.getEvent(&event);

  pitch = event.orientation.x;
  roll = event.orientation.y;
  yaw = event.orientation.z;
}

void readRTC(){
  DateTime now = rtc.now();
  formatTarihSaat(gonderme_saati, now);
}

// Ucus bilgisayari fonksiyonlari
void sensorler(){
  // BNO055 verisini sürekli oku


  // RTC verisi al


  // Verileri her saniyede bir kez yazdır
  unsigned long currentMillisPrint = millis();
  if (currentMillisPrint - previousMillisPrint >= intervalPrint) {
    previousMillisPrint = currentMillisPrint;



    // GPS verileri aktarımı


    // BNO055 verileri aktarımı


    // MS5611 verileri aktarımı

  }
}

// Tarih ve saati "GG/AA/YY,SS/DD/SS" formatında doldurur
void formatTarihSaat(char* buffer, DateTime &time) {
  // Gün (2 hane)
  buffer[0] = (time.day() / 10) + '0';    // Onlar basamağı
  buffer[1] = (time.day() % 10) + '0';    // Birler basamağı
  buffer[2] = '/';
  
  // Ay (2 hane)
  buffer[3] = (time.month() / 10) + '0';
  buffer[4] = (time.month() % 10) + '0';
  buffer[5] = '/';
  
  // Yıl (2 hane - son 2 rakam)
  buffer[6] = ((time.year() % 100) / 10) + '0';
  buffer[7] = (time.year() % 10) + '0';
  buffer[8] = ',';
  
  // Saat (2 hane)
  buffer[9] = (time.hour() / 10) + '0';
  buffer[10] = (time.hour() % 10) + '0';
  buffer[11] = '/';
  
  // Dakika (2 hane)
  buffer[12] = (time.minute() / 10) + '0';
  buffer[13] = (time.minute() % 10) + '0';
  buffer[14] = '/';
  
  // Saniye (2 hane)
  buffer[15] = (time.second() / 10) + '0';
  buffer[16] = (time.second() % 10) + '0';
}





void ayirString(String girdi) {
  int baslangic = 0;
  int son = girdi.indexOf('#', baslangic);
  
  while (son != -1) {
    baslangic = son + 1;
    son = girdi.indexOf('#', baslangic);
    
    String parca;
    if (son == -1) {
      // Son parça
      parca = girdi.substring(baslangic - 1);
    } else {
      parca = girdi.substring(baslangic - 1, son);
    }
    
    // Boş olmayan parçaları işle
    if (parca.length() > 0) {
      // Burada parçayı istediğiniz gibi kullanabilirsiniz
      Serial.println("Ayrılan parça: " + parca);
      
      // İsterseniz burada parçaları daha fazla işleyebilirsiniz
      // Örneğin ":" karakterine göre anahtar ve değer ayırabilirsiniz
      int ikiNokta = parca.indexOf(':');
      if (ikiNokta != -1) {
        String anahtar = parca.substring(0, ikiNokta);
        String deger = parca.substring(ikiNokta + 1);
        Serial.print("Anahtar: ");
        Serial.print(anahtar);
        Serial.print(", Değer: ");
        Serial.println(deger);
      }
    }
  }
}

void lora(){
  if (e22.available()>1) {
    ResponseContainer rc = e22.receiveMessage();
    
    if (rc.status.code!=1){
      Serial.println(rc.status.getResponseDescription());
    }else{
      Serial.println(rc.status.getResponseDescription());
      String receivedMsg = rc.data;
      Serial.println(receivedMsg);
    }
  }
}

void sendLoRaPacket() {
  // Struct verilerini doldur
  dataSend.packet_no = paket_numarasi;
  dataSend.status = uydu_statusu;
  strncpy(dataSend.error_code, hata_kodu, 6);
  strncpy(dataSend.time_data, gonderme_saati, 17);
  dataSend.pressure1 = basinc1;
  dataSend.pressure2 = basinc2;
  dataSend.altitude1 = yukseklik1;
  dataSend.altitude2 = yukseklik2;
  dataSend.heigh_difference = irtifa_farki;
  dataSend.vertical_speed = inis_hizi;
  dataSend.temperature = sicaklik;
  dataSend.battery_voltage = pil_gerilimi;
  dataSend.gps_latitude = gps1_latitude;
  dataSend.gps_longitude = gps1_longitude;
  dataSend.gps_altitude = gps1_altitude;
  dataSend.pitch = pitch;
  dataSend.roll = roll;
  dataSend.yaw = yaw;
  strncpy(dataSend.rhrh, rhrh, 15);
  dataSend.iot1 = iot_s1_data;
  dataSend.iot2 = iot_s2_data;
  dataSend.team_no = takim_no;

  // Gönder
  ResponseStatus rs = e22_2.sendFixedMessage(0, 6, 6, &dataSend, sizeof(DataNew));
  
  if (rs.code != 1) {
    Serial.print("Hata: ");
    Serial.println(rs.getResponseDescription());
  } else {
    Serial.println("Veri başarıyla gönderildi.");
  }
}


void createCSVWithHeader() {
  // Dosya zaten varsa bir şey yapma
  if (SD.exists("veriler.csv")) {
    return;
  }
  
  File dataFile = SD.open("veriler.csv", FILE_WRITE);
  if (dataFile) {
    dataFile.println("Paket No;Uydu Statusu;Hata Kodu;Gonderme Saati;Basinc1;Basinc2;Yukseklik1;Yukseklik2;Irtifa Farki;Inis Hizi;Sicaklik;Pil Gerilimi;Latitude;Longitude;Altitude;Pitch;Roll;Yaw;RHRH;IoT S1 Data;IoT S2 Data;Takim No");
    dataFile.close();
    Serial.println("CSV baslik satiri olusturuldu.");
  } else {
    Serial.println("Dosya olusturulamadi!");
  }
}


void writeDataToCSV() {
  // Dosyayı ekleme (append) modunda aç
  File dataFile = SD.open("veriler.csv", FILE_WRITE);
  
  if (dataFile) {
    // Verileri CSV formatında yaz (noktalı virgül ile ayrılmış)
    dataFile.print(paket_numarasi); dataFile.print(";");
    dataFile.print(uydu_statusu); dataFile.print(";");
    dataFile.print(hata_kodu); dataFile.print(";");
    dataFile.print(gonderme_saati); dataFile.print(";");
    dataFile.print(basinc1, 2); dataFile.print(";");
    dataFile.print(basinc2, 2); dataFile.print(";");
    dataFile.print(yukseklik1, 2); dataFile.print(";");
    dataFile.print(yukseklik2, 2); dataFile.print(";");
    dataFile.print(irtifa_farki, 2); dataFile.print(";");
    dataFile.print(inis_hizi, 2); dataFile.print(";");
    dataFile.print(sicaklik, 2); dataFile.print(";");
    dataFile.print(pil_gerilimi, 2); dataFile.print(";");
    dataFile.print(gps1_latitude, 6); dataFile.print(";");
    dataFile.print(gps1_longitude, 6); dataFile.print(";");
    dataFile.print(gps1_altitude, 2); dataFile.print(";");
    dataFile.print(pitch, 2); dataFile.print(";");
    dataFile.print(roll, 2); dataFile.print(";");
    dataFile.print(yaw, 2); dataFile.print(";");
    dataFile.print(rhrh); dataFile.print(";");
    dataFile.print(iot_s1_data, 2); dataFile.print(";");
    dataFile.print(iot_s2_data, 2); dataFile.print(";");
    dataFile.println(takim_no); // Son değerden sonra yeni satır
    
    dataFile.close(); // Dosyayı kapat
    
    Serial.print("Veri SD karta yazildi. Paket No: ");
    Serial.println(paket_numarasi);
  } else {
    Serial.println("Hata: Dosya acilamadi!");
  }
}







//Interruptlar
void rtcInterrupt() {
  noInterrupts();
  rtcFlag = true;
  interrupts();  
}

void ms5611Interrupt() {
  noInterrupts();
  ms5611Flag = true;
  interrupts();  
}

void bno055Interrupt() {
  noInterrupts();
  bno055Flag = true;
  interrupts();  
}

void LoraInterrupt() {
  noInterrupts();
  loraFlag = true;
  interrupts();  
}

void sendLoRaPacketInterrupt() {
  noInterrupts();
  sendLoRaPacketFlag = true;
  interrupts();  
}












void printTelemetryData() {
  Serial.println("------ TELEMETRI VERILERI ------");

  Serial.print("Paket No: ");
  Serial.println(paket_numarasi);

  Serial.print("Uydu Statusu: ");
  Serial.println(uydu_statusu);

  Serial.print("Hata Kodu: ");
  Serial.println(hata_kodu);

  Serial.print("Gonderme Saati: ");
  Serial.println(gonderme_saati);

  Serial.print("Basinc1 (Pa): ");
  Serial.println(basinc1);

  Serial.print("Basinc2 (Pa): ");
  Serial.println(basinc2);

  Serial.print("Yukseklik1 (m): ");
  Serial.println(yukseklik1);

  Serial.print("Yukseklik2 (m): ");
  Serial.println(yukseklik2);

  Serial.print("Irtifa Farki (m): ");
  Serial.println(irtifa_farki);

  Serial.print("Inis Hizi (m/s): ");
  Serial.println(inis_hizi);

  Serial.print("Sicaklik (C): ");
  Serial.println(sicaklik);

  Serial.print("Pil Gerilimi (V): ");
  Serial.println(pil_gerilimi);

  Serial.print("GPS Latitude: ");
  Serial.println(gps1_latitude, 6);

  Serial.print("GPS Longitude: ");
  Serial.println(gps1_longitude, 6);

  Serial.print("GPS Altitude (m): ");
  Serial.println(gps1_altitude);

  Serial.print("Pitch (deg): ");
  Serial.println(pitch);

  Serial.print("Roll (deg): ");
  Serial.println(roll);

  Serial.print("Yaw (deg): ");
  Serial.println(yaw);

  Serial.print("RHRH: ");
  Serial.println(rhrh);

  Serial.print("IoT S1 Data (C): ");
  Serial.println(iot_s1_data);

  Serial.print("IoT S2 Data (C): ");
  Serial.println(iot_s2_data);

  Serial.print("Takim No: ");
  Serial.println(takim_no);

  Serial.println("-------------------------------");
}



void setup() {
  // Seri haberleşmeyi başlat
  Serial.begin(9600);
  Serial2.addMemoryForRead(myRxBuffer, sizeof(myRxBuffer));
  Serial2.addMemoryForWrite(myTxBuffer, sizeof(myTxBuffer));
  Serial6.begin(9600); // GPS için (RX: 0, TX: 1)
  Serial2.begin(9600); // lora için
  Serial7.begin(9600);
  Serial7.addMemoryForRead(myRxBuffer, sizeof(myRxBuffer));
  Serial7.addMemoryForWrite(myTxBuffer, sizeof(myTxBuffer));
  e22.begin(); // lora baslat
  e22_2.begin(); // lora baslat
  Wire.begin();

  // RTC başlatma
  if (!rtc.begin()) {
    Serial.println("PCF8523 bağlanamadı! Bağlantıları kontrol edin.");
    while (1);
  }

  if (!rtc.initialized() || rtc.lostPower()) {
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__))); // Derleme zamanı
  }

  // MS5611 başlatma
  reset();
  calibration = read_calibration_data();
  initialize_altitude_offset();

  // BNO055 başlatma
  if (!bno.begin()) {
    while (1);  // Sonsuz döngüde bekler
  }
  bno.setExtCrystalUse(true);
  delay(1000);  // Sensörün tam olarak başlatılabilmesi için küçük bir gecikme

  if (!SD.begin(SD_CARD_PIN)) {
  Serial.println("SD kart başlatılamadı!");
  return;
  }

  createCSVWithHeader();

  rtcTimer.begin(rtcInterrupt,1000000);
  ms5611Timer.begin(ms5611Interrupt,1000000);
  bno055Timer.begin(bno055Interrupt,1000000);
  sendLoRaPacketTimer.begin(sendLoRaPacketInterrupt,1000000);
  //loraTimer.begin(LoraInterrupt,1000000);
}


void loop() {
  readGPS();
  
  
  noInterrupts();
  localFlag = rtcFlag;
  rtcFlag = false;
  interrupts();

  if(localFlag){
    readRTC();
  }

  

  noInterrupts();
  localFlag = ms5611Flag;
  ms5611Flag = false;
  interrupts();

  if(localFlag){
    readMS5611();
  }

  

  noInterrupts();
  localFlag = bno055Flag;
  bno055Flag = false;
  interrupts();

  if(localFlag){
    readBNO055();
    printTelemetryData();
    writeDataToCSV();
  }
  
  lora();

  noInterrupts();
  localFlag = sendLoRaPacketFlag;
  sendLoRaPacketFlag = false;
  interrupts();

  if(localFlag){
    sendLoRaPacket();

  }

}
