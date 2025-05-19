# Bu koddaki her print birer process olarak düşünülmüştür.
# BİSMİLLAHİRRAHMANİRRAHİM
import threading

# Değişkenler, tanımlamalar vs vs.




# Paket alma, yollama yapısı

def baslangic():
    print("Statü = 0")

def sensor():
    print("Tüm sensörler başlatıldı.")
    print("Tüm sensör verileri alınıp global değişkenlere kaydedildi")

def lora ():
    ayristir()
    print("Gelen paketler yerel degiskenlere kaydedildi.")
    kaydetSD()
    print("Yer istasyonuna paket gönderildi.")

def paketle():
    print("Yerel degiskenler paket haline getirildi.")
    print("Ayrilma zamanı sensörden okundu ve paket bilgisi için SD kart kontrol edildi.")

def kaydetSD():
    print("Yerel degiskenlerdeki veriler SD karta kaydedildi.")

def ayristir():
    print("Gelen paketin hangi birimden geldiği ayirt edildi.")

def aras():
    print("Uydu hizi kontrol edildi ve aras degiskeni guncellendi.")
    print("Gorev yuku inis hizi kontrol edildi ve aras degiskeni guncellendi.")
    print("Tasiyici basinc verisi kontrol edildi ve aras degiskeni guncellendi.")
    print("Gorev yuku konum verisi kontrol edildi ve aras degiskeni guncellendi.")
    print("Ayrilma durumu kontrol edildi ve aras degiskeni guncellendi.")
    print("Mekanik filtre kontrol edildi ve aras degiskeni guncellendi.")

def komutUygula():
    print("Gelen komut uygulanmak için ilgili fonksiyon çağırıldı.")

def ayril():
    print("Ayrilma gerceklesti.")
    print("Ayrilma gerçekleşme bilgisi aras verisine kaydedildi.")

def kalibrasyon():
    print("Kalibrasyon gerçekleşti.")

def mekanikFiltre():
    print("Mekanik filtre çalıştırıldı.")
    print("Mekanik filtre dönme bilgisi aras verisine kaydedildi.")

def sesliIkaz():
    print("Buzzer ötmeye başladı.")

def statuKontrol():
    print("Statü güncellendi.")

def genelKontrol():
    print("Uydu yükselmeye başladıysa statü =1, inişe geçtiyse 2, 410m den aşağıdaysa 3, ayrildiysa 4, uydu durduysa 5")
    print("410 metrede ise ayrilma komutunu cagir.")
    ayril()


# Ana kod bloğu

def setup ():
    pass

def loop ():
    pass

setup()
loop()