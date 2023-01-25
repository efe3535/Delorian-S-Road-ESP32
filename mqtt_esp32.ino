#include <PubSubClient.h>
#include <WiFi.h>
#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>

TinyGPSPlus gps;
SoftwareSerial ss(4, 2);

const char* ssid = "WIFI_SSID"; // WiFi adiniz
const char* wifi_password = "WIFI_SIFRE"; // WiFi sifreniz
const char reason[128] = "Su borusu patladigindan yol 
kapalidir. Ornek Sebep";
int a = 0;

const char* mqtt_server = "raspberrypi.local";// raspberrypi.local sunucu ipsi (raspberrypi.local ornek olsun diye verilmistir, local domainleri 'resolve' edecek bir mDNS ozelligi bulunmamaktadir.)
const int port = 1883; // default olarak tcp ile mqtt baglantisi 1883 portu uzerinden gerceklestirilir
const char* topic = "esp32/coordinates"; // mqtt topic'imiz, buraya gonderdigimiz veri sunucudaki python scripti tarafindan sqlite3 veritabanina kaydedilir. 
char bufflat[15]; // enlem
char bufflng[15]; // boylam
char msg[256]; // sebep

WiFiClient wifiClient;
PubSubClient client(mqtt_server, port, wifiClient);

static void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do 
  {
    while (ss.available())
      gps.encode(ss.read());
  } while (millis() - start < ms);
}

void connect_MQTT() {
  Serial.print("Aga baglaniliyor -> ");
  Serial.println(ssid);
  ss.begin(9600);
  WiFi.begin(ssid, wifi_password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("wifi baglandi");
  Serial.print("IP adresi: ");
  Serial.println(WiFi.localIP());

  if (client.connect("esp32client")) {
    Serial.println("mqtt baglandi");
  } else {
    Serial.println("mqtt baglantisi basarisiz");
  }
}

void setup() {
  Serial.begin(115200);
  connect_MQTT();
}

void loop() {
  Serial.setTimeout(2000);
  if (gps.location.isValid()) {
    dtostrf(gps.location.lat(),6,4,bufflat);
    dtostrf(gps.location.lng(),6,4,bufflng);
    strcat(msg, bufflat);
    strcat(msg,","); // enlem ve boylam degerlerini ayiran virgul
    strcat(msg,bufflng);
    strcat(msg,"?"); // sunucuda sebep kismini ayirmak icin '?' kullandik
    strcat(msg,reason);

    if(client.publish(topic, msg)) {
      Serial.println(msg);
    }
    strcpy(msg,"");
  } else {
   if (client.publish(topic, "INVALID")) {
      Serial.println("INVALID"); // gps'imiz hazir degilse veya sinyal cekmiyorsa INVALID gondererek sunucumuzun hata vermesini (ayiracak koordinat ve sebep degeri yok) engelleriz.
   }
  }
  smartDelay(3000);  // 3 saniye bekle ve gps degerlerini guncelleyerek basa don, mqtt ile koordinatlari gonder - kaynak: TinyGPS Plus kutuphanesi ornekleri
}
