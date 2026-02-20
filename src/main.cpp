#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

const char* ssid = "fffiachra";
const char* password = "hitamlegam";
const char* mqtt_server = "broker.emqx.io";
const char* mqtt_topic = "julian/message";

WiFiClient espClient;
PubSubClient client(espClient);

void setup_wifi() {
  delay(10);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected");
}

// Fungsi ini dipanggil otomatis saat ada pesan masuk
void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Pesan masuk [");
  Serial.print(topic);
  Serial.print("]: ");

  // Konversi payload ke String
  String messageTemp;
  for (int i = 0; i < length; i++) {
    messageTemp += (char)payload[i];
  }
  Serial.println(messageTemp);

  // Parsing JSON 
  // Alokasi memori untuk JSON (200 byte cukup untuk data kecil)
  StaticJsonDocument<200> doc;
  DeserializationError error = deserializeJson(doc, messageTemp);

  if (error) {
    Serial.print(F("deserializeJson() failed: "));
    Serial.println(error.f_str());
    return;
  }

  // ambil nilai spesifik
  float altitude = doc["alt"];
  float latitude = doc["lat"];
  float longitude = doc["lon"];
  const char* device = doc["device"];

  // logprint
  Serial.println("--- HASIL PARSING ---");
  Serial.print("Device: "); Serial.println(device);
  Serial.print("Alt: "); Serial.println(altitude, 6);
  Serial.print("Lat: "); Serial.println(latitude, 6);
  Serial.print("Lon: "); Serial.println(longitude, 6);
  Serial.println("---------------------");
}

void reconnect() {
  while (!client.connected()) {
    Serial.print("Menghubungkan ke MQTT...");
    String clientId = "ESP32Client-";
    clientId += String(random(0xffff), HEX);
    
    if (client.connect(clientId.c_str())) {
      Serial.println("connected");
      // Subscribe ke topik setelah connect
      client.subscribe(mqtt_topic);
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}

void setup() {
  Serial.begin(115200);
  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback); // Daftarkan fungsi callback
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop(); // agar tetap connect & terima pesan
}