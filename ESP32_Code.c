#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <Adafruit_BME280.h>
#include <time.h>

// WiFi
const char* WIFI_SSID     = "hgfbfd";
const char* WIFI_PASSWORD = "12345678";

// HiveMQ Cloud
const char* MQTT_HOST = "666cb063f1aa4e9a97e37ed4cadebf48.s1.eu.hivemq.cloud";
const int   MQTT_PORT = 8883; // TLS MQTT

// Credentials
const char* MQTT_USER = "hgfbfd";
const char* MQTT_PASS = "12345678aA";

// Topics
const char* TOPIC_BME  = "hgfbfd/bme280";
const char* TOPIC_CMD  = "hgfbfd/ir/cmd";
const char* TOPIC_STAT = "hgfbfd/ir/status";

// to STM
static const int STM_RX2 = 16; // ESP32 RX2  <- STM32 TX (PA9)
static const int STM_TX2 = 17; // ESP32 TX2  -> STM32 RX (PA10)
static const int UART_BAUD = 115200;

// I2C
static const int SDA_PIN = 21;
static const int SCL_PIN = 22;

// BME
const uint8_t BME_ADDR = 0x76;

#define USE_INSECURE_TLS

WiFiClientSecure tlsClient;
PubSubClient mqtt(tlsClient);
Adafruit_BME280 bme;

bool bmeOK = false;

unsigned long lastBmeMs = 0;
const unsigned long BME_PERIOD_MS = 2000;

void wifiConnect() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  Serial.print("WiFi connecting");
  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED) {
    delay(300);
    Serial.print(".");
    if (millis() - t0 > 20000) {
      Serial.println("\nWiFi FAILED (20s timeout)");
      return;
    }
  }
  Serial.println("\nWiFi OK. IP = " + WiFi.localIP().toString());
}

void syncTime() {
  configTime(0, 0, "pool.ntp.org", "time.nist.gov");
  Serial.print("Syncing time");
  time_t now = time(nullptr);

  unsigned long t0 = millis();
  while (now < 1700000000) {
    delay(300);
    Serial.print(".");
    now = time(nullptr);
    if (millis() - t0 > 20000) { 
      Serial.println("\nTime sync FAILED (timeout)");
      return;
    }
  }
  Serial.println(" OK");
}

void publishStatus(const String& s) {
  Serial.println(s);
  mqtt.publish(TOPIC_STAT, s.c_str(), false);
}

// MQTT message receive
void onMqttMessage(char* topic, byte* payload, unsigned int length) {
  String t = String(topic);
  String msg;
  msg.reserve(length);

  for (unsigned int i = 0; i < length; i++) msg += (char)payload[i];
  msg.trim();

  Serial.printf("MQTT IN [%s]: %s\n", t.c_str(), msg.c_str());

  if (t == TOPIC_CMD) {
    if (msg.equalsIgnoreCase("SEND0")) {
      Serial2.print("SEND0\n");
      publishStatus("ESP32->STM32: SEND0");
    }
    else if (msg.equalsIgnoreCase("SEND1")) {
      Serial2.print("SEND1\n");
      publishStatus("ESP32->STM32: SEND1");
    }
    else {
      publishStatus("Unknown CMD (use SEND0 or SEND1)");
    }
  }
}

void mqttConnect() {
  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  mqtt.setCallback(onMqttMessage);

#ifdef USE_INSECURE_TLS
  tlsClient.setInsecure();
#else
  tlsClient.setCACert(ROOT_CA);
#endif

  Serial.print("MQTT connecting");
  while (!mqtt.connected()) {
    String clientId = "esp32-" + String((uint32_t)ESP.getEfuseMac(), HEX);

    bool ok = mqtt.connect(clientId.c_str(), MQTT_USER, MQTT_PASS);
    if (ok) {
      Serial.println("...OK");
      mqtt.subscribe(TOPIC_CMD);
      publishStatus("ESP32 online + subscribed to hgfbfd/ir/cmd");
    } else {
      Serial.printf("...FAIL, rc=%d\n", mqtt.state());
      delay(1500);
    }
  }
}

void publishBmeOnce() {
  float temp = bme.readTemperature();
  float hum  = bme.readHumidity();
  float pres = bme.readPressure() / 100.0f;

  if (isnan(temp) || isnan(hum) || isnan(pres)) return;

  char json[128];
  snprintf(json, sizeof(json),
           "{\"temp\":%.2f,\"hum\":%.2f,\"pres\":%.2f}",
           temp, hum, pres);

  mqtt.publish(TOPIC_BME, json, false);
  Serial.printf("MQTT OUT [%s]: %s\n", TOPIC_BME, json);
}

void pumpStm32ToMqtt() {
  static String line;
  while (Serial2.available()) {
    char c = (char)Serial2.read();
    if (c == '\r') continue;

    if (c == '\n') {
      line.trim();
      if (line.length()) {
        Serial.println("STM32: " + line);
        mqtt.publish(TOPIC_STAT, ("STM32: " + line).c_str(), false);
      }
      line = "";
    } else {
      if (line.length() < 120) line += c;
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(200);

  Serial.println("\nBooting...");

  // to STM
  Serial2.begin(UART_BAUD, SERIAL_8N1, STM_RX2, STM_TX2);

  // I2C, BME
  Wire.begin(SDA_PIN, SCL_PIN);
  bmeOK = bme.begin(BME_ADDR);
  if (!bmeOK) {
    Serial.println("BME280 not found! Check wiring/address (0x76/0x77).");
  } else {
    Serial.println("BME280 OK");
  }

  wifiConnect();

#ifndef USE_INSECURE_TLS
  if (WiFi.status() == WL_CONNECTED) syncTime();
#endif

  if (WiFi.status() == WL_CONNECTED) mqttConnect();

  lastBmeMs = millis();
}

void loop() {
  // try to reconnect
  if (WiFi.status() != WL_CONNECTED) {
    wifiConnect();
#ifndef USE_INSECURE_TLS
    if (WiFi.status() == WL_CONNECTED) syncTime();
#endif
    if (WiFi.status() == WL_CONNECTED) mqttConnect();
  }
  if (WiFi.status() == WL_CONNECTED && !mqtt.connected()) {
    mqttConnect();
  }

  mqtt.loop();

  // BME Signal sending
  if (bmeOK && mqtt.connected() && millis() - lastBmeMs >= BME_PERIOD_MS) {
    lastBmeMs = millis();
    publishBmeOnce();
  }

  // STM messages to MQTT
  if (mqtt.connected()) pumpStm32ToMqtt();
}
