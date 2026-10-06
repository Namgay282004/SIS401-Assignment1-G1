// ============================================================================
// SIS401 Programming Assignment 1 - Multi-Sensor IoT Monitoring System
// Group Name: 1 
//
// Group members:
//        Namgay wangchuk 
//        Ranjung Yeshi Norbu 
//        Khem Raj Ghalley 
//        Yonten Kinley Tenzin 
//
// Description:
//   An ESP32 reads three sensor types simultaneously: a DHT22 (digital
//   single-wire) for temperature and humidity, an LDR (analog, ADC1) for light
//   level, and an MPU6050 IMU (I2C) for acceleration, rotation, tilt and motion.
//   Readings are shown live on an SSD1306 OLED and published over Wi-Fi to a
//   Mosquitto MQTT broker using a topic hierarchy plus a combined JSON message.
//   Startup diagnostics report I2C devices, Wi-Fi visibility and sensor faults.
// ============================================================================

#include <Wire.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>

// ============================================================================
// >>> EDIT THESE SETTINGS BEFORE UPLOADING <<<
// ============================================================================
const char* WIFI_SSID      = "CST-HA";        // must be a 2.4 GHz network
const char* WIFI_PASSWORD  = "Hostel@HA#";
const char* MQTT_BROKER    = "10.2.23.29";       // your Mac's IP: run ipconfig getifaddr en0
const int   MQTT_PORT      = 1883;
const char* MQTT_CLIENT_ID = "esp32-sis401-11"; // must be unique per board
// ============================================================================

// ---- Pin definitions --------------------------------------------------------
#define DHT_PIN   4      // DHT22 data line (digital single-wire protocol), labelled D4
#define DHT_TYPE  DHT22
#define LDR_PIN   34     // ADC1 pin: ADC2 pins stop working while Wi-Fi is active
// I2C bus uses the ESP32 defaults: SDA = GPIO21, SCL = GPIO22.
// The OLED and MPU6050 share this bus because their addresses differ.

// ---- I2C addresses ----------------------------------------------------------
#define OLED_ADDR      0x3C
#define MPU_ADDR_LOW   0x68  // MPU6050 with AD0 to GND (or unconnected)
#define MPU_ADDR_HIGH  0x69  // MPU6050 with AD0 to 3V3

// ---- Display size -----------------------------------------------------------
#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT 64

// ---- Timing (ms): each task runs at its own rate using millis() -------------
// delay() would freeze the board and stop mqtt.loop() from running, so every
// task checks its own timer instead and the loop never blocks.
const unsigned long DHT_INTERVAL        = 2000;   // DHT22 cannot be read faster than ~0.5 Hz
const unsigned long FAST_INTERVAL       = 200;    // LDR and MPU6050 sampled quickly
const unsigned long DISPLAY_INTERVAL    = 500;
const unsigned long PUBLISH_INTERVAL    = 5000;
const unsigned long WIFI_RETRY_INTERVAL = 15000;  // gives each Wi-Fi attempt time to finish
const unsigned long MQTT_RETRY_INTERVAL = 5000;
const unsigned long WIFI_TIMEOUT        = 15000;

// ---- Tuning constants -------------------------------------------------------
const int   LDR_SAMPLES        = 10;    // ADC readings averaged to smooth out noise
const int   ADC_MAX            = 4095;  // 12-bit ADC
const float GRAVITY            = 9.81;  // m/s^2, what the MPU6050 reads when still
const float ACCEL_THRESHOLD    = 1.5;   // m/s^2 away from 1 g counts as movement
const float GYRO_THRESHOLD     = 30.0;  // deg/s on any axis counts as movement
const int   DHT_FAIL_THRESHOLD = 3;     // consecutive failures before reporting an error

// ---- MQTT topics ------------------------------------------------------------
const char* TOPIC_BASE   = "assignment1/sensors";
const char* TOPIC_STATUS = "assignment1/status";

// ---- Hardware objects -------------------------------------------------------
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
DHT dht(DHT_PIN, DHT_TYPE);
Adafruit_MPU6050 mpu;
WiFiClient wifiClient;
PubSubClient mqtt(wifiClient);

// ---- Latest readings from all sensors, shared by display/serial/MQTT --------
struct SensorReadings {
  float temperature  = NAN;  // C, NAN until the first valid DHT22 read
  float humidity     = NAN;  // %
  int   lightRaw     = 0;    // 0-4095
  float lightPercent = 0;    // 0-100 %
  float accelX = 0, accelY = 0, accelZ = 0;  // m/s^2
  float gyroX  = 0, gyroY  = 0, gyroZ  = 0;  // deg/s
  float pitch  = 0, roll   = 0;              // degrees
  bool  moving = false;
};
SensorReadings readings;

bool mpuAvailable     = false;
int  dhtFailCount     = 0;      // consecutive failed DHT22 reads
bool dhtHintPrinted   = false;  // wiring help is printed only once

// ---- Task timers ------------------------------------------------------------
unsigned long lastDhtRead       = 0;
unsigned long lastFastRead      = 0;
unsigned long lastDisplayUpdate = 0;
unsigned long lastPublish       = 0;
unsigned long lastWifiAttempt   = 0;
unsigned long lastMqttAttempt   = 0;

// ============================================================================
// Setup
// ============================================================================
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n\n===== SIS401 Assignment 1 - starting =====");

  Wire.begin();  // SDA = 21, SCL = 22
  scanI2C();     // report what is on the I2C bus before using it

  initDisplay();
  showBootMessage("Starting sensors...");
  initSensors();

  showBootMessage("Connecting WiFi...");
  connectWiFi();

  mqtt.setServer(MQTT_BROKER, MQTT_PORT);
  mqtt.setBufferSize(512);  // default 256 bytes is too small for the JSON payload
  showBootMessage("Connecting MQTT...");
  reconnectMqtt();
  lastMqttAttempt = millis();

  Serial.println("===== System initialised =====\n");
}

// ============================================================================
// Main loop: non-blocking scheduler, all sensors run simultaneously
// ============================================================================
void loop() {
  unsigned long now = millis();

  maintainConnections(now);

  if (now - lastFastRead >= FAST_INTERVAL) {
    lastFastRead = now;
    readLdr();
    readMpu();
  }

  if (now - lastDhtRead >= DHT_INTERVAL) {
    lastDhtRead = now;
    readDht();
    printSerial();
  }

  if (now - lastDisplayUpdate >= DISPLAY_INTERVAL) {
    lastDisplayUpdate = now;
    updateDisplay();
  }

  if (now - lastPublish >= PUBLISH_INTERVAL) {
    lastPublish = now;
    publishReadings();
  }
}

// ============================================================================
// Diagnostics helpers
// ============================================================================
bool i2cDevicePresent(uint8_t address) {
  Wire.beginTransmission(address);
  return Wire.endTransmission() == 0;
}

// Lists every device on the I2C bus so wiring problems are visible at boot.
void scanI2C() {
  Serial.println("[I2C] Scanning bus (expect 0x3C = OLED, 0x68/0x69 = MPU6050)...");
  int found = 0;
  for (uint8_t address = 1; address < 127; address++) {
    if (i2cDevicePresent(address)) {
      Serial.printf("[I2C]   device found at 0x%02X\n", address);
      found++;
    }
  }
  if (found == 0) {
    Serial.println("[I2C]   NO devices found - check SDA->GPIO21, SCL->GPIO22, power and GND.");
  }
}

// Converts a Wi-Fi status code into a helpful message.
const char* wifiStatusText(int status) {
  switch (status) {
    case WL_CONNECTED:      return "connected";
    case WL_NO_SSID_AVAIL:  return "network not found - check the name, and that it is 2.4 GHz";
    case WL_CONNECT_FAILED: return "connection failed - check the password";
    case WL_CONNECTION_LOST:return "connection lost";
    case WL_DISCONNECTED:   return "disconnected - wrong password or weak signal";
    case WL_IDLE_STATUS:    return "idle";
    default:                return "unknown status";
  }
}

// Checks whether the configured network is visible before trying to join it.
void checkWifiVisible() {
  Serial.printf("[WiFi] Scanning for '%s'...\n", WIFI_SSID);
  int count = WiFi.scanNetworks();
  bool found = false;

  for (int i = 0; i < count; i++) {
    if (WiFi.SSID(i) == WIFI_SSID) {
      found = true;
      Serial.printf("[WiFi]   '%s' found, signal %d dBm.\n", WIFI_SSID, WiFi.RSSI(i));
    }
  }

  if (!found) {
    Serial.printf("[WiFi]   '%s' NOT visible. Networks the ESP32 can see:\n", WIFI_SSID);
    for (int i = 0; i < count; i++) {
      Serial.printf("[WiFi]     '%s' (%d dBm)\n", WiFi.SSID(i).c_str(), WiFi.RSSI(i));
    }
    Serial.println("[WiFi]   Copy the exact name from this list, or switch to a 2.4 GHz network");
    Serial.println("[WiFi]   (iPhone hotspot: turn on 'Maximize Compatibility').");
  }
  WiFi.scanDelete();
}

// ============================================================================
// Initialisation helpers
// ============================================================================
void initDisplay() {
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println("[OLED] Not found - check wiring and address.");
    while (true) { delay(1000); }  // cannot continue without the display
  }
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  Serial.println("[OLED] Ready.");
}

void showBootMessage(const char* message) {
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("SIS401 Assignment 1");
  display.println();
  display.println(message);
  display.display();
}

void initSensors() {
  dht.begin();
  Serial.println("[DHT22] Started on GPIO4.");

  // 12-bit resolution gives 0-4095. The default 11 dB attenuation already
  // covers the full 0-3.3 V range, so no attenuation change is needed.
  analogReadResolution(12);
  Serial.println("[LDR] Reading from GPIO34.");

  // Detect which address the MPU6050 is using instead of assuming one.
  uint8_t mpuAddress = 0;
  if (i2cDevicePresent(MPU_ADDR_LOW))       mpuAddress = MPU_ADDR_LOW;
  else if (i2cDevicePresent(MPU_ADDR_HIGH)) mpuAddress = MPU_ADDR_HIGH;

  if (mpuAddress != 0 && mpu.begin(mpuAddress)) {
    mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);  // built-in low-pass filter reduces jitter
    mpuAvailable = true;
    Serial.printf("[MPU6050] Ready at 0x%02X.\n", mpuAddress);
  } else {
    // Keep running so the other sensors still work; the OLED shows the error.
    Serial.println("[MPU6050] NOT found at 0x68 or 0x69.");
    Serial.println("[MPU6050]   Check: VCC->3V3, GND->GND, SDA->GPIO21, SCL->GPIO22,");
    Serial.println("[MPU6050]   and that the header pins are soldered to the board.");
  }
}

// ============================================================================
// Sensor reading helpers
// ============================================================================

// Digital sensor: DHT22 over its single-wire protocol.
void readDht() {
  float t = dht.readTemperature();
  float h = dht.readHumidity();

  if (isnan(t) || isnan(h)) {
    // Occasional failed reads are normal; keep the last valid values.
    dhtFailCount++;
    Serial.printf("[DHT22] Read failed (%d in a row) - keeping last valid values.\n", dhtFailCount);

    if (dhtFailCount >= DHT_FAIL_THRESHOLD && !dhtHintPrinted) {
      Serial.println("[DHT22] Repeated failures. Check:");
      Serial.println("[DHT22]   1) DATA wire is on GPIO4 (labelled D4)");
      Serial.println("[DHT22]   2) Pin order: module = + / OUT / -, bare sensor = VCC, DATA, NC, GND");
      Serial.println("[DHT22]   3) Bare 4-pin sensor needs a 10k resistor from DATA to 3V3");
      Serial.println("[DHT22]   4) Breadboard power rails may be split in the middle");
      dhtHintPrinted = true;
    }
    return;
  }

  if (dhtFailCount >= DHT_FAIL_THRESHOLD) Serial.println("[DHT22] Working again.");
  dhtFailCount = 0;
  dhtHintPrinted = false;
  readings.temperature = t;
  readings.humidity    = h;
}

// Analog sensor: LDR in a voltage divider read through the ADC.
void readLdr() {
  long total = 0;
  for (int i = 0; i < LDR_SAMPLES; i++) {
    total += analogRead(LDR_PIN);
  }
  readings.lightRaw = total / LDR_SAMPLES;

  // Wiring is 3V3 -> LDR -> GPIO34 -> 10k -> GND. More light lowers the LDR's
  // resistance, raising the voltage at GPIO34, so a higher reading = brighter.
  readings.lightPercent = readings.lightRaw * 100.0 / ADC_MAX;
}

// I2C sensor: MPU6050 accelerometer + gyroscope.
void readMpu() {
  if (!mpuAvailable) return;

  sensors_event_t accel, gyro, temp;
  mpu.getEvent(&accel, &gyro, &temp);

  readings.accelX = accel.acceleration.x;
  readings.accelY = accel.acceleration.y;
  readings.accelZ = accel.acceleration.z;

  // The library reports rotation in rad/s; deg/s is easier to interpret.
  readings.gyroX = gyro.gyro.x * RAD_TO_DEG;
  readings.gyroY = gyro.gyro.y * RAD_TO_DEG;
  readings.gyroZ = gyro.gyro.z * RAD_TO_DEG;

  // Tilt angles from the direction of gravity (accurate while the board is
  // not being shaken, since only gravity is acting on the accelerometer).
  float ax = readings.accelX, ay = readings.accelY, az = readings.accelZ;
  readings.pitch = atan2(-ax, sqrt(ay * ay + az * az)) * RAD_TO_DEG;
  readings.roll  = atan2(ay, az) * RAD_TO_DEG;

  // Motion detection: total acceleration far from 1 g, or fast rotation.
  float magnitude = sqrt(ax * ax + ay * ay + az * az);
  bool accelMoving = fabs(magnitude - GRAVITY) > ACCEL_THRESHOLD;
  bool gyroMoving  = fabs(readings.gyroX) > GYRO_THRESHOLD ||
                     fabs(readings.gyroY) > GYRO_THRESHOLD ||
                     fabs(readings.gyroZ) > GYRO_THRESHOLD;
  readings.moving = accelMoving || gyroMoving;
}

// ============================================================================
// Output helpers: OLED and Serial
// ============================================================================
void updateDisplay() {
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("SIS401 IoT Monitor");
  display.drawFastHLine(0, 9, SCREEN_WIDTH, SSD1306_WHITE);

  display.setCursor(0, 13);
  if (!isnan(readings.temperature)) {
    display.printf("Temp : %.1f C\n", readings.temperature);
    display.printf("Hum  : %.1f %%\n", readings.humidity);
  } else if (dhtFailCount >= DHT_FAIL_THRESHOLD) {
    display.println("Temp : DHT ERROR");
    display.println("Hum  : check wiring");
  } else {
    display.println("Temp : -- C");
    display.println("Hum  : -- %");
  }
  display.printf("Light: %.0f %%\n", readings.lightPercent);

  if (mpuAvailable) {
    display.printf("Tilt : P%.0f R%.0f deg\n", readings.pitch, readings.roll);
    display.printf("Motion: %s\n", readings.moving ? "MOVING" : "STILL");
  } else {
    display.println("IMU  : not found");
  }

  // Connection status on the bottom line
  display.setCursor(0, 56);
  display.printf("WiFi:%s MQTT:%s",
                 WiFi.status() == WL_CONNECTED ? "OK" : "--",
                 mqtt.connected() ? "OK" : "--");
  display.display();
}

void printSerial() {
  Serial.printf("Temp: %.1f C | Hum: %.1f %% | Light: %.0f %% (raw %d) | "
                "Pitch: %.1f deg | Roll: %.1f deg | "
                "Accel: %.2f, %.2f, %.2f m/s2 | Gyro: %.1f, %.1f, %.1f deg/s | %s\n",
                readings.temperature, readings.humidity,
                readings.lightPercent, readings.lightRaw,
                readings.pitch, readings.roll,
                readings.accelX, readings.accelY, readings.accelZ,
                readings.gyroX, readings.gyroY, readings.gyroZ,
                readings.moving ? "MOVING" : "STILL");
}

// ============================================================================
// Wireless helpers: Wi-Fi and MQTT
// ============================================================================
void connectWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(100);
  checkWifiVisible();

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.printf("[WiFi] Connecting to '%s'", WIFI_SSID);

  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < WIFI_TIMEOUT) {
    delay(500);  // blocking is acceptable here: setup runs only once
    Serial.print(".");
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("[WiFi] Connected. ESP32 IP: %s\n", WiFi.localIP().toString().c_str());
  } else {
    Serial.printf("[WiFi] Failed: %s\n", wifiStatusText(WiFi.status()));
    Serial.println("[WiFi] Will keep retrying in the background.");
  }
  lastWifiAttempt = millis();
}

// Makes a single connection attempt so the loop is never blocked.
bool reconnectMqtt() {
  if (WiFi.status() != WL_CONNECTED) return false;

  Serial.printf("[MQTT] Connecting to broker %s:%d... ", MQTT_BROKER, MQTT_PORT);
  // Last Will: if the ESP32 drops off unexpectedly, the broker itself
  // publishes "offline" to the status topic on our behalf.
  bool connected = mqtt.connect(MQTT_CLIENT_ID, TOPIC_STATUS, 1, true, "offline");

  if (connected) {
    mqtt.publish(TOPIC_STATUS, "online", true);  // retained, so new subscribers see it
    Serial.println("connected.");
  } else {
    Serial.printf("failed (state %d).\n", mqtt.state());
    if (mqtt.state() == -2) {
      Serial.println("[MQTT]   Cannot reach broker: check MQTT_BROKER IP, that mosquitto is");
      Serial.println("[MQTT]   running, same network as the Mac, and the Mac firewall.");
    }
  }
  return connected;
}

// Keeps Wi-Fi and MQTT alive. Retries are spaced out so a connection attempt
// that is still in progress is never interrupted, and sensors keep running.
void maintainConnections(unsigned long now) {
  if (WiFi.status() != WL_CONNECTED) {
    if (now - lastWifiAttempt >= WIFI_RETRY_INTERVAL) {
      lastWifiAttempt = now;
      Serial.printf("[WiFi] Not connected (%s) - retrying...\n", wifiStatusText(WiFi.status()));
      WiFi.disconnect();
      WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    }
    return;
  }

  if (mqtt.connected()) {
    mqtt.loop();
    return;
  }

  if (now - lastMqttAttempt >= MQTT_RETRY_INTERVAL) {
    lastMqttAttempt = now;
    reconnectMqtt();
  }
}

// Publishes one numeric value to assignment1/sensors/<subtopic>.
void publishValue(const char* subtopic, float value, int decimals) {
  char topic[64];
  char payload[16];
  snprintf(topic, sizeof(topic), "%s/%s", TOPIC_BASE, subtopic);
  snprintf(payload, sizeof(payload), "%.*f", decimals, value);
  mqtt.publish(topic, payload);
}

// Publishes a text value to assignment1/sensors/<subtopic>.
void publishText(const char* subtopic, const char* text) {
  char topic[64];
  snprintf(topic, sizeof(topic), "%s/%s", TOPIC_BASE, subtopic);
  mqtt.publish(topic, text);
}

// Writes a float as text, or "null" if it is NaN, so the JSON stays valid.
void formatJsonNumber(char* buffer, size_t size, float value, int decimals) {
  if (isnan(value)) {
    snprintf(buffer, size, "null");
  } else {
    snprintf(buffer, size, "%.*f", decimals, value);
  }
}

void publishReadings() {
  if (!mqtt.connected()) {
    Serial.println("[MQTT] Offline - skipping publish.");
    return;
  }

  // 1) Individual values using a topic hierarchy
  if (!isnan(readings.temperature)) {
    publishValue("temperature", readings.temperature, 1);
    publishValue("humidity", readings.humidity, 1);
  }
  publishValue("light", readings.lightPercent, 0);

  if (mpuAvailable) {
    publishValue("motion/pitch", readings.pitch, 1);
    publishValue("motion/roll", readings.roll, 1);
    publishText("motion/state", readings.moving ? "moving" : "still");
  }

  // 2) All readings combined into one JSON message
  char tempText[12], humText[12];
  formatJsonNumber(tempText, sizeof(tempText), readings.temperature, 1);
  formatJsonNumber(humText, sizeof(humText), readings.humidity, 1);

  char json[400];
  snprintf(json, sizeof(json),
           "{\"temperature\":%s,\"humidity\":%s,"
           "\"light\":%.0f,\"light_raw\":%d,"
           "\"accel\":{\"x\":%.2f,\"y\":%.2f,\"z\":%.2f},"
           "\"gyro\":{\"x\":%.1f,\"y\":%.1f,\"z\":%.1f},"
           "\"pitch\":%.1f,\"roll\":%.1f,\"moving\":%s,"
           "\"uptime_s\":%lu}",
           tempText, humText,
           readings.lightPercent, readings.lightRaw,
           readings.accelX, readings.accelY, readings.accelZ,
           readings.gyroX, readings.gyroY, readings.gyroZ,
           readings.pitch, readings.roll,
           readings.moving ? "true" : "false",
           millis() / 1000);

  char topic[64];
  snprintf(topic, sizeof(topic), "%s/all", TOPIC_BASE);
  mqtt.publish(topic, json);

  Serial.println("[MQTT] Published readings.");
}