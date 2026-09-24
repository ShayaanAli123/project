// Washer-done notifier — Stage 4: WiFi + phone notification on DONE.
//
// Board:  ESP32-S3-DevKitC-1
// Sensor: MPU-6500 on GY-521 (WHO_AM_I=0x70), I2C SDA=21, SCL=47, VCC=3V3.
//
// On the IDLE -> RUNNING -> MAYBE_DONE -> DONE state machine reaching DONE,
// we POST a message to ntfy.sh which pushes a notification to the subscribed
// phone. Notification code is isolated in sendNotification() so it can be
// swapped for Telegram / Pushover / a bot / whatever later without touching
// the detection logic.

#include <Wire.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>

// ---- SECRETS / PER-DEVICE CONFIG (fill these in LOCALLY — do not commit real values) ----
// The ESP32-S3 only supports 2.4 GHz WiFi.
static const char* BINGU_SSID  = "YOUR_WIFI_SSID";
static const char* BINGU_EATS  = "YOUR_WIFI_PASSWORD";

// Pick something unique and unguessable. Anyone who knows your topic can
// send you notifications. In the ntfy app on your phone, subscribe to
// this exact string.
static const char* NTFY_TOPIC  = "your-unique-topic-name-here";

// ---- HARDWARE ----
static const uint8_t I2C_SDA_PIN = 21;
static const uint8_t I2C_SCL_PIN = 47;

static const uint8_t MPU_ADDR         = 0x68;
static const uint8_t REG_PWR_MGMT_1   = 0x6B;
static const uint8_t REG_ACCEL_XOUT_H = 0x3B;

static const float ACCEL_LSB_PER_G = 16384.0f;
static const float G_TO_MS2        = 9.80665f;

static const uint32_t SAMPLE_PERIOD_MS = 100;

// ---- TUNABLE DETECTION PARAMETERS ----
static const float    VIBRATION_THRESHOLD_MS2 = 1.5f;
static const uint32_t STATE_DEBOUNCE_MS       = 3000;   // 3 s — rejects one-off jolts
static const uint32_t DONE_TIMEOUT_MS         = 30000;  // 30 s (TESTING) — raise to 300000 (5 min) for real use

// ---- WIFI PARAMETERS ----
static const uint32_t WIFI_CONNECT_TIMEOUT_MS = 20000;  // wait up to 20 s at boot

// ---- STATE MACHINE ----
enum State { IDLE, RUNNING, MAYBE_DONE, DONE };

State    state            = IDLE;
uint32_t state_entered_ms = 0;
uint32_t last_active_ms   = 0;
uint32_t last_quiet_ms    = 0;
uint32_t last_sample_ms   = 0;

const char* stateName(State s) {
  switch (s) {
    case IDLE:       return "IDLE";
    case RUNNING:    return "RUNNING";
    case MAYBE_DONE: return "MAYBE_DONE";
    case DONE:       return "DONE";
  }
  return "?";
}

void transitionTo(State next) {
  Serial.print(">>> STATE: ");
  Serial.print(stateName(state));
  Serial.print(" -> ");
  Serial.println(stateName(next));
  state = next;
  state_entered_ms = millis();
}

float readVibration() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(REG_ACCEL_XOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, (uint8_t)6);

  uint8_t b[6];
  for (uint8_t i = 0; i < 6; i++) {
    b[i] = Wire.read();
  }

  int16_t raw_x = (int16_t)((b[0] << 8) | b[1]);
  int16_t raw_y = (int16_t)((b[2] << 8) | b[3]);
  int16_t raw_z = (int16_t)((b[4] << 8) | b[5]);

  float ax = (raw_x / ACCEL_LSB_PER_G) * G_TO_MS2;
  float ay = (raw_y / ACCEL_LSB_PER_G) * G_TO_MS2;
  float az = (raw_z / ACCEL_LSB_PER_G) * G_TO_MS2;

  float magnitude = sqrtf(ax * ax + ay * ay + az * az);
  return fabsf(magnitude - G_TO_MS2);
}

void connectWiFi() {
  Serial.print("Connecting to WiFi ");
  Serial.print(BINGU_SSID);
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.begin(BINGU_SSID, BINGU_EATS);

  uint32_t start = millis();
  while (WiFi.status() != WL_CONNECTED &&
         (millis() - start) < WIFI_CONNECT_TIMEOUT_MS) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("Connected. IP: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("WiFi connect timed out. Notifications will be skipped.");
  }
}

// Notification sender. Isolated so the detection code doesn't care what
// service we use — swap the body of this function to move to Telegram etc.
bool sendNotification(const char* title, const char* body) {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("sendNotification: WiFi not connected, attempting reconnect...");
    WiFi.reconnect();
    uint32_t start = millis();
    while (WiFi.status() != WL_CONNECTED && (millis() - start) < 5000) {
      delay(200);
    }
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("sendNotification: reconnect failed, giving up.");
      return false;
    }
  }

  // ntfy.sh serves over HTTPS. We use WiFiClientSecure with cert verification
  // disabled — good enough for a hobby project posting to a public service.
  // For production or sensitive data you'd pin ntfy.sh's root CA instead.
  WiFiClientSecure client;
  client.setInsecure();

  HTTPClient http;
  String url = String("https://ntfy.sh/") + NTFY_TOPIC;
  if (!http.begin(client, url)) {
    Serial.println("sendNotification: http.begin() failed.");
    return false;
  }
  http.addHeader("Title", title);
  http.addHeader("Content-Type", "text/plain");

  int code = http.POST((uint8_t*)body, strlen(body));
  Serial.print("sendNotification: HTTP ");
  Serial.println(code);
  http.end();

  return code >= 200 && code < 300;
}

void setup() {
  Serial.begin(115200);
  while (!Serial) { delay(10); }

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(REG_PWR_MGMT_1);
  Wire.write(0x00);
  if (Wire.endTransmission() != 0) {
    Serial.println("Sensor not responding on I2C. Check wiring / power.");
    while (true) { delay(1000); }
  }

  connectWiFi();

  uint32_t now = millis();
  state_entered_ms = now;
  last_active_ms   = now;
  last_quiet_ms    = now;

  Serial.println("Sensor ready. State: IDLE");
}

void loop() {
  uint32_t now = millis();
  if (now - last_sample_ms < SAMPLE_PERIOD_MS) return;
  last_sample_ms = now;

  float vib = readVibration();
  bool is_active = vib > VIBRATION_THRESHOLD_MS2;

  if (is_active) last_active_ms = now;
  else           last_quiet_ms  = now;

  switch (state) {
    case IDLE:
      if ((now - last_quiet_ms) > STATE_DEBOUNCE_MS) transitionTo(RUNNING);
      break;

    case RUNNING:
      if ((now - last_active_ms) > STATE_DEBOUNCE_MS) transitionTo(MAYBE_DONE);
      break;

    case MAYBE_DONE:
      if ((now - last_quiet_ms) > STATE_DEBOUNCE_MS)          transitionTo(RUNNING);
      else if ((now - last_active_ms) > DONE_TIMEOUT_MS)      transitionTo(DONE);
      break;

    case DONE:
      // Fire the notification exactly once and reset for the next cycle.
      sendNotification("Washer done", "Cycle complete — go swap the load.");
      transitionTo(IDLE);
      break;
  }

  Serial.print("vib=");
  Serial.print(vib, 2);
  Serial.print("  state=");
  Serial.println(stateName(state));
}
