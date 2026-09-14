// Washer-done notifier — Stage 3: state machine IDLE -> RUNNING -> MAYBE_DONE -> DONE.
//
// Board:  ESP32-S3-DevKitC-1
// Sensor: MPU-6500 on GY-521 (WHO_AM_I=0x70), I2C SDA=21, SCL=47, VCC=3V3.
//
// The trick that makes this work on a real washing machine is MAYBE_DONE.
// Real machines go quiet for 3-5 minutes between wash/rinse/spin cycles.
// We don't declare DONE the moment vibration drops — we wait through a long
// quiet timeout that's longer than the longest pause. If activity resumes,
// we go back to RUNNING and reset the timer. Only sustained silence counts.

#include <Wire.h>

static const uint8_t I2C_SDA_PIN = 21;
static const uint8_t I2C_SCL_PIN = 47;

static const uint8_t MPU_ADDR         = 0x68;
static const uint8_t REG_PWR_MGMT_1   = 0x6B;
static const uint8_t REG_ACCEL_XOUT_H = 0x3B;

static const float ACCEL_LSB_PER_G = 16384.0f;
static const float G_TO_MS2        = 9.80665f;

static const uint32_t SAMPLE_PERIOD_MS = 100;  // 10 Hz sampling

// ---- TUNABLE PARAMETERS ----

// Vibration threshold in m/s^2. Above -> "active" sample, below -> "quiet."
// Chosen from bench data: idle noise ~0.3, desk knocks peak ~1.5, real
// motion is well above. Once mounted on your machine, watch a full cycle
// in Serial Monitor and pick a value clearly above idle but below run.
static const float VIBRATION_THRESHOLD_MS2 = 1.5f;

// State hysteresis: how long a run of consecutive active-or-quiet samples
// must persist before we change state. Prevents one-off jolts (door slam,
// someone bumping the machine) from flipping the machine "on" or "off."
static const uint32_t STATE_DEBOUNCE_MS = 3000;  // 3 s

// How long "quiet" must last in MAYBE_DONE before we call it DONE.
// MUST be longer than the longest pause between the machine's cycles.
// Real deployment: 300000 (5 min), or longer if your machine has long pauses.
// BENCH TESTING: 30 s so you can actually see the transition without waiting.
static const uint32_t DONE_TIMEOUT_MS = 30000;   // 30 s (TESTING) — raise to 300000 for real use

// ---- STATE MACHINE ----

enum State {
  IDLE,         // waiting for the machine to start
  RUNNING,      // vibration seen consistently — machine is going
  MAYBE_DONE,   // vibration stopped, but might just be a pause between cycles
  DONE          // quiet long enough — cycle is truly over
};

State state = IDLE;
uint32_t state_entered_ms = 0;

// Timestamps of the most recent above/at-or-below threshold samples.
uint32_t last_active_ms = 0;
uint32_t last_quiet_ms  = 0;

// Non-blocking sample pacing (no delay() in loop() anymore — the state
// timers need millis() to keep flowing while the sketch does other work).
uint32_t last_sample_ms = 0;

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

  uint32_t now = millis();
  state_entered_ms = now;
  last_active_ms = now;
  last_quiet_ms  = now;

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
      // Been consistently active for STATE_DEBOUNCE_MS -> machine started.
      if ((now - last_quiet_ms) > STATE_DEBOUNCE_MS) {
        transitionTo(RUNNING);
      }
      break;

    case RUNNING:
      // Been consistently quiet for STATE_DEBOUNCE_MS -> might be done, watch.
      if ((now - last_active_ms) > STATE_DEBOUNCE_MS) {
        transitionTo(MAYBE_DONE);
      }
      break;

    case MAYBE_DONE:
      // Activity resumed for the debounce window -> was just a pause, still running.
      if ((now - last_quiet_ms) > STATE_DEBOUNCE_MS) {
        transitionTo(RUNNING);
      }
      // Quiet long enough to outlast any inter-cycle pause -> truly done.
      else if ((now - last_active_ms) > DONE_TIMEOUT_MS) {
        transitionTo(DONE);
      }
      break;

    case DONE:
      // Terminal for Stage 3. Stage 4 will fire the phone notification here
      // and reset back to IDLE so we can detect the next load.
      break;
  }

  Serial.print("vib=");
  Serial.print(vib, 2);
  Serial.print("  state=");
  Serial.println(stateName(state));
}
