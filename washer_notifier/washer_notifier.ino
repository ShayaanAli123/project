// Washer-done notifier — Stage 2: collapse x/y/z into one vibration number.
//
// Board:  ESP32-S3-DevKitC-1
// Sensor: MPU-6500 on GY-521 (WHO_AM_I=0x70), I2C SDA=21, SCL=47, VCC=3V3.
//
// The magnitude of the acceleration vector is sqrt(ax^2 + ay^2 + az^2). At
// rest the sensor still feels gravity (~9.8 m/s^2) no matter how it's tilted,
// so magnitude sits near 9.8 when nothing is moving. Subtracting standard
// gravity gives a "vibration" number that hovers near 0 when still and grows
// when the sensor is shaken — that's the signal we'll threshold on later.

#include <Wire.h>

static const uint8_t I2C_SDA_PIN = 21;
static const uint8_t I2C_SCL_PIN = 47;

static const uint8_t MPU_ADDR         = 0x68;
static const uint8_t REG_PWR_MGMT_1   = 0x6B;
static const uint8_t REG_ACCEL_XOUT_H = 0x3B;

static const float ACCEL_LSB_PER_G = 16384.0f;
static const float G_TO_MS2        = 9.80665f;

static const uint16_t SAMPLE_PERIOD_MS = 100;

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
  Serial.println("Sensor ready.");
}

void loop() {
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

  // Length of the acceleration vector (Pythagoras in 3D).
  float magnitude = sqrtf(ax * ax + ay * ay + az * az);

  // Remove the ~9.8 m/s^2 baseline from gravity so vibration sits near 0
  // when still, regardless of how the sensor is oriented on the machine.
  float vibration = fabsf(magnitude - G_TO_MS2);

  Serial.print("mag="); Serial.print(magnitude, 2);
  Serial.print("  vib="); Serial.print(vibration, 2);
  Serial.println(" m/s^2");

  delay(SAMPLE_PERIOD_MS);
}
