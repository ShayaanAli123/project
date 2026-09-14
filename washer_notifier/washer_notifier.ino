// Washer-done notifier — Stage 1: prove the accel sensor is alive.
// Reads accelerometer and prints x/y/z to Serial at 115200 baud.
//
// Board:  ESP32-S3-DevKitC-1
// Sensor: MPU-6500 (cheap "MPU-6050" clone on GY-521 board — WHO_AM_I=0x70).
//         I2C, SDA=GPIO21, SCL=GPIO47, VCC=3V3.
//
// We skip the Adafruit MPU6050 library because it rejects any chip whose
// WHO_AM_I isn't 0x68 (real MPU-6050). The MPU-6500's accel register map is
// identical to the MPU-6050 for our purposes, so we just read registers
// directly over Wire.

#include <Wire.h>

// I2C pins on the ESP32-S3. GPIO 21 and 47 are plain general-purpose GPIOs.
// Do NOT use GPIO 19/20 (native USB D-/D+). GPIO 22-25 do not exist on S3.
static const uint8_t I2C_SDA_PIN = 21;
static const uint8_t I2C_SCL_PIN = 47;

// MPU-6500 / MPU-6050 I2C address and register addresses.
// AD0 pin low -> 0x68 (default on the GY-521 breakout).
static const uint8_t MPU_ADDR         = 0x68;
static const uint8_t REG_PWR_MGMT_1   = 0x6B;  // power-management, bit7=reset, bit6=sleep
static const uint8_t REG_ACCEL_XOUT_H = 0x3B;  // first of 6 accel bytes (X_H, X_L, Y_H, Y_L, Z_H, Z_L)

// Accel scaling. Default range on power-up is +/- 2g, giving 16384 LSB per g.
// Multiply raw / LSB_PER_G by 9.80665 to get m/s^2 (standard gravity).
static const float ACCEL_LSB_PER_G = 16384.0f;
static const float G_TO_MS2        = 9.80665f;

// How often we sample + print. 100 ms = 10 Hz, easy to read in Serial Monitor.
static const uint16_t SAMPLE_PERIOD_MS = 100;

void setup() {
  Serial.begin(115200);
  while (!Serial) { delay(10); }

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);

  // Wake the sensor: clear the sleep bit in PWR_MGMT_1 (default value is 0x40
  // after reset — sleep bit set — so we explicitly write 0x00 to run it on
  // the internal 20 MHz oscillator with sleep off).
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
  // Point the sensor's internal register pointer at ACCEL_XOUT_H, then
  // read 6 consecutive bytes (X, Y, Z each as high byte + low byte, MSB first).
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(REG_ACCEL_XOUT_H);
  Wire.endTransmission(false);  // restart, don't release the bus
  Wire.requestFrom(MPU_ADDR, (uint8_t)6);

  uint8_t b[6];
  for (uint8_t i = 0; i < 6; i++) {
    b[i] = Wire.read();
  }

  // Combine each (high, low) pair into a signed 16-bit int (two's complement).
  int16_t raw_x = (int16_t)((b[0] << 8) | b[1]);
  int16_t raw_y = (int16_t)((b[2] << 8) | b[3]);
  int16_t raw_z = (int16_t)((b[4] << 8) | b[5]);

  float ax = (raw_x / ACCEL_LSB_PER_G) * G_TO_MS2;
  float ay = (raw_y / ACCEL_LSB_PER_G) * G_TO_MS2;
  float az = (raw_z / ACCEL_LSB_PER_G) * G_TO_MS2;

  Serial.print("ax="); Serial.print(ax, 2);
  Serial.print("  ay="); Serial.print(ay, 2);
  Serial.print("  az="); Serial.print(az, 2);
  Serial.println(" m/s^2");

  delay(SAMPLE_PERIOD_MS);
}
