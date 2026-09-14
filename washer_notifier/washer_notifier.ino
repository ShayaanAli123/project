// Washer-done notifier — Stage 1: prove the MPU-6050 is alive.
// Reads accelerometer and prints x/y/z to Serial at 115200 baud.
//
// Board:  ESP32-S3-DevKitC-1
// Sensor: MPU-6050 (GY-521), I2C, SDA=GPIO8, SCL=GPIO9, VCC=3V3.

#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// I2C pins. These are the Arduino-core defaults for the ESP32-S3 and are safe
// general-purpose GPIOs (no boot strapping, no peripheral conflicts).
// Do NOT use GPIO 19 or 20 on the S3 — those are the native USB D-/D+ lines
// and will fight with USB CDC Serial. Also note GPIO 22 does not exist on
// the ESP32-S3 (chip GPIOs are 0-21 and 26-48).
static const uint8_t I2C_SDA_PIN = 8;
static const uint8_t I2C_SCL_PIN = 9;

// How often we sample + print. 100 ms = 10 Hz, easy to read in Serial Monitor.
static const uint16_t SAMPLE_PERIOD_MS = 100;

Adafruit_MPU6050 mpu;

void setup() {
  Serial.begin(115200);
  while (!Serial) { delay(10); }  // native USB may not be ready right away

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);

  if (!mpu.begin()) {
    Serial.println("MPU-6050 not found. Check wiring, power (3V3!), and address.");
    while (true) { delay(1000); }  // halt: nothing useful to do without a sensor
  }
  Serial.println("MPU-6050 ready.");
}

void loop() {
  sensors_event_t accel, gyro, temp;
  mpu.getEvent(&accel, &gyro, &temp);

  Serial.print("ax="); Serial.print(accel.acceleration.x, 2);
  Serial.print("  ay="); Serial.print(accel.acceleration.y, 2);
  Serial.print("  az="); Serial.print(accel.acceleration.z, 2);
  Serial.println(" m/s^2");

  delay(SAMPLE_PERIOD_MS);
}
