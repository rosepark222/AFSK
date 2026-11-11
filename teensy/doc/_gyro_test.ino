#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>


Adafruit_MPU6050 mpu;


float angleZ = 0;


unsigned long previousTime = 0;


void setup() {
  Serial.begin(115200);
  Wire.begin();


  if (!mpu.begin()) {
    Serial.println("MPU6050 not detected!");
    while (1);
  }


  Serial.println("MPU6050 connected!");


  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);


  previousTime = micros();
}


void loop() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);


  // Calculate elapsed time
  unsigned long currentTime = micros();
  float dt = (currentTime - previousTime) / 1000000.0;
  previousTime = currentTime;


  // Calculate Z-axis angle
  angleZ += g.gyro.z * 180.0 / PI * dt;


  // Print angle
  Serial.print("Angle Z: ");
  Serial.print(angleZ);
  Serial.println(" degrees");


  delay(10);
}
