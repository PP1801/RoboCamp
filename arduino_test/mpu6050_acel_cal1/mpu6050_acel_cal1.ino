#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;

float AccelMinX = 0;
float AccelMaxX = 0;
float AccelMinY = 0;
float AccelMaxY = 0;
float AccelMinZ = 0;
float AccelMaxZ = 0;

int AccelCalibrationNumber;

void setup() {
  Serial.begin(115200);
  while (!Serial)
    delay(10);
  Serial.println("mpu6050_acel_cal1");

  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  
}

void loop() {
  Serial.println("Type key when ready..."); 
  while (!Serial.available()){}  // wait for a character

//  AccelMinX = -9.34;
//  AccelMaxX = 10.3;
//  AccelMinY = -9.93;
//  AccelMaxY = 9.88;
//  AccelMinZ = -10.22;
//  AccelMaxZ = 10.11;
  sensors_event_t accelEvent, g, temp;

  Serial.println("Taking measurements...");
  Serial.print(":0........5"); Serial.println("0......100:");
  Serial.print(":");
  for (AccelCalibrationNumber=0;
         AccelCalibrationNumber<1000;
         AccelCalibrationNumber ++) {
    mpu.getEvent(&accelEvent, &g, &temp);

    if (accelEvent.acceleration.x < AccelMinX) AccelMinX = accelEvent.acceleration.x;
    if (accelEvent.acceleration.x > AccelMaxX) AccelMaxX = accelEvent.acceleration.x;
    
    if (accelEvent.acceleration.y < AccelMinY) AccelMinY = accelEvent.acceleration.y;
    if (accelEvent.acceleration.y > AccelMaxY) AccelMaxY = accelEvent.acceleration.y;
    
    if (accelEvent.acceleration.z < AccelMinZ) AccelMinZ = accelEvent.acceleration.z;
    if (accelEvent.acceleration.z > AccelMaxZ) AccelMaxZ = accelEvent.acceleration.z;

    if (AccelCalibrationNumber % 50 == 0)
    {
      Serial.print(".");
    }
      
    delay(1);
  }
  Serial.println(":");
  Serial.println("Done: ");
  Serial.print("Accel Minimums: "); Serial.print(AccelMinX); Serial.print("  ");Serial.print(AccelMinY); Serial.print("  "); Serial.print(AccelMinZ); Serial.println();
  Serial.print("Accel Maximums: "); Serial.print(AccelMaxX); Serial.print("  ");Serial.print(AccelMaxY); Serial.print("  "); Serial.print(AccelMaxZ); Serial.println();
  Serial.print("Accel Current:  "); Serial.print(accelEvent.acceleration.x); Serial.print("  ");Serial.print(accelEvent.acceleration.y); Serial.print("  ");
  Serial.print(accelEvent.acceleration.z); Serial.println();
  while (Serial.available())
  {
    char inc = Serial.read();  // clear the input buffer
    if (inc == 'm')
    {
      mpu.getEvent(&accelEvent, &g, &temp);
      Serial.print("Accel Current:  "); Serial.print(accelEvent.acceleration.x); Serial.print("  ");Serial.print(accelEvent.acceleration.y);
      Serial.print("  "); Serial.print(accelEvent.acceleration.z); Serial.println();
    }
  }
  

}
