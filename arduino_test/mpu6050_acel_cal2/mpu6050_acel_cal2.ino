#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;

float AccelAvrX = 0;
float AccelAvrY = 0;
float AccelAvrZ = 0;

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

//  AccelMinX = -9.31;
//  AccelMaxX = 10.25;
//  AccelMinY = -9.90;
//  AccelMaxY = 9.77;
//  AccelMinZ = -10.18;
//  AccelMaxZ = 10.05;

  sensors_event_t accelEvent, g, temp;

  Serial.println("Taking measurements...");
  Serial.print(":0........5"); Serial.println("0......100:");
  Serial.print(":");
  for (AccelCalibrationNumber=0;
         AccelCalibrationNumber<1000;
         AccelCalibrationNumber ++) {
    mpu.getEvent(&accelEvent, &g, &temp);

    AccelAvrX += accelEvent.acceleration.x;
    AccelAvrY += accelEvent.acceleration.y;
    AccelAvrZ += accelEvent.acceleration.z;
    
    if (AccelCalibrationNumber % 50 == 0)
    {
      Serial.print(".");
    }
      
    delay(1);
  }

  AccelAvrX /= 1000;
  AccelAvrY /= 1000;
  AccelAvrZ /= 1000; 
  
  Serial.println(":");
  Serial.println("Done: ");
  Serial.print("Accel Averages: "); Serial.print(AccelAvrX); Serial.print("  ");Serial.print(AccelAvrY); Serial.print("  "); Serial.print(AccelAvrZ); Serial.println();
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
