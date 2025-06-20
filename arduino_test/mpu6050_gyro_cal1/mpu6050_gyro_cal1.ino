#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;


float RateRoll, RatePitch, RateYaw;
float RateCalibrationRoll, RateCalibrationPitch, RateCalibrationYaw;
int RateCalibrationNumber;
//void gyro_signals(void) {
//  Wire.beginTransmission(0x68);
//  Wire.write(0x1A);
//  Wire.write(0x05);
//  Wire.endTransmission(); 
//  Wire.beginTransmission(0x68);
//  Wire.write(0x1B);
//  Wire.write(0x08);
//  Wire.endTransmission();
//  Wire.beginTransmission(0x68);
//  Wire.write(0x43);
//  Wire.endTransmission(); 
//  Wire.requestFrom(0x68,6);
//  int16_t GyroX=Wire.read()<<8 | Wire.read();
//  int16_t GyroY=Wire.read()<<8 | Wire.read();
//  int16_t GyroZ=Wire.read()<<8 | Wire.read();
//  RateRoll=(float)GyroX/65.5;
//  RatePitch=(float)GyroY/65.5;
//  RateYaw=(float)GyroZ/65.5;
//}

void setup() {
  Serial.begin(115200);
  while (!Serial)
    delay(10);
  Serial.println("mpu6050_gyro_cal1");

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
  Serial.println("Starting calibration");
  delay(100);
  
  for (RateCalibrationNumber=0;
         RateCalibrationNumber<2000; 
         RateCalibrationNumber ++) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    
    RateCalibrationRoll+=g.gyro.y;
    RateCalibrationPitch+=g.gyro.x;
    RateCalibrationYaw+=g.gyro.z;
    delay(1);
  }
  RateCalibrationRoll/=2000;
  RateCalibrationPitch/=2000;
  RateCalibrationYaw/=2000;
  
//  RateCalibrationRoll = 0.0;
//  RateCalibrationPitch = -0.04;
//  RateCalibrationYaw = -0.02;


  Serial.print("Cal Roll (y): ");
  Serial.println(RateCalibrationRoll);
  
  Serial.print("Cal Pitch(x): ");
  Serial.println(RateCalibrationPitch);
  
  Serial.print("Cal Yaw  (z): ");
  Serial.println(RateCalibrationYaw);

  delay(2000);
  Serial.println("Resume reading...");
// 
//  Serial.begin(57600);
//  pinMode(13, OUTPUT);
//  digitalWrite(13, HIGH);
//  Wire.setClock(400000);
//  Wire.begin();
//  delay(250);
//  Wire.beginTransmission(0x68); 
//  Wire.write(0x6B);
//  Wire.write(0x00);
//  Wire.endTransmission();
//  
//  for (RateCalibrationNumber=0;
//         RateCalibrationNumber<2000; 
//         RateCalibrationNumber ++) {
//    gyro_signals();
//    RateCalibrationRoll+=RateRoll;
//    RateCalibrationPitch+=RatePitch;
//    RateCalibrationYaw+=RateYaw;
//    delay(1);
//  }
//  RateCalibrationRoll/=2000;
//  RateCalibrationPitch/=2000;
//  RateCalibrationYaw/=2000;   
}
void loop() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  /* Print out the values */
//  Serial.print("Acceleration X: ");
//  Serial.print(a.acceleration.x);
//  Serial.print(", Y: ");
//  Serial.print(a.acceleration.y);
//  Serial.print(", Z: ");
//  Serial.print(a.acceleration.z);
//  Serial.println(" m/s^2");

  float gyro_x = g.gyro.x - RateCalibrationPitch;
  float gyro_y = g.gyro.y - RateCalibrationRoll;
  float gyro_z = g.gyro.z - RateCalibrationYaw;

  Serial.print("Rotation X: ");
  //Serial.print(g.gyro.x);
  Serial.print(gyro_x);
  
  Serial.print(", Y: ");
  //Serial.print(g.gyro.y);
  Serial.print(gyro_y);
  
  Serial.print(", Z: ");
  //Serial.print(g.gyro.z);
  Serial.print(gyro_z);
  Serial.println(" rad/s");

//  Serial.print("Temperature: ");
//  Serial.print(temp.temperature);
//  Serial.println(" degC");

  Serial.println("");
  delay(100);
}
