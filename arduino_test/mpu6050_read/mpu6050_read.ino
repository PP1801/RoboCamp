// Basic demo for accelerometer readings from Adafruit MPU6050

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;

#define TIME_DELAY_MS 100

#define GYRO 1
#define ACCEL 1
#define COR_GYRO 2
#define COR_ACCEL 2
#define TEMP 0

float a_x, a_y, a_z;
float g_x, g_y, g_z;

#if COR_GYRO == 1
float GyroRollY = 0.0;
float GyroPitchX = -0.04;
float GyroYawZ = -0.02;
#elif COR_GYRO == 2
float GyroRollY = 0.015;
float GyroPitchX = -0.04;
float GyroYawZ = -0.01;
#endif

#if COR_ACCEL == 1
// initial (min/max)
//float AccelMinX = -9.30;
//float AccelMaxX = 10.27;
//float AccelMinY = -9.91;
//float AccelMaxY = 9.79;
//float AccelMinZ = -10.24;
//float AccelMaxZ = 10.21;

// custom (intuition) - very good
//float AccelMinX = -9.30;
//float AccelMaxX = 10.24;
//float AccelMinY = -9.90;
//float AccelMaxY = 9.76;
//float AccelMinZ = -10.18;
//float AccelMaxZ = 10.04;

// new (average/1000) - very good
float AccelMinX = -9.31;
float AccelMaxX = 10.25;
float AccelMinY = -9.90;
float AccelMaxY = 9.77;
float AccelMinZ = -10.18;
float AccelMaxZ = 10.05;

#elif COR_ACCEL == 2
float AccelMinX = -9.17;
float AccelMaxX = 10.47;
float AccelMinY = -9.86;
float AccelMaxY = 9.58;
float AccelMinZ = -10.39;
float AccelMaxZ = 9.77;

#endif

float refH = 9.81;
float refL = -9.81;
float refR = refH - refL;

void setup(void) {
  Serial.begin(115200);
  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("mpu6050_read");

  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange()) {
  case MPU6050_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case MPU6050_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case MPU6050_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case MPU6050_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange()) {
  case MPU6050_RANGE_250_DEG:
    Serial.println("+- 250 deg/s");
    break;
  case MPU6050_RANGE_500_DEG:
    Serial.println("+- 500 deg/s");
    break;
  case MPU6050_RANGE_1000_DEG:
    Serial.println("+- 1000 deg/s");
    break;
  case MPU6050_RANGE_2000_DEG:
    Serial.println("+- 2000 deg/s");
    break;
  }

  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth()) {
  case MPU6050_BAND_260_HZ:
    Serial.println("260 Hz");
    break;
  case MPU6050_BAND_184_HZ:
    Serial.println("184 Hz");
    break;
  case MPU6050_BAND_94_HZ:
    Serial.println("94 Hz");
    break;
  case MPU6050_BAND_44_HZ:
    Serial.println("44 Hz");
    break;
  case MPU6050_BAND_21_HZ:
    Serial.println("21 Hz");
    break;
  case MPU6050_BAND_10_HZ:
    Serial.println("10 Hz");
    break;
  case MPU6050_BAND_5_HZ:
    Serial.println("5 Hz");
    break;
  }

  Serial.println("");
  delay(100);
}

void loop() {

  /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

#if COR_ACCEL != 0 and ACCEL == 1 
  a_x = (a.acceleration.x - AccelMinX)*refR/(AccelMaxX - AccelMinX) + refL;
  a_y = (a.acceleration.y - AccelMinY)*refR/(AccelMaxY - AccelMinY) + refL;
  a_z = (a.acceleration.z - AccelMinZ)*refR/(AccelMaxZ - AccelMinZ) + refL;
#else
  a_x = a.acceleration.x;
  a_y = a.acceleration.y;
  a_z = a.acceleration.z;
#endif

#if COR_GYRO != 0 and GYRO == 1
  g_x = g.gyro.x - GyroPitchX;
  g_y = g.gyro.y - GyroRollY;
  g_z = g.gyro.z - GyroYawZ;
#else
  g_x = g.gyro.x;
  g_y = g.gyro.y;
  g_z = g.gyro.z;
#endif

  /* Print out the values */
#if ACCEL == 1
  Serial.print("Accel_X: ");
  Serial.print(a_x); Serial.print("\t");
  Serial.print("Accel_Y: ");
  Serial.print(a_y); Serial.print("\t");
  Serial.print("Accel_Z: ");
  Serial.print(a_z); Serial.print("\t");
  //Serial.println(" m/s^2");
#endif

#if GYRO == 1
  Serial.print("Gyro_X: ");
  Serial.print(g_x); Serial.print("\t");
  Serial.print("Gyro_Y: ");
  Serial.print(g_y); Serial.print("\t");
  Serial.print("Gyro_Z: ");
  Serial.print(g_z); Serial.print("\t");
  //Serial.println(" rad/s");
#endif

#if TEMP == 1
  Serial.print("Temp: ");
  Serial.print(temp.temperature); Serial.print("\t");
  //Serial.println(" degC");
#endif

  Serial.println("");
  delay(TIME_DELAY_MS);

}
