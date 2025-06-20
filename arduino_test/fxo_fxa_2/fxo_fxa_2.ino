/***************************************************************************
  This is an example for the Adafruit SensorLab library
  It will look for a supported magnetometer and output
  uTesla data as well as the hard iron calibration offsets
  
  Written by Limor Fried for Adafruit Industries.
 ***************************************************************************/

#include <Adafruit_FXOS8700.h>
Adafruit_FXOS8700 accelmag = Adafruit_FXOS8700(0x8700A, 0x8700B);

#include <Adafruit_FXAS21002C.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
Adafruit_FXAS21002C gyro = Adafruit_FXAS21002C(0x0021002C);

float AccelAvrX = 0;
float AccelAvrY = 0;
float AccelAvrZ = 0;

int AccelCalibrationNumber;

void setup(void) {
  Serial.begin(115200);
  while (!Serial) delay(10);     // will pause Zero, Leonardo, etc until serial console opens
  
  Serial.println(F("fxo_fxa_acel"));

  if (!gyro.begin()) {
    Serial.println("Ooops, no FXAS21002C detected ... Check your wiring!");
    while (1) delay(1000);
  }

  if (!accelmag.begin()) {
    Serial.println("Ooops, no FXOS8700 detected ... Check your wiring!");
    while (1) delay(1000);
  }

  Serial.println("Starting calibration");
  delay(100);

  sensors_event_t aevent, mevent, gevent;
  accelmag.getEvent(&aevent, &mevent);
  
  for (RateCalibrationNumber=0;
         RateCalibrationNumber<2000; 
         RateCalibrationNumber ++) {
    gyro.getEvent(&gevent);

    RateCalibrationPitch+= gevent.gyro.x;
    RateCalibrationRoll += gevent.gyro.y;
    RateCalibrationYaw  += gevent.gyro.z;
    delay(1);
  }
  RateCalibrationPitch/=2000;
  RateCalibrationRoll /=2000;
  RateCalibrationYaw  /=2000;

// FXO/FXA gyro
//  RateCalibrationPitch=-0.01;
//  RateCalibrationRoll = 0.0;
//  RateCalibrationYaw  = 0.0;

  Serial.print("Cal Pitch(x): ");
  Serial.println(RateCalibrationPitch);

  Serial.print("Cal Roll (y): ");
  Serial.println(RateCalibrationRoll);
  
  Serial.print("Cal Yaw  (z): ");
  Serial.println(RateCalibrationYaw);

  delay(2000);
  Serial.println("Resume reading...");


}

void loop() {
  
  sensors_event_t aevent, mevent, gevent;

  gyro.getEvent(&gevent);
  
  float gyro_x = gevent.gyro.x - RateCalibrationPitch;
  float gyro_y = gevent.gyro.y - RateCalibrationRoll;
  float gyro_z = gevent.gyro.z - RateCalibrationYaw;

  Serial.print("Rotation X: ");
  Serial.print(gyro_x);
  
  Serial.print(", Y: ");
  Serial.print(gyro_y);
  
  Serial.print(", Z: ");
  Serial.print(gyro_z);
  Serial.println(" rad/s");


  Serial.println("");
  delay(100);
}
