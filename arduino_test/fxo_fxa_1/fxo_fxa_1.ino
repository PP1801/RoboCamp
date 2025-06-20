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


void setup(void) {
  Serial.begin(115200);
  while (!Serial) delay(10);     // will pause Zero, Leonardo, etc until serial console opens
  
  Serial.println(F("fxo_fxa_1"));

  if (!gyro.begin()) {
    Serial.println("Ooops, no FXAS21002C detected ... Check your wiring!");
    while (1) delay(1000);
  }

  if (!accelmag.begin()) {
    Serial.println("Ooops, no FXOS8700 detected ... Check your wiring!");
    while (1) delay(1000);
  }

  Serial.println('\t');

}

float gyro_c_x = -0.01;
float gyro_c_y = 0.0;
float gyro_c_z = 0.0;

float refH = 9.81;
float refL = -9.81;
float refR = refH - refL;

float mid_x =  22.40;
float mid_y = -19.00;
float mid_z = -100.65;

float AccelMinX = -9.92;
float AccelMaxX =  9.99;
float AccelMinY = -9.63;
float AccelMaxY =  9.39;
float AccelMinZ = -9.78;
float AccelMaxZ =  9.68;

// magnetometer Hard-Iron offsets
float mx_o = 12.869;
float my_o = -19.361;
float mz_o = -44.681;
// magnetometer Soft-Iron matrix
float axx = 0.992,  axy = 0.005,  axz = 0.035;
float ayx = 0.005,  ayy = 0.974,  ayz =-0.041;
float azx = 0.035,  azy =-0.041,  azz = 0.856;

void loop() {
  
  sensors_event_t aevent, mevent, gevent;
  accelmag.getEvent(&aevent, &mevent);
  gyro.getEvent(&gevent);
  
//  int ax = aevent.acceleration.x * 1000;
//  int ay = aevent.acceleration.y * 1000;
//  int az = aevent.acceleration.z * 1000;

  float mxc = mevent.magnetic.x - mx_o;
  float myc = mevent.magnetic.y - my_o;
  float mzc = mevent.magnetic.z - mz_o;

  float mx = axx*mxc + axy*myc + axz*mzc;
  float my = ayx*mxc + ayy*myc + ayz*mzc;
  float mz = azx*mxc + azy*myc + azz*mzc;

  float gx = (gevent.gyro.x - gyro_c_x); //* 1000;
  float gy = (gevent.gyro.y - gyro_c_y); //* 1000;
  float gz = (gevent.gyro.z - gyro_c_z); //* 1000;

  float ax = (aevent.acceleration.x - AccelMinX)*refR/(AccelMaxX - AccelMinX) + refL;
  float ay = (aevent.acceleration.y - AccelMinY)*refR/(AccelMaxY - AccelMinY) + refL;
  float az = (aevent.acceleration.z - AccelMinZ)*refR/(AccelMaxZ - AccelMinZ) + refL;

//  float mx = mevent.magnetic.x;
//  float my = mevent.magnetic.y;
//  float mz = mevent.magnetic.z;

//  float gx = gevent.gyro.x;
//  float gy = gevent.gyro.y;
//  float gz = gevent.gyro.z;

  //Serial.print("Raw:");
  Serial.print("Corrected:\t");
  Serial.printf("%.2f",ax);
  Serial.print("\t");
  Serial.printf("%.2f",ay);
  Serial.print("\t");
  Serial.printf("%.2f",az);
  Serial.print("\t");
  Serial.printf("%.2f",gx);
  Serial.print("\t");
  Serial.printf("%.2f",gy);
  Serial.print("\t");
  Serial.printf("%.2f",gz);
  Serial.print("\t");
  Serial.printf("%.2f",mx);
  Serial.print("\t");
  Serial.printf("%.2f",my);
  Serial.print("\t");
  Serial.printf("%.2f",mz);
  Serial.println();
    
  delay(100); 
}
