//#include <stlport.h>
#include <Eigen30.h>

#include "ekfNavINS2.h"

//#include <stdio.h>
//#include <signal.h>
//#include <getopt.h>
//#include <time.h>
//#include <rc/mpu.h>
//#include <rc/time.h>

//#include "ekfNavINS.h"
//#include "SparkFun_u-blox_GNSS_Arduino_Library.h"

//#include <iostream>
//#include <fstream>
//#include <string>
//#include <iomanip>
//#include <tuple>

//using namespace std;

#include <Wire.h>

#include <Adafruit_FXOS8700.h>
Adafruit_FXOS8700 accelmag = Adafruit_FXOS8700(0x8700A, 0x8700B);

#include <Adafruit_FXAS21002C.h>
#include <Adafruit_Sensor.h>
Adafruit_FXAS21002C gyro = Adafruit_FXAS21002C(0x0021002C);

#include <Adafruit_GPS.h>
Adafruit_GPS GPS(&Wire);

uint32_t ekf_timer = millis();
uint32_t print_timer = millis();

float gps_init_lat = 0.0, gps_init_lon = 0.0, gps_init_alt = 0.0;

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

float gps_lat = gps_init_lat;
float gps_lon = gps_init_lon;
float gps_alt = gps_init_alt;
float gps_vel_n = 0.0;
float gps_vel_e = 0.0;
float gps_vel_d = 0.0;
float gps_angle = 0.0;
float gps_speed = 0.0;
float gps_old_alt = 0.0;

float KNOTS_TO_MS = 463.0/900.0;
float KMH_TO_MS = 5.0/18.0;
//float DEG_TO_RAD = PI/180.0;
//float RAD_TO_DEG = 180.0/PI;

static int running = 0;

float ax, ay, az, gx, gy, gz, mx, my, mz, pitch, roll, yaw;

const byte redPin = 12;
const byte greenPin = 13;
const byte bluePin = 14;

sensors_event_t aevent, mevent, gevent; 

float adder = 0.01;

// interrupt handler to catch ctrl-c
//static void __signal_handler(__attribute__ ((unused)) int dummy)
//{
//  running=0;
//  return;
//}


//int main(int argc, char *argv[])
//{
//
//
//
//  while (running) {
//
//
//    // update the filter
////    if (gps.getPVT()) {
////      ax = data.accel[0];
////      ay = -1*data.accel[1];
////      az = data.accel[2];
////      hx = data.mag[0]; 
////      hy = data.mag[1]; 
////      hz = data.mag[2];
////      std::tie(pitch,roll,yaw) = ekf.getPitchRollYaw(ax, ay, az, hx, hy, hz);
////      ekf.ekf_update(time(NULL) /*,gps.getTimeOfWeek()*/, gps.getNedNorthVel()*1e-3, gps.getNedEastVel()*1e-3, gps.getNedDownVel()*1e-3,
////        gps.getLatitude()*1e-7*DEG_TO_RAD, gps.getLongitude()*1e-7*DEG_TO_RAD, (gps.getAltitude()*1e-3),
////        data.gyro[0]*DEG_TO_RAD, -1*data.gyro[1]*DEG_TO_RAD, data.gyro[2]*DEG_TO_RAD,
////        ax, ay, az, hx, hy, hz);
////
////      printf("------------------------- %ld -------------------------- \n", gps.getTimeOfWeek());
////      printf("Latitude  : %2.7f %2.7f\n", gps.getLatitude()*1e-7, ekf.getLatitude_rad()*RAD_TO_DEG);
////      printf("Longitute : %2.7f %2.7f\n", gps.getLongitude()*1e-7, ekf.getLongitude_rad()*RAD_TO_DEG);
////      printf("Altitude  : %2.3f %2.3f\n", gps.getAltitude()*1e-3, ekf.getAltitude_m());
////      printf("Speed (N) : %2.3f %2.3f\n", gps.getNedNorthVel()*1e-3, ekf.getVelNorth_ms());
////      printf("Speed (E) : %2.3f %2.3f\n", gps.getNedEastVel()*1e-3, ekf.getVelEast_ms());
////      printf("Speed (D) : %2.3f %2.3f\n", gps.getNedDownVel()*1e-3, ekf.getVelDown_ms());
////      printf("Roll    : %2.3f %2.3f\n", roll, ekf.getRoll_rad());
////      printf("Pitch     : %2.3f %2.3f\n", pitch, ekf.getPitch_rad());
////      printf("Yaw       : %2.3f %2.3f\n", yaw, ekf.getHeading_rad());
////      /*printf("Gyro X    : %f  %f\n", data.gyro[0]*DEG_TO_RAD, ekf.getGyroBiasX_rads());
////      printf("Gyro Y    : %f  %f\n", data.gyro[1]*DEG_TO_RAD, ekf.getGyroBiasY_rads());
////      printf("Gyro Z    : %f  %f\n", data.gyro[2]*DEG_TO_RAD, ekf.getGyroBiasZ_rads());
////      printf("Accel X   : %f  %f\n", data.accel[0], ekf.getAccelBiasX_mss());
////      printf("Accel Y   : %f  %f\n", data.accel[1], ekf.getAccelBiasY_mss());
////      printf("Accel Z   : %f  %f\n", data.accel[2], ekf.getAccelBiasZ_mss());*/
////      printf("-----------------------------------------------------------------\n");
////    }
////    rc_usleep(100000);
//  }
//  printf("\n");
//
//  rc_mpu_power_off();
//  return 0;
//}




void setup() {

//  running = 1;

  Serial.begin(115200);
  while (!Serial) delay(10);     // will pause Zero, Leonardo, etc until serial console opens
  
  Serial.println(F("balam_ekf_nav2"));
  
  pinMode(redPin,   OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin,  OUTPUT);

  set_rgb(0); // off

  Serial.println(F("Setting GPS"));
  GPS.begin(0x10);  // I2C address of the GPS module
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_GGAONLY);
  delay(100);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  delay(1000);

  Serial.println(F("Setting gyro"));
  if (!gyro.begin()) {
    Serial.println(F("Ooops, no FXAS21002C detected ... Check your wiring!"));
    while (1)
      delay(1000);
  }

  Serial.println(F("Setting accelmag"));
  if (!accelmag.begin()) {
    Serial.println(F("Ooops, no FXOS8700 detected ... Check your wiring!"));
    while (1)
      delay(1000);
  }

  //::ekfNavINS ekf;  
}

void loop() {
  
  Serial.println(F("Init ekfNavINS"));
  ekfNavINS ekf;
//  if (running != 1){
//    Serial.println(F("Init ekfNavINS"));
//    extern ekfNavINS ekf;
//    running = 1;  
//  }
  
  /*  GPS NMEA read handle*/
  char gps_dbg = GPS.read();
  if (GPS.newNMEAreceived()) {
    Serial.println(GPS.lastNMEA());
//    if (!GPS.parse(GPS.lastNMEA())) // this also sets the newNMEAreceived() flag to false
//      return; // we can fail to parse a sentence in which case we should just wait for another
//    GPS.parse(GPS.lastNMEA());
//    if (GPS.latitude) {
//      gps_lat = nmeaToDecimal(GPS.latitude, GPS.lat);
//      gps_lon = nmeaToDecimal(GPS.longitude, GPS.lon);  
//    
//      gps_alt = GPS.altitude;
//      gps_angle = GPS.angle*PI/180.0; // * math.PI/180.0;
//      gps_speed = GPS.speed;
//      gps_vel_n = gps_speed * cos(gps_angle);
//      gps_vel_e = gps_speed * sin(gps_angle); 
//      gps_vel_d = gps_alt - gps_old_alt;
//      gps_old_alt = gps_alt;
//    }
//    String nmeaData = GPS.lastNMEA();
//    Serial.print(nmeaData);
//    // Split fused NMEA messages
//    int startIdx = 0;
//    while (true) {
//      int nextDollarIdx = nmeaData.indexOf('$', startIdx + 1); // Find next '$'
//
//      if (nextDollarIdx == -1) {
//        String singleNMEA = nmeaData.substring(startIdx); // Extract last part
//        singleNMEA.trim(); // Trim whitespace in place
//        if (!singleNMEA.isEmpty()) {
//          //handle data
//          gps_lat = GPS.latitude;
//          gps_lon = GPS.longitude;
//          gps_alt = GPS.altitude;
//          gps_angle = GPS.angle; // * math.PI/180.0;
//          gps_speed = GPS.speed;
//          gps_vel_n = gps_speed * cos(gps_angle);
//          gps_vel_e = gps_speed * sin(gps_angle); 
//          gps_vel_d = gps_alt - gps_old_alt;
//          gps_old_alt = gps_alt;
//        }
//        break;
//      }
//      String singleNMEA = nmeaData.substring(startIdx, nextDollarIdx); // Extract part
//      singleNMEA.trim(); // Trim whitespace in place
//      if (!singleNMEA.isEmpty()) {
//        //handle data
//          gps_lat = GPS.latitude;
//          gps_lon = GPS.longitude;
//          gps_alt = GPS.altitude;
//          gps_angle = GPS.angle;
//          gps_speed = GPS.speed;
//      }
//      startIdx = nextDollarIdx;
//    }
  }

  /*  IMU sensor update timer */


  if (millis() - ekf_timer > 10) {  // 10ms -> 100Hz
    ekf_timer = millis(); // reset the timer

    adder += 0.01;

    accelmag.getEvent(&aevent, &mevent);
    gyro.getEvent(&gevent);
    
    float mxc = mevent.magnetic.x - mx_o;
    float myc = mevent.magnetic.y - my_o;
    float mzc = mevent.magnetic.z - mz_o;
  
    mx = axx*mxc + axy*myc + axz*mzc;
    my = ayx*mxc + ayy*myc + ayz*mzc;
    mz = azx*mxc + azy*myc + azz*mzc;
  
    gx = (gevent.gyro.x - gyro_c_x); //* 1000;
    gy = (gevent.gyro.y - gyro_c_y); //* 1000;
    gz = (gevent.gyro.z - gyro_c_z); //* 1000;
  
    ax = (aevent.acceleration.x - AccelMinX)*refR/(AccelMaxX - AccelMinX) + refL;
    ay = (aevent.acceleration.y - AccelMinY)*refR/(AccelMaxY - AccelMinY) + refL;
    az = (aevent.acceleration.z - AccelMinZ)*refR/(AccelMaxZ - AccelMinZ) + refL;

    //if (GPS.latitude) {
      /*  EKF update */
//  ekf.ekf_update(time(NULL) /*,gps.getTimeOfWeek()*/, gps.getNedNorthVel()*1e-3, gps.getNedEastVel()*1e-3, gps.getNedDownVel()*1e-3,
//    gps.getLatitude()*1e-7*DEG_TO_RAD, gps.getLongitude()*1e-7*DEG_TO_RAD, (gps.getAltitude()*1e-3),
//    data.gyro[0]*DEG_TO_RAD, -1*data.gyro[1]*DEG_TO_RAD, data.gyro[2]*DEG_TO_RAD,
//    ax, ay, az, hx, hy, hz);
    Serial.println(F("ekf_update"));
    //std::tie(pitch,roll,yaw) = ekf.getPitchRollYaw(ax, ay, az, mx, my, mz);
//    ekf.ekf_update(0, gps_vel_n, gps_vel_e, gps_vel_d,
//      gps_lat*DEG_TO_RAD, gps_lon*DEG_TO_RAD, gps_alt,
//      gx, -1*gy, gz, //DEG_TO_RAD
//      ax, -1*ay, az,
//      mx, my, mz);
    //}
//    ekf.ekf_update(0, -adder, adder, 0,
//      45.5-adder/100, 15.5+adder/100, 10.0,
//      0.0, 0.0, adder, //DEG_TO_RAD
//      0.0-adder, -9.81, 0.0+adder,
//      45.0, 30.0, -60.0);

//    Serial.println(F("vTaskDelay"));
//    vTaskDelay(1);
    disableCore0WDT();
    disableCore1WDT();
    ekf.ekf_update(0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0,
      0.0, 0.0, 0.0,
      0.0, 0.0, 0.0,
      0.0, 0.0, 0.0); 
    
    Serial.println(F("ekf_done"));
    /*time(NULL) ,gps.getTimeOfWeek()*/
    
  } 


  /*  Print timer */
  if (millis() - print_timer > 2000) {  // 2s -> 0.5Hz
    print_timer = millis(); // reset the timer 
    /*Serial.printf("\n\nLatitude  : ", gps_lat); Serial.println();
    Serial.printf("Longitute : ", gps_lon); Serial.println();
    Serial.printf("Altitude  : %.2f \n", gps_alt);
    Serial.printf("Speed (N) : %2.2f \n", gps_vel_n);
    Serial.printf("Speed (E) : %2.2f \n", gps_vel_e);
    Serial.printf("Speed (D) : %2.2f \n", gps_vel_d);

    Serial.printf("Speed(knt): %2.2f \n", gps_speed);
    Serial.printf("Angle(deg): %2.2f \n", GPS.angle);
    Serial.printf("Alt (old) : %2.2f \n", gps_old_alt);
    //Serial.printf("Roll      : %2.3f \n", roll);
    //Serial.printf("Pitch     : %2.3f \n", pitch);
    //Serial.printf("Yaw       : %2.3f \n", yaw);*/
    Serial.printf("Accel X   : %.3f \n", ax);
    Serial.printf("Accel Y   : %.3f \n", ay);
    Serial.printf("Accel Z   : %.3f \n", az);
    Serial.printf("Gyro  X   : %.3f \n", gx);
    Serial.printf("Gyro  Y   : %.3f \n", gy);
    Serial.printf("Gyro  Z   : %.3f \n", gz);
    Serial.printf("Magneto X : %.3f \n", mx);
    Serial.printf("Magneto Y : %.3f \n", my);
    Serial.printf("Magneto Z : %.3f \n", mz);

    /*Serial.printf("\n\nLatitude  : %2.7f %2.7f\n", gps_lat, ekf.getLatitude_rad()*RAD_TO_DEG);
    Serial.printf("Longitute : %2.7f %2.7f\n", gps_lon, ekf.getLongitude_rad()*RAD_TO_DEG);
    Serial.printf("Altitude  : %2.3f %2.3f\n", gps_alt, ekf.getAltitude_m());
    Serial.printf("Speed (N) : %2.3f %2.3f\n", gps_vel_n, ekf.getVelNorth_ms());
    Serial.printf("Speed (E) : %2.3f %2.3f\n", gps_vel_e, ekf.getVelEast_ms());
    Serial.printf("Speed (D) : %2.3f %2.3f\n", gps_vel_d, ekf.getVelDown_ms());
//    Serial.printf("Roll    : %2.3f %2.3f\n", roll, ekf.getRoll_rad());
//    Serial.printf("Pitch     : %2.3f %2.3f\n", pitch, ekf.getPitch_rad());
//    Serial.printf("Yaw       : %2.3f %2.3f\n", yaw, ekf.getHeading_rad());
    Serial.printf("Accel X   : %f  %f\n", ax, ekf.getAccelBiasX_mss());
    Serial.printf("Accel Y   : %f  %f\n", ay, ekf.getAccelBiasY_mss());
    Serial.printf("Accel Z   : %f  %f\n", az, ekf.getAccelBiasZ_mss());
    Serial.printf("Gyro  X   : %f  %f\n", gx, ekf.getGyroBiasX_rads());
    Serial.printf("Gyro  Y   : %f  %f\n", gy, ekf.getGyroBiasY_rads());
    Serial.printf("Gyro  Z   : %f  %f\n", gz, ekf.getGyroBiasZ_rads());*/
  }

  
}


// Convert NMEA latitude/longitude to decimal degrees
float nmeaToDecimal(float nmeaCoord, char direction) {
  int degrees;
  float minutes;

  if (nmeaCoord < 1000) {
    // Latitude format: ddmm.mmmmm
    degrees = int(nmeaCoord / 100);
    minutes = nmeaCoord - (degrees * 100);
  } else {
    // Longitude format: dddmm.mmmmm
    degrees = int(nmeaCoord / 100);
    minutes = nmeaCoord - (degrees * 100);
  }
  float decimalDegrees = degrees + (minutes / 60.0);

  // Apply hemisphere correction
  if (direction == 'S' || direction == 'W') {
    decimalDegrees *= -1;
  }
  return decimalDegrees;
}



void set_rgb(byte inStage)  // set RGB LED
{
  if (bitRead(inStage, 1)) digitalWrite(redPin, LOW);
  else digitalWrite(redPin, HIGH);
  
  if (bitRead(inStage, 0)) digitalWrite(greenPin, LOW);
  else digitalWrite(greenPin, HIGH);
  
  if (bitRead(inStage, 2)) digitalWrite(bluePin, LOW); 
  else digitalWrite(bluePin, HIGH);  
}
