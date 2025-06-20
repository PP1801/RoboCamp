/***************************************************************************
  This is an example for the Adafruit SensorLab library
  It will look for a supported magnetometer and output
  uTesla data as well as the hard iron calibration offsets
  
  Written by Limor Fried for Adafruit Industries.
 ***************************************************************************/

#include <Adafruit_FXOS8700.h>
Adafruit_FXOS8700 accelmag = Adafruit_FXOS8700(0x8700A, 0x8700B);

sensor_t accel, mag;

sensors_event_t event;
float min_x, max_x, mid_x;
float min_y, max_y, mid_y;
float min_z, max_z, mid_z;


void displaySensorDetails(void) {
  accelmag.getSensor(&accel, &mag);
  Serial.println("------------------------------------");
  Serial.println("ACCELEROMETER");
  Serial.println("------------------------------------");
  Serial.print("Sensor:       ");
  Serial.println(accel.name);
  Serial.print("Driver Ver:   ");
  Serial.println(accel.version);
  Serial.print("Unique ID:    0x");
  Serial.println(accel.sensor_id, HEX);
  Serial.print("Min Delay:    ");
  Serial.print(accel.min_delay);
  Serial.println(" s");
  Serial.print("Max Value:    ");
  Serial.print(accel.max_value, 4);
  Serial.println(" m/s^2");
  Serial.print("Min Value:    ");
  Serial.print(accel.min_value, 4);
  Serial.println(" m/s^2");
  Serial.print("Resolution:   ");
  Serial.print(accel.resolution, 8);
  Serial.println(" m/s^2");
  Serial.println("------------------------------------");
  Serial.println("");
  Serial.println("------------------------------------");
  Serial.println("MAGNETOMETER");
  Serial.println("------------------------------------");
  Serial.print("Sensor:       ");
  Serial.println(mag.name);
  Serial.print("Driver Ver:   ");
  Serial.println(mag.version);
  Serial.print("Unique ID:    0x");
  Serial.println(mag.sensor_id, HEX);
  Serial.print("Min Delay:    ");
  Serial.print(accel.min_delay);
  Serial.println(" s");
  Serial.print("Max Value:    ");
  Serial.print(mag.max_value);
  Serial.println(" uT");
  Serial.print("Min Value:    ");
  Serial.print(mag.min_value);
  Serial.println(" uT");
  Serial.print("Resolution:   ");
  Serial.print(mag.resolution);
  Serial.println(" uT");
  Serial.println("------------------------------------");
  Serial.println("");
}


void setup(void) {
  Serial.begin(115200);
  while (!Serial) delay(10);     // will pause Zero, Leonardo, etc until serial console opens
  
  Serial.println(F("fxos8700_mag_cal1"));
  
  /* Initialise the sensor */
  if (!accelmag.begin()) {
    /* There was a problem detecting the FXOS8700 ... check your connections */
    Serial.println("Ooops, no FXOS8700 detected ... Check your wiring!");
    while (1)
      ;
  }

  /* Set accelerometer range (optional, default is 2G) */
  // accelmag.setAccelRange(ACCEL_RANGE_8G);
  /* Set the sensor mode (optional, default is hybrid mode) */
  // accelmag.setSensorMode(ACCEL_ONLY_MODE);
  /* Set the magnetometer's oversampling ratio (optional, default is 7) */
  // accelmag.setMagOversamplingRatio(MAG_OSR_7);
  /* Set the output data rate (optional, default is 100Hz) */
  // accelmag.setOutputDataRate(ODR_400HZ);

  displaySensorDetails();
  delay(100);
  
  sensors_event_t aevent, mevent;
  accelmag.getEvent(&aevent, &mevent);

  min_x = max_x = mevent.magnetic.x;
  min_y = max_y = mevent.magnetic.y;
  min_z = max_z = mevent.magnetic.z;
  delay(10);
}
// test 1           // test 2
//mid_x = 16.00;    //mid_x = 20.85;
//mid_y = -29.20;   //mid_y = -28.65;
//mid_z = -97.40;   //mid_z = -100.95;
//fld_x = 49.30;    //fld_x = 53.05;
//fld_y = 50.60;    //fld_y = 47.85;
//fld_z = 55.60;    //fld_z = 55.05;

// test 3 (vani)    // test 4 (vani)
//mid_x = 22.20;    //mid_x = 18.80;
//mid_y = -29.25;   //mid_y = -21.55;
//mid_z = -90.80;   //mid_z = -99.00;
//fld_x = 44.90;    //fld_x = 43.40;
//fld_y = 45.15;    //fld_y = 44.35;
//fld_z = 42.90;    //fld_z = 44.90;

// test 5
//mid_x =  22.40;
//mid_y = -19.00;
//mid_z = -100.65;
//fld_x = 48.00;
//fld_y = 47.90;
//fld_z = 53.70;
  
//float m_bias_x = 30.95;
//float m_bias_y = -19.45;
//float m_bias_z = -93.90;

float m_bias_x = 0.0;
float m_bias_y = 0.0;
float m_bias_z = 0.0;

void loop() {
  
  sensors_event_t aevent, mevent;
  accelmag.getEvent(&aevent, &mevent);
  float x = mevent.magnetic.x - m_bias_x;
  float y = mevent.magnetic.y - m_bias_y;
  float z = mevent.magnetic.z - m_bias_z;
  
  //Serial.print("Mag: (");
  Serial.print(x); Serial.print(", ");
  Serial.print(y); Serial.print(", ");
  Serial.print(z); Serial.print(" ");
  //Serial.print(z); Serial.println();
  

  min_x = min(min_x, x);
  min_y = min(min_y, y);
  min_z = min(min_z, z);

  max_x = max(max_x, x);
  max_y = max(max_y, y);
  max_z = max(max_z, z);

  mid_x = (max_x + min_x) / 2;
  mid_y = (max_y + min_y) / 2;
  mid_z = (max_z + min_z) / 2;
  Serial.print(" Hard offset: (");
  Serial.print(mid_x); Serial.print(", ");
  Serial.print(mid_y); Serial.print(", ");
  Serial.print(mid_z); Serial.print(")");  

  Serial.print(" Field: (");
  Serial.print((max_x - min_x)/2); Serial.print(", ");
  Serial.print((max_y - min_y)/2); Serial.print(", ");
  Serial.print((max_z - min_z)/2); Serial.println(")");  

  delay(50); 
}

/*
 * magnetometer_calibration.py
 * 
 * Hard-Iron Offsets (B):
 * [ 12.1489472  -12.285129   -48.09902217]
 * 
 * Correction Matrix (A), normalized2:
 * [[ 0.98522075  0.02561224  0.04331495]
 *  [ 0.02561224  0.93708898 -0.04041048]
 *  [ 0.04331495 -0.04041048  0.80822917]]
 * 
 */
