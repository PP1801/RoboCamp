#include <Wire.h>
#include <Adafruit_Sensor.h>

#include <Adafruit_FXOS8700.h>

#include <Adafruit_FXAS21002C.h>

#include <Adafruit_Simple_AHRS.h>

// Create sensor instances.
//Adafruit_LSM303_Accel_Unified  accel(30301);
//Adafruit_LSM303DLH_Mag_Unified mag(30302);

Adafruit_FXOS8700 accelmag = Adafruit_FXOS8700(0x8700A, 0x8700B);
Adafruit_FXAS21002C gyro = Adafruit_FXAS21002C(0x0021002C);

//sensors_event_t aevent, mevent;

// Create simple AHRS algorithm using the above sensors.
//Adafruit_Simple_AHRS          ahrs(&accel, &mag);
Adafruit_Simple_AHRS          ahrs(&accelmag);
//Adafruit_Simple_AHRS          ahrs(&aevent, &mevent);


void setup()
{
  Serial.begin(115200);
  Serial.println(F("Adafruit 9 DOF Board AHRS Example")); Serial.println("");
  
  // Initialize the sensors.
//  accel.begin();
//  mag.begin();

  if (!accelmag.begin()) {
    Serial.println("Ooops, no FXOS8700 detected ... Check your wiring!");
    while (1) delay(1000);
  }

}

void loop(void)
{
  //sensors_event_t aevent, mevent;
  //accelmag.getEvent(&aevent, &mevent);
  
  sensors_vec_t   orientation;
  
  // Use the simple AHRS function to get the current orientation.
  if (ahrs.getOrientation(&orientation))
  {
    /* 'orientation' should have valid .roll and .pitch fields */
    Serial.print(F("Orientation: "));
    Serial.print(orientation.roll);
    Serial.print(F(" "));
    Serial.print(orientation.pitch);
    Serial.print(F(" "));
    Serial.print(orientation.heading);
    Serial.println(F(""));
  }
  
  delay(100);
}
