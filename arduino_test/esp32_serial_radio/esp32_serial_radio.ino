/*
    Software serial multiple serial test

    Receives from the hardware serial, sends to software serial.
    Receives from software serial, sends to hardware serial.

    The circuit:
    * RX is digital pin 7 (connect to TX of other device)
    * TX is digital pin 8 (connect to RX of other device)

    created back in the mists of time
    modified 25 May 2012
    by Tom Igoe
    based on Mikal Hart's example

    This example code is in the public domain.
*/

//#include <SoftwareSerial.h>

// just RX pin
Bonezegei_SoftSerial mySerial(13);


// Best for Teensy LC & 3.2
//SoftwareSerial mySerial(0, 1); // RX,TX
//SoftwareSerial mySerial(13, 12);
//SoftwareSerial mySerial(9, 10);

// Best for Teensy 3.5 & 3.6
//SoftwareSerial mySerial(0, 1); // RX,TX
//SoftwareSerial mySerial(7, 8);
//SoftwareSerial mySerial(9, 10);
//SoftwareSerial mySerial(31, 32);
//SoftwareSerial mySerial(34, 33);
//SoftwareSerial mySerial(47, 48);

// Best for Teensy 4.1
//SoftwareSerial mySerial(0, 1); // RX,TX
//SoftwareSerial mySerial(7, 8);
//SoftwareSerial mySerial(15, 14);
//SoftwareSerial mySerial(16, 17);
//SoftwareSerial mySerial(21, 20);
//SoftwareSerial mySerial(25, 24);
//SoftwareSerial mySerial(28, 29);
//SoftwareSerial mySerial(34, 35);


void setup()
{
  // Open serial communications and wait for port to open:
  //Serial.begin(57600);
  Serial.begin(115200);

  while (!Serial) {
    ; // wait for serial port to connect. Needed for Leonardo only
  }


  Serial.println("Start NMEA:");

  // set the data rate for the SoftwareSerial port
  //mySerial.begin(57600);
  mySerial.begin(115200);
  delay(1);
}

void loop() // run over and over
{

  while (mySerial.available()){
    Serial.write(mySerial.read());
  }

}

