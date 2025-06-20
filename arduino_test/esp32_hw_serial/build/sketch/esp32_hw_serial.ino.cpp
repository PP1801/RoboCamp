#include <Arduino.h>
#line 1 "C:\\Users\\CRTA\\Documents\\Putanec_documents\\esp32_stuff\\esp32_hw_serial\\esp32_hw_serial.ino"
//
#define RXD2 16
#define TXD2 17

#line 5 "C:\\Users\\CRTA\\Documents\\Putanec_documents\\esp32_stuff\\esp32_hw_serial\\esp32_hw_serial.ino"
void setup();
#line 17 "C:\\Users\\CRTA\\Documents\\Putanec_documents\\esp32_stuff\\esp32_hw_serial\\esp32_hw_serial.ino"
void loop();
#line 5 "C:\\Users\\CRTA\\Documents\\Putanec_documents\\esp32_stuff\\esp32_hw_serial\\esp32_hw_serial.ino"
void setup()
{
	//Serial2.begin(baud-rate, protocol, RX pin, TX pin)
    Serial.begin(115200);
    Serial2.begin(57600, SERIAL_8N1, RXD2, TXD2);

    Serial.println("Serial2 TX pin: " +String(TXD2));
    Serial.println("Serial2 RX pin: " +String(RXD2));

    Serial.println("NMEA start");
}

void loop()
{
	while(Serial2.available() > 0)
    {
        Serial.print(char(Serial2.read()));
    }
}

