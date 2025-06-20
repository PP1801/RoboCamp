# 1 "C:\\Users\\CRTA\\Documents\\Putanec_documents\\esp32_stuff\\esp32_hw_serial\\esp32_hw_serial.ino"
//



void setup()
{
 //Serial2.begin(baud-rate, protocol, RX pin, TX pin)
    Serial0.begin(115200);
    Serial2.begin(57600, SERIAL_8N1, 16, 17);

    Serial0.println("Serial2 TX pin: " +String(17));
    Serial0.println("Serial2 RX pin: " +String(16));

    Serial0.println("NMEA start");
}

void loop()
{
 while(Serial2.available() > 0)
    {
        Serial0.print(char(Serial2.read()));
    }
}
