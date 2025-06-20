

#include <WiFi.h>
#include <WiFiUdp.h>

/* WiFi network name and password */
//const char * ssid = "Astro";
//const char * pwd = "AstroCrt4!";

const char * ssid = "HUAWEI P9 lite 2017";
const char * pwd = "5a595b115cc4";
// IP address to send UDP data to.
// it can be ip address of the server or 
// a network broadcast address
// here is broadcast address

const char * udpAddress = "192.168.43.221";

//const char * udpAddress = "192.168.1.100";
const int udpPort = 3333;
//create UDP instance
WiFiUDP udp;

uint8_t msg_buffer[100] = "";
uint8_t i_b = 0;

//Are we currently connected?
boolean connected = false;

// Hardware Serial (Serial2)
#define RXD2 16 // GREEN
#define TXD2 17 // ORANGE

void setup()
{
    Serial.begin(115200);
  //Serial2.begin(baud-rate, protocol, RX pin, TX pin)
    Serial2.begin(57600, SERIAL_8N1, RXD2, TXD2);
  //Serial2.begin(11500, SERIAL_8N1, RXD2, TXD2);


    Serial.println("Serial2 TX pin: "+String(TXD2));
    Serial.println("Serial2 RX pin: "+String(RXD2));

  //Connect to the WiFi network
    connectToWiFi(ssid, pwd);

    Serial.println("NMEA UDP short start");
}

void loop()
{
  while(Serial2.available() > 0)
  {
    //Serial.print(char(Serial2.peek()));

    //msg_buffer[i_b] = char(Serial2.read());
    msg_buffer[i_b] = Serial2.read();
    
    if (msg_buffer[i_b] == '$')
    {
      // $ G P G G A   $ G N G G A 
      // 0 1 2 3 4 5   0 1 2 3 4 5
      // we are only interested in $GPGGA or $GNGGA messages
      // why? -> because only these messages are processed
      // in MissionPlanner/Controls/FollowMe.cs

      if ((msg_buffer[2] == 'P' || msg_buffer[2] == 'N') && 
          (msg_buffer[4] == 'G' && msg_buffer[5] == 'A'))
      {
        if (connected)    // If we have connection, send by wi-fi udp
        {
          udp.beginPacket(udpAddress, udpPort);
          for (uint8_t ii = 0 ; ii < i_b - 1 ; ii++)
            udp.print(char(msg_buffer[ii]));
          udp.print('\n');
          udp.endPacket();
        }
        else              // or if we don't, print to serial
          for (uint8_t ii = 0 ; ii < i_b ; ii++)
            Serial.print(char(msg_buffer[ii]));
        // either it is the right message and is sent, or
      } // it is the wrong message we can just ignore
        // in either case we reset the buffer
      msg_buffer[0] = '$';  // force the first character to have a clean start
      i_b = 0; 

    } // if($)

    i_b++;
  } // while(Serial2)
}   // loop

void connectToWiFi(const char *ssid, const char *pwd) {
  Serial.println("Connecting to WiFi network: " + String(ssid));

  // delete old config
  WiFi.disconnect(true);
  //register event handler
  WiFi.onEvent(WiFiEvent);  // Will call WiFiEvent() from another thread.

  //Initiate connection
  WiFi.begin(ssid, pwd);

  Serial.println("Waiting for WIFI connection...");
}

// WARNING: WiFiEvent is called from a separate FreeRTOS task (thread)!
void WiFiEvent(WiFiEvent_t event) {
  switch (event) {
    case ARDUINO_EVENT_WIFI_STA_GOT_IP:
      //When connected set
      Serial.print("WiFi connected! IP address: ");
      Serial.println(WiFi.localIP());
      //initializes the UDP state
      //This initializes the transfer buffer
      udp.begin(WiFi.localIP(), udpPort);
      connected = true;
      break;
    case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
      Serial.println("WiFi lost connection");
      connected = false;
      break;
    default: break;
  }
}
