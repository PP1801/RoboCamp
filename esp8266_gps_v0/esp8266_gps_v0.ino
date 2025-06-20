/*


 */


#include <SPI.h>
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>

#include "arduino_secrets.h" 
///////please enter your sensitive data in the Secret tab/arduino_secrets.h
char ssid[] = SECRET_SSID;   // your network SSID (name)
char pwd[] = SECRET_PASS;    // your network password (use for WPA, or use as key for WEP)

//const char * udpAddress = "192.168.43.221";// robocamp1 -mobile
const char * udpAddress = "192.168.0.145"; // robocamp1 -Putanec
//const char * udpAddress = "192.168.208.130"; // robocamp1 -Putanec WM
//const char * udpAddress = "192.168.46.173";// Studenti
const int udpPort = 3333;      // local port to listen on

WiFiUDP udp;

#include <Adafruit_GPS.h>
Adafruit_GPS GPS(&Wire);

#define HAS_INA 0

#if HAS_INA == 1
  #include <Adafruit_INA260.h>
  //#define INA_READ_FREQ 1 // [Hz]
  Adafruit_INA260 ina260 = Adafruit_INA260();
  unsigned long inaStartTime = 0;// timer for Current measurement
  unsigned long inaPeriod = 2000; // [ms]
  bool ina_init = 0;
  bool ina_on = 0;
#endif

char msg_buffer[100]; //buffer to hold incoming packet
int keyIndex = 0;            // your network key index number (needed only for WEP)
boolean connected = false;    //Wi-Fi connection status

const byte butPin = 13;
unsigned long butStartTime = 0;// timer for debounce
unsigned long butDelay = 50;   // [ms]

unsigned long ledStartTime = 0;// timer for flashing
unsigned long ledPeriod = 400; // [ms]
bool flashing = 0;

unsigned long longDelay = 2000;// [ms] 
bool long_press = 0;// long press for facial tracking

byte stage = 0;     // current color code
byte prog = 0;      // current program tracker
uint8_t cmd = 0;    // command tracker

const byte redPin = 12;
const byte greenPin = 15;
const byte bluePin = 14;


void setup() {
  //Initialize serial and wait for port to open:
  Serial.begin(115200);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  
  Serial.println(F("esp8266_gps_v0"));
  
  pinMode(butPin, INPUT);

  pinMode(redPin,   OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin,  OUTPUT);

  set_rgb(0); // off

  connectToWiFi(ssid, pwd); //Connect to the WiFi network

#if HAS_INA == 1
  Serial.println(F("with INA260"));
  if (ina260.begin()){
    set_rgb(6);
    ina260.setMode(INA260_MODE_CONTINUOUS);
    ina260.setCurrentConversionTime(INA260_TIME_140_us);
    ina260.setVoltageConversionTime(INA260_TIME_140_us);
    ina_init = 1;
    delay(100);
    Serial.println(F("INA OK"));
  }
  else
    Serial.println(F("INA FAILED"));
  set_rgb(0);
#endif


  GPS.begin(0x10);  // I2C address of the GPS module
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_GGAONLY);
  delay(100);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  delay(100);
  //GPS.sendCommand(PMTK_ENABLE_SBAS);
  //delay(100);
  //GPS.sendCommand(PMTK_ENABLE_WAAS);
  //delay(100);
  
  //GPS.sendCommand(PGCMD_ANTENNA);  // Request updates on antenna status, comment out to keep quiet
  //delay(100);


}

void loop() {

  static bool pushed = 0;      // push memory
  static bool button = 0;      // button state
  static bool buttonLast = 0;  // button state -previous
  
  bool buttonRead = digitalRead(butPin);  // button handle
  
  if (buttonRead != buttonLast) butStartTime = millis();  // restart timer

  //Serial.println(buttonRead);
 
  if ((millis() - butStartTime) > butDelay) // debounced button
  {
    if (buttonRead != button)// button state change detection
    {
      button = buttonRead;

      if (button)            // pushed
      {
        pushed = 1;
      }
      if (pushed && !button) // pushed and released
      {
        
        pushed = 0;

        if (long_press)
        {
          prog = 3;
          long_press = 0;
        }
        else
        {
          prog++;
          if (prog > 2) prog = 0;
        }
        
        if (prog == 0)      // off
        {
          Serial.println(F("off"));
          stage = 0;
          flashing = 0;
          set_rgb(0);

          if (connected)
          {
            udp.beginPacket(udpAddress, udpPort);
            udp.print("off");
            //udp.print('\n');
            udp.endPacket();
          }
        }
        
        else if (prog == 1) // prog = 1 -> forward GPS location and find
        {
          Serial.println(F("find gps"));
          stage = 1;  // green
          flashing = 0;
          set_rgb(stage);
          
          if (connected)
          {
            udp.beginPacket(udpAddress, udpPort);
            udp.print("gps");
            //udp.print('\n');
            udp.endPacket();
          }
        }
        
        // send help message       
        else if (connected && (prog == 2))  // If we have connection, request search
        {
          Serial.println(F("search face"));
          stage = 4;  // blue
          set_rgb(stage);
          flashing = 1;
       
          udp.beginPacket(udpAddress, udpPort); // send by wi-fi udp
          udp.print("search");
          //udp.print('\n');
          udp.endPacket();
        }

        else if (connected && (prog == 3)) // If we have connection, request tracking
        {
          Serial.println(F("track face"));
          stage = 3;  // yellow
          set_rgb(stage);
          flashing = 1;
          
          udp.beginPacket(udpAddress, udpPort); // send by wi-fi udp
          udp.print("track");
          //udp.print('\n');
          udp.endPacket();
        }
      
      } // if (pushed && !button)

    } // if (buttonRead != button)
    else if (pushed && !long_press)// long press detection
    {
      if ((millis() - butStartTime) > longDelay)
      {
        long_press = 1;
        //Serial.println(F("long"));
      }
    }
  }
  
  buttonLast = buttonRead;  


//  // if there's data available, read a packet
//  int packetSize = udp.parsePacket();
//  if (packetSize) {
//    Serial.print("Received packet of size ");
//    Serial.println(packetSize);
//    Serial.print("From ");
//    IPAddress remoteIp = udp.remoteIP();
//    Serial.print(remoteIp);
//    Serial.print(", port ");
//    Serial.println(udp.remotePort());
//
//    // read the packet into msg_buffer
//    int len = udp.read(msg_buffer, 255);
//    if (len > 0) {
//      msg_buffer[len] = 0;
//    }
//    Serial.println("Contents:");
//    Serial.println(msg_buffer);
//
//    // send a reply, to the IP address and port that sent us the packet we received
//    udp.beginPacket(udp.remoteIP(), udp.remotePort());
//    udp.print(ReplyBuffer); // write()
//    udp.print('\n');
//    udp.endPacket();
//  } // if (packetSize)

  if (udp.parsePacket() > 0)      // udp recieve handle
  {
    String message;
    while(udp.available() > 0)
    {
      char inChar = udp.read();
      if (inChar == '\n') break;
      message += (char)inChar;
    }
    if (message == "lost")
    {
      flashing = 1;
      Serial.println(F("lost vis!"));
      set_rgb(stage);
    }
    else if (message == "search1")
    {
      flashing = 0;
      Serial.println(F("search on"));
      set_rgb(stage);
    }
    else if (message == "track1")
    {
      flashing = 0;
      Serial.println(F("track on"));
      set_rgb(stage);
    }
    else if (message == "gps1")
    {
      flashing = 1;
      Serial.println(F("gps on"));
      set_rgb(stage);
    }
    else if (message == "off2")
    {
      flashing = 0;
      Serial.println(F("force off"));
      stage = 0;
      prog = 0;
      set_rgb(0);

      IPAddress ip = WiFi.localIP();
      udp.beginPacket(udpAddress, udpPort);
      udp.print("#off3,");
      udp.print(ip);
      //udp.print('\n');
      udp.endPacket();
    }
    
#if HAS_INA == 1
    else if (message == "ina")
    {
      ina_on = !ina_on;  
      Serial.print(F("ina echo: "));
      Serial.println(ina_on);
      
      udp.beginPacket(udpAddress, udpPort); // send by wi-fi udp
      udp.print("ina,");
      udp.print(ina_on);
      udp.endPacket();
    }
#endif
  }


  char gps_dbg = GPS.read();
  //if (gps_dbg) Serial.print(gps_dbg);
  
//  if (GPS.newNMEAreceived())
//  {
//
//    if (connected && (prog >= 1)) // If we have connection, send by wi-fi udp
//    {
//      Serial.print(GPS.lastNMEA());
//      
//      udp.beginPacket(udpAddress, udpPort);
//      udp.print(GPS.lastNMEA());
//      udp.print('\n');
//      udp.endPacket();
//    }
//  }

  if (GPS.newNMEAreceived()) {
    String nmeaData = GPS.lastNMEA();
    
    if (connected && (prog >= 1)) {

      Serial.print(nmeaData);
      // Split fused NMEA messages
      int startIdx = 0;
      while (true) {
        int nextDollarIdx = nmeaData.indexOf('$', startIdx + 1); // Find next '$'
        //Serial.println(nextDollarIdx);
        
        if (nextDollarIdx == -1) {
          // Send the remaining part (last NMEA message)
          String singleNMEA = nmeaData.substring(startIdx); // Extract last part
          singleNMEA.trim(); // Trim whitespace in place
          //Serial.println(singleNMEA);
          if (!singleNMEA.isEmpty()) {
            sendUDP(singleNMEA);
          }
          break;
        }
        // Extract single NMEA message
        String singleNMEA = nmeaData.substring(startIdx, nextDollarIdx); // Extract part
        singleNMEA.trim(); // Trim whitespace in place
        //Serial.println(singleNMEA);
        if (!singleNMEA.isEmpty()) {
          sendUDP(singleNMEA);
        }
        // Move to the next part
        startIdx = nextDollarIdx;
      }
    }
  }


  if (flashing)                   // LED control timer
  {
    static bool ledState = 0;
    unsigned long currentTime = millis();
    if (currentTime - ledStartTime > ledPeriod)
    {
      ledState = !ledState;
    
      if (ledState)
      {
        set_rgb(stage);
      }
      
      else 
      {
        if (!connected) set_rgb(2);  // red
        else set_rgb(0);
      }
      ledStartTime = millis();
    }    
  } // if (flashing)

#if HAS_INA == 1
  if (ina_init)
  {
    unsigned long currentTime = millis();
    if (currentTime - inaStartTime > inaPeriod)
    {
      float ina_c = ina260.readCurrent();
      float ina_v = ina260.readBusVoltage();
      float ina_p = ina260.readPower();
      
      if (connected && ina_on)
      {
        Serial.print(F("C: ")); Serial.println(ina_c);
        Serial.print(F("V: ")); Serial.println(ina_v);
        Serial.print(F("P: ")); Serial.println(ina_p);

        udp.beginPacket(udpAddress, udpPort); // send by wi-fi udp
        udp.print("ina,"); 
        udp.print(ina_c); udp.print(',');
        udp.print(ina_v); udp.print(','); 
        udp.print(ina_p); udp.print('\n');
        udp.endPacket();
      }
      inaStartTime = millis();
    }
  }
#endif

} // loop


void sendUDP(const String& message) {
  udp.beginPacket(udpAddress, udpPort);
  udp.print(message);
  udp.print('\n'); // Add newline for clarity
  udp.endPacket();
}

void connectToWiFi(const char *ssdi, const char *pwd) {
  
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, pwd);
  
  // attempt to connect to WiFi network:
  Serial.print("Attempting to connect to SSID: ");
  Serial.println(ssid);
  set_rgb(2); // red
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print('.');
    delay(500);
  }

  set_rgb(1); // green
  Serial.print("Connected to WiFi -> ");
  Serial.println(WiFi.localIP());
  connected = true;

  udp.begin(udpPort);

  delay(40);
  
  udp.beginPacket(udpAddress, udpPort); // send by wi-fi udp
  udp.print("new");
  udp.endPacket();

  set_rgb(0); // off
  
  //udp.begin(WiFi.localIP(), udpPort);}
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
