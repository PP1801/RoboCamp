

#include <WiFi.h>
#include <WiFiUdp.h>

// Hardware Serial (Serial2)
#define RXD2 16 // GREEN
#define TXD2 17 // ORANGE

/* WiFi network name and password */
//const char * ssid = "Astro";
//const char * pwd  = "AstroCrt4!";
//const char * ssid = "HUAWEI P9 lite 2017";
//const char * pwd  = "5a595b115cc4";
//const char * ssid = "Studenti";
//const char * pwd  = "Stud3ntCrt4!";
const char * ssid = "robocamp1";
const char * pwd  = "robocamp1";

//const char * udpAddress = "192.168.43.221";// HUAWEI P9
//const char * udpAddress = "192.168.46.173";  // Studenti
//const char * udpAddress = "192.168.40.156";  // Wired
//const char * udpAddress = "192.168.0.108"; // Astro ??
const char * udpAddress = "192.168.0.229"; // robocamp1 

const int udpPort = 3333;

WiFiUDP udp;  //create UDP instance

uint8_t msg_buffer[100] = ""; // for udp messages
uint8_t i_b = 0;              // buffer index
boolean connected = false;    //Wi-Fi connection status


const byte butPin = 19;
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
const byte greenPin = 13;
const byte bluePin = 14;


void setup()
{
  Serial.begin(115200);
  //Serial2.begin(baud-rate, protocol, RX pin, TX pin)
  Serial2.begin(57600, SERIAL_8N1, RXD2, TXD2);
  //Serial2.begin(11500, SERIAL_8N1, RXD2, TXD2);
  
  pinMode(butPin, INPUT);

  Serial.println("udp_short_v1");
  

  connectToWiFi(ssid, pwd); //Connect to the WiFi network
    
  pinMode(redPin,   OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin,  OUTPUT);

  digitalWrite(redPin,   LOW);
  digitalWrite(greenPin, HIGH);
  digitalWrite(bluePin,  HIGH);

}

void loop()
{
  static bool pushed = 0;      // push memory
  static bool button = 0;      // button state
  static bool buttonLast = 0;  // button state -previous
  
  bool buttonRead = digitalRead(butPin);  // button handle
  
  if (buttonRead != buttonLast) butStartTime = millis();  // restart timer
 
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
            udp.print('\n');
            udp.endPacket();
          }
        }
        
        else if (prog == 1) // prog = 1 -> forward GPS location
        {
          Serial.println(F("tracking"));
          stage = 4;  // blue
          flashing = 1;

          if (connected)
          {
            udp.beginPacket(udpAddress, udpPort);
            udp.print("gps");
            udp.print('\n');
            udp.endPacket();
          }
        }
        
        // send help message       
        else if (connected && (prog == 2))  // If we have connection, request help!
        {
          Serial.println(F("help!"));
          stage = 1;  // green
          set_rgb(stage);
          flashing = 0;
       
          udp.beginPacket(udpAddress, udpPort); // send by wi-fi udp
          udp.print("help");
          udp.print('\n');
          udp.endPacket();
        }

        else if (connected && (prog == 3)) // If we have connection, request tracking
        {
          Serial.println(F("face"));
          stage = 3;  // yellow
          set_rgb(stage);
          flashing = 0;
          
          udp.beginPacket(udpAddress, udpPort); // send by wi-fi udp
          udp.print("face");
          udp.print('\n');
          udp.endPacket();
        }
      
      }

    }
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
  
  while(Serial2.available() > 0)  // GPS connection handle
  {
    digitalWrite(redPin, HIGH);
    //Serial.print(char(Serial2.read()));

    //msg_buffer[i_b] = char(Serial2.read());
    msg_buffer[i_b] = Serial2.read();
    
    if (msg_buffer[i_b] == '$')
    {
      /* $ G P G G A   $ G N G G A 
      /  0 1 2 3 4 5   0 1 2 3 4 5
      /  we are only interested in $GPGGA or $GNGGA messages
      /  why? -> because only these messages are processed
      /  in MissionPlanner/Controls/FollowMe.cs
     */
      if ((msg_buffer[2] == 'P' || msg_buffer[2] == 'N') && 
          (msg_buffer[4] == 'G' && msg_buffer[5] == 'A'))
      {
        if (connected && (prog >= 1)) // If we have connection, send by wi-fi udp
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
      }
        /* either it is the right message and is sent, or
        /  it is the wrong message we can just ignore.
        /  in either case we reset the buffer
       */
      msg_buffer[0] = '$';  // force the first character to have a clean start
      i_b = 0; 
    }       // if($)
    i_b++;  
  }         // while(Serial2)

  
  if (udp.parsePacket() > 0)      // udp recieve handle
  {
    String message;
    while(udp.available() > 0)
    {
      char inChar = udp.read();
      if (inChar == '\n') break;
      message += (char)inChar;
    }
    if (message == "foll")
    {
      flashing = 1;
      Serial.println(F("got foll!"));
    }
    else if (message == "face")
    {
      flashing = 1;
      Serial.println(F("got face!"));
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
  }

}   // loop

void connectToWiFi(const char *ssid, const char *pwd) {
  set_rgb(1); // green
  
  Serial.print(F("Connecting to WiFi network: "));
  Serial.println(String(ssid));

  WiFi.disconnect(true);    // delete old config
                            //register event handler
  WiFi.onEvent(WiFiEvent);  // Will call WiFiEvent() from another thread.
  
  WiFi.begin(ssid, pwd);    //Initiate connection

  Serial.println(F("Waiting for WIFI connection..."));
  delay(40);

  set_rgb(0);
}

// WARNING: WiFiEvent is called from a separate FreeRTOS task (thread)!
void WiFiEvent(WiFiEvent_t event) {
  switch (event) {
    case ARDUINO_EVENT_WIFI_STA_GOT_IP:
      set_rgb(1); // green
      //When connected set
      Serial.print(F("WiFi connected! IP address: "));
      Serial.println(WiFi.localIP());
      //initializes the UDP state
      //This initializes the transfer buffer
      udp.begin(WiFi.localIP(), udpPort);
      delay(40);
      set_rgb(0);
      
      connected = true;
      break;
    case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
      set_rgb(2); // red
      Serial.println(F("WiFi lost connection"));
      connected = false;
      break;
    default: break;
  }
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
