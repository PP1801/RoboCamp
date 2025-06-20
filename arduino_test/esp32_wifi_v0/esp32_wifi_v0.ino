
#include <WiFi.h>
#include <WiFiUdp.h>

WiFiUDP udp;



//#include <BluetoothSerial.h>

//#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
//#error Bluetooth is not enabled!
//#endif

//BluetoothSerial BTlink;
//bool BT = 0;    // connection state

#include "EEPROM.h"
#define EEPROM_SIZE 43
byte autoload = 0;  // autoload saved stage
//byte set_wifi = 0;  // autoload new wifi data on startup

/* WiFi network name and password*/
char wifiName[16] = "Astro";
char wifiPass[16] = "AstroCrt4!";
int udpAddr[4] = {192, 168, 0, 103};
int udpPort[2] = {13, 5};    // 13*256 + 5 = 3333

//char ipAddr[] = "192.168.0.103";
int ipPort = 3333;

uint8_t msg_buffer[100] = "";   // for udp messages
uint8_t i_b = 0;                // index buffer
boolean connected = false;      // wi-fi connection state

uint8_t cmd = 0;    // command tracker

const byte ledPin = 18;
const byte butPin = 19;

bool pushed = 0;      // push memory
bool button = 0;      // button state
bool buttonLast = 0;  // button state -previous
unsigned long butStartTime = 0;// timer for debounce
unsigned long butDelay = 50;   // [ms]

unsigned long ledStartTime = 0;// timer for flashing
unsigned long ledPeriod = 400; // [ms]
//bool flashing = 0;

byte stage = 0;       // current color code

const byte redPin = 12;
const byte greenPin = 13;
const byte bluePin = 14;


void setup() {

  pinMode(butPin, INPUT);
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);
  
  Serial.begin(115200);
  //BTlink.begin("ESP32 bracelet");
  
  Serial.println(F("esp32_wifi_v0"));

  pinMode(redPin,   OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin,  OUTPUT);

  digitalWrite(redPin,   HIGH);
  digitalWrite(greenPin, HIGH);
  digitalWrite(bluePin,  HIGH);

  EEPROM.begin(EEPROM_SIZE);
  autoload = EEPROM.read(0);
  Serial.print(F("Autoload: "));
  Serial.println(autoload);

  connectToWiFi(wifiName, wifiPass);
  
  if (autoload > 0)
  {
    if (bitRead(autoload, 0))   // autoload LED stage
    {
      stage = EEPROM.read(1);
      set_rgb(stage);
      Serial.print(F"eestage:")); Serial.println(stage);    
    }
    
    if (bitRead(autoload, 1)) load_wifi();  // autoload Wi-Fi name and password
    
    if (bitRead(autoload, 2)) load_ip();    // autoload IP adress and port
  }

  
}

void loop() {
  
  bool buttonRead = digitalRead(butPin);
  
  if (buttonRead != buttonLast) butStartTime = millis();  // restart timer
 
  if ((millis() - butStartTime) > butDelay) // debounced button
  {
    if (buttonRead != button)
    {
      button = buttonRead;

      if (button) {pushed = 1;}
      if (pushed && !button) // pushed and released
      {
        pushed = 0;

        cmd = 0;
        do_stage();
      }
    }
  }
  buttonLast = buttonRead;
  
  if (Serial.available() > 0)   // Serial handle
  {
    String command;
    while (Serial.available() > 0)
    {
      char inChar = Serial.read();
      if (inChar == '\n') break;
      command += (char)inChar;
    }
    //if (BT) { BTlink.print(command); BTlink.print('\r'); BTlink.print('\n'); delay(10);}
    if (command != "") do_command(command);
  }

  /*if (BTlink.available() > 0)   // Bluetooth handle
  {
    BT = 1;
    
    String command;
    while (BTlink.available() > 0)
    {
      char inChar = BTlink.read();
      command += (char)inChar;
    }
    Serial.println(command);
    if (command != "") do_command(command);
  }
  */
}

void connectToWiFi(const char *ssid, const char *pwd) {
  set_rgb(2); // green
  
  Serial.println("Connecting to WiFi network: " + String(ssid));
  
  // delete old config
  WiFi.disconnect(true);
  //register event handler
  WiFi.onEvent(WiFiEvent);  // Will call WiFiEvent() from another thread.

  //Initiate connection
  WiFi.begin(ssid, pwd);

  Serial.println(F("Waiting for WIFI connection..."));

  set_rgb(0);
}

// WARNING: WiFiEvent is called from a separate FreeRTOS task (thread)!
void WiFiEvent(WiFiEvent_t event) {
  switch (event) {
    case ARDUINO_EVENT_WIFI_STA_GOT_IP:
      set_rgb(2); // green
      //When connected set
      Serial.print(F("WiFi connected! IP address: "));
      Serial.println(WiFi.localIP());
      //initializes the UDP state
      //This initializes the transfer buffer
      udp.begin(WiFi.localIP(), ipPort);  // udpPort
      set_rgb(0);
      
      connected = true;
      break;
    case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
      set_rgb(1); // red
      Serial.println(F("WiFi lost connection"));
      connected = false;
      break;
    default: break;
  }
}

void do_command(String command)
{
  bool do_rgb = 0;
  
  if      (cmd == 1) // change Wi-Fi name
  {
    byte cmd_size = command.length();

    if (cmd_size <= 16){
      for (byte i = 0; i < 16; i++)
      {
        if (i == cmd_size) {wifiName[i] = '\0'; break;}
        else wifiName[i] = command[i];
      }
    }
    //else {BTlink.print("too long, keeping ");}
    
    //BTlink.print(wifiName); BTlink.print('\r'); BTlink.print('\n'); delay(10);   
    cmd = 0;
  }
  
  else if (cmd == 2) // change Wi-Fi password
  {
    byte cmd_size = command.length();

    if (cmd_size <= 16)
    {
      for (byte i = 0; i < 16; i++)
      {
        if (i == cmd_size) {wifiPass[i] = '\0'; break;}
        else wifiPass[i] = command[i];
      }
    }
    //else {BTlink.print("too long, keeping ");}
    
    //BTlink.print(wifiPass); BTlink.print('\r'); BTlink.print('\n'); delay(10);
    cmd = 0;    
  }
  
  else if (cmd == 3)  // change IP address
  { 
    // int udpAddr[4] = {192, 168, 0, 103};
    byte cmd_size = command.length();

    byte tmp_array[4] = {0, 0, 0, 0};
    byte j = 0; // addr field
    byte k = 0; // field index
    byte digit = 0;
    
    for (byte i = 0; i < cmd_size; i++)
    {
      if (command[i] == ' ')
      {
        //Serial.print(k); Serial.print(" ");
        tmp_array[j] = k;
        j++;
        k = 0;
      }
      else k++;
    }
    tmp_array[j] = k; // contains number of digits per field
    //Serial.print(k); Serial.println("");
    j = 0;
    k = 0;

    for (byte i = 0; i < 4; i++)
    {
      if (tmp_array[i] > 3)
      {
        //BTlink.print("wrong IP"); BTlink.print('\r'); BTlink.print('\n'); delay(10);
        cmd = 0;
        return;
      }
    }
    
    udpAddr[0] = 0;
    udpAddr[1] = 0;
    udpAddr[2] = 0;
    udpAddr[3] = 0;
    
    for (byte i = 0; i < cmd_size; i++)
    {
        if (k < tmp_array[j])
        {
          byte tmp_digit = (int)command[i] - 48; // ASCII conversion
          //Serial.print(tmp_digit); Serial.print(" ");
          udpAddr[j] += tmp_digit*pow(10,tmp_array[j]-1-k);
          //Serial.println(udpAddr[j]);
          k++;       
        }
        else
        {
          //Serial.println(" ");
          j++;
          k = 0;
        }
    }

    cmd = 0;
    printip();
  }
  
  else if (cmd == 4)  // change IP port
  {
    byte cmd_size = command.length();
    
    if (cmd_size <= 5)
    {
      ipPort = 0;
      for (byte i = 0; i < cmd_size; i++)
      {
        byte digit = (int)command[i] - 48; // ASCII conversion
        //Serial.println(digit);
        ipPort += digit*pow(10,cmd_size-1-i);
        //Serial.println(ipPort);
      }
    }
    //else {BTlink.print("too long, keeping ");}
    
    //BTlink.print(ipPort); BTlink.print('\r'); BTlink.print('\n'); delay(10);   
    cmd = 0; 
  }
  
  else if (cmd == 5)  // change autoload configuration
  {
    
    autoload = EEPROM.read(0);

    int inNum = (int)command[0] - 48; // ASCII conversion
    
    if (inNum > 7) Serial.print(F("old:")); //BTlink.print("(old) ");
    else
    {
      autoload = inNum;
      EEPROM.write(0, autoload);
      EEPROM.commit();
    }
    cmd = 0;
    //BTlink.print(autoload); BTlink.print('\r'); BTlink.print('\n'); delay(10);
  }
  else if (command == "r") {do_rgb = 1; stage = 2;}
  else if (command == "g") {do_rgb = 1; stage = 1;}
  else if (command == "b") {do_rgb = 1; stage = 4;}
  else if (command == "o") {do_rgb = 1; stage = 0;}
  else if (command == "m") {do_rgb = 1; stage = 6;}
  else if (command == "y") {do_rgb = 1; stage = 3;}
  else if (command == "w") {do_rgb = 1; stage = 7;}
  else if (command == "+") {do_stage();}

  else if (command == "name") // change Wi-Fi name
  {
    cmd_change(command, 1);
    return;
  }
  else if (command == F("pass")) // change Wi-Fi password
  {
    cmd_change(command, 2);
    return;
  }
  else if (command == "ip")   // change IP address
  {
    cmd_change(command, 3);
    return;    
  }
  else if (command == "port") // change IP port
  {
    cmd_change(command, 4);
    return;    
  }
  else if (command == "wifi") // display Wi-Fi data
  {
    printall();
  }
  else if (command == "loadw")// load Wi-Fi data from EEPROM
  {
    load_wifi();

    load_ip();

    /*if(BT)
    {
      BTlink.print("loaded:"); BTlink.print('\r'); BTlink.print('\n'); delay(10);
      printall();
      }
    */
  }
  else if (command == "savew")// save Wi-Fi data to EEPROM
  {
    // save Wi-Fi name and password
    for (int i = 2; i <= 17; i++)
    {
      char inChar = wifiName[i-2];
      EEPROM.write(i, inChar);
      if (inChar == '\0') break;
    }
    wifiName[15] = '\0';
    Serial.print(F("seeWiFi:")); Serial.println(wifiName);
     
    for (int i = 18; i <= 33; i++)
    {
      char inChar = wifiPass[i-18];
      EEPROM.write(i, inChar);
      if (inChar == '\0') break; 
    }
    wifiPass[15] = '\0';
    Serial.print(F("seePass:")); Serial.println(wifiPass);
    
    // save IP adress and port
    Serial.print(F("seeIP:"));
    for (int i = 0; i < 4; i++)
    {
      int tmpNum = udpAddr[i];
      Serial.print(tmpNum);
      if (i < 3) Serial.print('.');
      EEPROM.write(i + 34, tmpNum);
    }
    Serial.println("");

    byte upperByte = ipPort >> 8;
    byte lowerByte = byte(ipPort);

    EEPROM.write(38, upperByte);
    EEPROM.write(39, lowerByte);

    Serial.print(upperByte); Serial.print(" + "); Serial.println(lowerByte);

    Serial.print(F("seePort:")); Serial.println(ipPort);

    EEPROM.commit();
    /*if (BT) 
    {
      BTlink.print("saved:"); BTlink.print('\r'); BTlink.print('\n'); delay(10);
      printall();
    }
    */
  }

  else if (command == "save")
  {
    EEPROM.write(1, stage);
    EEPROM.commit();
    Serial.print(F("Saved to memory: "));
    Serial.println(stage);
  }
  else if (command == "load")
  {
    stage = EEPROM.read(1);
    set_rgb(stage);
    Serial.print(F("Read from memory: "));
    Serial.println(stage);    
  }
  else if (command == "auto") // change autoload configuration
  {
    cmd_change(command, 5);
    return;
  }  

  if (do_rgb) set_rgb(stage);

}



void cmd_change(String command, uint8_t ini) // Print the change to BT
{
  cmd = ini;
  /*BTlink.print("new ");
  BTlink.print(command);
  BTlink.print(" = ");
  BTlink.print('\r');
  BTlink.print('\n');
  delay(10);
  */
}

void printip()              // print IP from array
{
  for (byte i = 0; i < 4; i++)
  {
    /*
    BTlink.print(udpAddr[i]);
    if (i < 3) BTlink.print('.');
    */
  }
  //BTlink.print('\r'); BTlink.print('\n'); delay(10);
}

void printall()             // print all Wi-Fi data
{
  /*
  BTlink.print("WiFi:"); BTlink.print(wifiName); BTlink.print('\r'); BTlink.print('\n'); delay(10);
  BTlink.print("Pass:"); BTlink.print(wifiPass); BTlink.print('\r'); BTlink.print('\n'); delay(10);
  BTlink.print("IP:"); printip();
  BTlink.print("Port:"); BTlink.print(ipPort); BTlink.print('\r'); BTlink.print('\n'); delay(10);
  */
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

void do_stage()     // advance to next stage
{
  stage++;
  if (stage > 7) stage = 0;

  set_rgb(stage);
  /*     
  BTlink.print("b: ");
  BTlink.print(bitRead(stage, 2));
  BTlink.print(bitRead(stage, 1));
  BTlink.print(bitRead(stage, 0));
  BTlink.print('\n');
  */
}

void load_wifi()    // load Wi-Fi name and password from EEPROM
{
  for (int i = 2; i <= 17; i++)
  {
    char inChar = EEPROM.read(i);
    wifiName[i-2] = inChar;
    //Serial.print(inChar);
    if (inChar == '\0') break;
  }
  wifiName[15] = '\0';
  //Serial.println("");
  Serial.print(F("eeWiFi:")); Serial.println(wifiName);
  //BTlink.print("eeWiFi:"); BTlink.print(wifiName); BTlink.print('\r'); BTlink.print('\n'); delay(10);
  for (int i = 18; i <= 33; i++)
  {
    char inChar = EEPROM.read(i);
    wifiPass[i-18] = inChar;
    //Serial.print(inChar);
    if (inChar == '\0') break;
  }
  wifiPass[15] = '\0';
  //Serial.println("");
  Serial.print(F("eePass:")); Serial.println(wifiPass);
  //BTlink.print("eePass:"); BTlink.print(wifiPass); BTlink.print('\r'); BTlink.print('\n'); delay(10);  
}

void load_ip()      // load IP address and port from EEPROM
{
  for (int i = 0; i < 4; i++)
  {
    int tmp_num = EEPROM.read(i + 34);
    udpAddr[i] = tmp_num;
  }
  Serial.print("eeIP:"); //Serial.println();

  for (byte i = 0; i < 4; i++)
  {
    Serial.print(udpAddr[i]);
    if (i < 3) Serial.print(F('.'));
  }
  Serial.println("");

  int upperByte = EEPROM.read(38);
  int lowerByte = EEPROM.read(39);
  udpPort[0] = upperByte;
  udpPort[1] = lowerByte;
  ipPort = upperByte*256 + lowerByte;

  Serial.print(F("eePort:")); Serial.println(ipPort);
    
  //int udpAddr[4] = {192, 168, 0, 103};
  //int udpPort[2] = {13, 5};    // 13*256 + 5 = 3333  
}
