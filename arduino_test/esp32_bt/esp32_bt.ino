#include <BluetoothSerial.h>

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled!
#endif

BluetoothSerial BTlink;

const byte ledPin = 18;
const byte butPin = 19;

bool pushed = 0;
bool button = 0;
bool buttonLast = 0;
unsigned long butStartTime = 0;
unsigned long butDelay = 50;  // [ms]

unsigned long ledStartTime = 0;
unsigned long ledPeriod = 400; // [ms]
bool flashing = 0;

byte stage = 0;

const byte redPin = 12;
const byte greenPin = 13;
const byte bluePin = 14;


void setup() {

  pinMode(butPin, INPUT);
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);
  
  Serial.begin(115200);
  BTlink.begin("ESP32 bracelet");
  
  Serial.println("esp32_bt");

  pinMode(redPin,   OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin,  OUTPUT);

  digitalWrite(redPin,   HIGH);
  digitalWrite(greenPin, HIGH);
  digitalWrite(bluePin,  HIGH);
}

void loop() {
  
  bool buttonRead = digitalRead(butPin);
  if (buttonRead != buttonLast)
  {
    butStartTime = millis();
  }

  if ((millis() - butStartTime) > butDelay)
  {
    if (buttonRead != button)
    {
      button = buttonRead;

      if (button) {pushed = 1;}
      if (pushed && !button) // pushed and released
      {
        pushed = 0;

        do_stage();
      }
    }
  }
  buttonLast = buttonRead;
  
  if (Serial.available() > 0)
  {
    while (Serial.available() > 0)
    {
      BTlink.write(Serial.read());
    }
    BTlink.write('\n');
  }

  if (BTlink.available() > 0)
  {
    String command;
    while (BTlink.available() > 0)
    {
      char inChar = BTlink.read();
      command += (char)inChar;
    }
    Serial.println(command);
    do_command(command);
  }

}

void do_command(String command)
{
  if      (command == "red"  )  {set_rgb(2);}
  else if (command == "green")  {set_rgb(1);}
  else if (command == "blue" )  {set_rgb(4);}
  else if (command == "off"  )  {set_rgb(0);}
  else if (command == "magenta"){set_rgb(6);}
  else if (command == "yellow" ){set_rgb(3);}
  else if (command == "white"  ){set_rgb(7);}
  else if (command == "+") {do_stage();}
}

void set_rgb(byte inStage)
{
  if (bitRead(inStage, 1)) digitalWrite(redPin, LOW);
  else digitalWrite(redPin, HIGH);
  if (bitRead(inStage, 0)) digitalWrite(greenPin, LOW);
  else digitalWrite(greenPin, HIGH);
  if (bitRead(inStage, 2)) digitalWrite(bluePin, LOW); 
  else digitalWrite(bluePin, HIGH);
}

void do_stage()
{
  stage++;
  if (stage > 7) stage = 0;

  set_rgb(stage);
        
  BTlink.print("b: ");
  BTlink.print(bitRead(stage, 2));
  BTlink.print(bitRead(stage, 1));
  BTlink.print(bitRead(stage, 0));
  BTlink.print('\n');
}
