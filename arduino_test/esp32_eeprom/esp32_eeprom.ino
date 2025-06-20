

#include "EEPROM.h"
#define EEPROM_SIZE 2
bool autoload = 0;

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
  Serial.println("esp32_eeprom");

  pinMode(redPin,   OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin,  OUTPUT);

  digitalWrite(redPin,   HIGH);
  digitalWrite(greenPin, HIGH);
  digitalWrite(bluePin,  HIGH);

  EEPROM.begin(EEPROM_SIZE);
  autoload = EEPROM.read(0);
  Serial.print("Autoload: ");
  Serial.println(autoload);

  if (autoload)
  {
    stage = EEPROM.read(1);
    set_rgb(stage);
    Serial.print("Setting stage to: ");
    Serial.println(stage);
  }
  
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
    String command;
    while (Serial.available() > 0)
    {
      char inChar = Serial.read();
      if (inChar == '\n') break;
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
  else if (command == "magenta"){set_rgb(6);}
  else if (command == "yellow") {set_rgb(3);}
  else if (command == "white" ) {set_rgb(7);}
  else if (command == "off"   ) {set_rgb(0);}
  else if (command == "+"     ) {do_stage();}
  
  else if (command == "save")
  {
    EEPROM.write(1, stage);
    EEPROM.commit();
    Serial.print("Saved to memory: ");
    Serial.println(stage);
  }
  else if (command == "load")
  {
    stage = EEPROM.read(1);
    set_rgb(stage);
    Serial.print("Read from memory: ");
    Serial.println(stage);    
  }
  else if (command == "auto")
  {
    autoload = EEPROM.read(0);
    Serial.print("Autoload changed from: "); Serial.print(autoload);
    
    if (autoload >= 1) autoload = 0;
    else autoload = 1;

    EEPROM.write(0, autoload);
    EEPROM.commit();
    Serial.print(" to "); Serial.println(autoload);
  }
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
        
  Serial.print("b: ");
  Serial.print(bitRead(stage, 2));
  Serial.print(bitRead(stage, 1));
  Serial.print(bitRead(stage, 0));
  Serial.print('\n');
}
