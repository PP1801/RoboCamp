
const byte ledPin = 18;
const byte butPin = 19;

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

  Serial.println("esp32_led_2.ino");

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
      //Serial.print("button = ");
      //Serial.println(button);
      
      if (button) 
      {
        stage++;
        if (stage > 7) stage = 0;

        if (bitRead(stage, 1)) digitalWrite(redPin, LOW);
        else digitalWrite(redPin, HIGH);
        if (bitRead(stage, 0)) digitalWrite(greenPin, LOW);
        else digitalWrite(greenPin, HIGH);
        if (bitRead(stage, 2)) digitalWrite(bluePin, LOW); 
        else digitalWrite(bluePin, HIGH);

        Serial.print("b: ");
        Serial.print(bitRead(stage, 2));
        Serial.print(bitRead(stage, 1));
        Serial.println(bitRead(stage, 0));
        
        //flashing = !flashing; //toggle led flashing
        //if (!flashing) digitalWrite(ledPin, LOW);
        //Serial.print("led = ");
        //Serial.println(flashing);
      }
    }
  }
  buttonLast = buttonRead;
  
  //if (flashing)
  //{
  //  unsigned long currentTime = millis();
  //  if (currentTime - ledStartTime > ledPeriod)
  //  //if (millis - ledStartTime > ledPeriod)
  //  {
  //    digitalWrite(ledPin, !digitalRead(ledPin));
  //    ledStartTime = millis();
  //  }    
  //}

}
