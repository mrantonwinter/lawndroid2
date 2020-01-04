#include <Psx.h>
#include <Servo.h>


////////////////////////////////////////////////////////////////
// arduino libary documentation
//
// PSX Library
// https://playground.arduino.cc/Main/PSXLibrary/
//
//
// Rhino RKI-1580
// https://robokits.co.in/control-boards/arduino-robot-control-board/arduino-uno-r3-based-20a-robot-control-board
// http://robokits.co.in/datasheets/RKI-1580(User_Manual).pdf
//



////////////////////////////////////////////////////////////////
// general setup
#define ledPin 3


////////////////////////////////////////////////////////////////
// playstation 2 controller setup and config

#define dataPin 4
#define cmdPin 5
#define attPin 6
#define clockPin 7

Psx playstation;
unsigned int playstationData = 0;


////////////////////////////////////////////////////////////////
// cutter motor setup and config
//
// the cutter motor is controlled via an RC switch, which is controlled by the servo output
//

#define CutterOn 120
#define CutterOff 90
#define CutterServoChannel A2

Servo cutterServo;
bool cutterOn = false;

////////////////////////////////////////////////////////////////
// height servo setup and config

#define FrontServoChannel A0
#define RearServoChannel A1

Servo frontServo;
Servo rearServo;


////////////////////////////////////////////////////////////////
// drive motors setup and config

#define rightMotorPin 9
#define leftMotorPin  10

#define rDirPin  8
#define lDirPin 12



// direction change delay to protect the circuits from frying when suddenly changing
#define directionChangeDelay 50
bool fast = false;

////////////////////////////////////////////////////////////////
//
// setup
//
void setup()
{
  //general
  Serial.begin(19200);
  pinMode(ledPin, OUTPUT);

  //playstation controller
  playstation.setupPins(dataPin, cmdPin, attPin, clockPin, 10);


  //servo motors

  //frontServo.attach(FrontServoChannel);
  //rearServo.attach(RearServoChannel);

  //make sure cutter is off
  cutterServo.attach(CutterServoChannel);
  Cutter(false);

  //drive motors
  pinMode(rightMotorPin, OUTPUT);
  pinMode(leftMotorPin, OUTPUT);

  pinMode(rDirPin, OUTPUT);
  pinMode(lDirPin, OUTPUT);


}

////////////////////////////////////////////////////////////////
//
// main loop
//
unsigned long previousMillis = 0;
const long interval = 50;

void loop()
{
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    playstationData = playstation.read();

    //Serial.print("psx: ");
    //Serial.println(playstationData);


    CutterLoop(playstationData);
    HeightLoop(playstationData);
    MovementLoop(playstationData);

  }
}




////////////////////////////////////////////////////////////////

//
// only run the cutter when the button is pressed
//
void  CutterLoop(int data) {
  //turns on the cutter and led if the playstation left button 2 is pressed
  if (data & psxL2){
    if (!cutterOn){
      cutterOn = true; 
      Cutter(true);
      Led(true); 
    }
  }
  else if (cutterOn) {
      Cutter(false);
      Led(false);     
      cutterOn = false; 
  }

}

////////////////////////////////////////////////////////////////
//
// change the height of the legs loop
//
void  HeightLoop(int data) {
  // TODO
  
  //Serial.print("servo: ");
  //Serial.println(frontServo.read());
}


////////////////////////////////////////////////////////////////
//
// handle movement and direction change loop
//
void  MovementLoop(int data) {
  if (data & psxLeft) {
    Left();
  }
  else if (data & psxRight) {
    Right();
  }
  else if (data & psxUp) {
    Forward();
  }
  else if (data & psxDown) {
    Backward();
  }

  if (data & psxTri) {
    Stop();
  }

  // right bottom button to move fast
  if (data & psxR2 ) {
    if (!fast) {
      fast = true;
      SetPWM();
    }
  }
  else if ( fast) {
    fast = false;
    SetPWM();
  }

}




////////////////////////////////////////////////////////////////


void Led(bool state)
{
  if (state) {
    digitalWrite(ledPin, HIGH);
  }
  else {
    digitalWrite(ledPin, LOW );
  }
}


////////////////////////////////////////////////////////////////

void Cutter( bool state)
{
  if (state) {
    cutterServo.write(CutterOn);
  }
  else {
    cutterServo.write(CutterOff);
  }
}


////////////////////////////////////////////////////////////////
//
// drive motion
//
void SetPWM()
{
  // PWM documentation
  // https://www.eprojectszone.com/how-to-modify-the-pwm-frequency-on-the-arduino-part1/
  // https://www.arduino.cc/en/Tutorial/SecretsOfArduinoPWM

  

  if (fast) {
    //medium pace, no whine
    TCCR1A = B10100001; // phase correct, compare match A and compare match B
    TCCR1B = B00001010; // phase correct, 1/8 prescale
    OCR1A = 12; //changing this doesnt seem to change the speed
    OCR1B = 12; //changing this doesnt seem to change the speed
  }
  else {
    //slow lots of whine
    TCCR1A = B10100001; //_BV(COM2A1) | _BV(COM2B1) | _BV(WGM20); //phase correct PWM, compare match A and compare match B
    TCCR1B = 0b00000010;  // 1/8 prescale,  
    OCR1A = 12; //changing this doesnt seem to change the speed
    OCR1B = 12; //changing this doesnt seem to change the speed
  }
}

void SetDirection(bool rightDirectionPin, bool leftDirectionPin)
{
  // stop and delay to protect the circuits from frying on a reversal of direction
  Stop();
  delay(directionChangeDelay);
  
  //extra delay to protect circuits from frying when moving fast
  if (fast){
    delay(directionChangeDelay);  
  }
  
  //set the arduino pins to the right direction
  digitalWrite(rDirPin, rightDirectionPin);
  digitalWrite(lDirPin, leftDirectionPin);

  // start the motors up
  SetPWM();
}

void Forward()
{
  SetDirection(LOW, LOW);
}

void Backward()
{
  SetDirection(HIGH, HIGH);
}

void Left()
{
  SetDirection(LOW, HIGH);
}


void Right()
{
  SetDirection(HIGH, LOW);
}

void Stop()
{
  Serial.println("stop");
  digitalWrite(rightMotorPin, LOW);
  digitalWrite(leftMotorPin, LOW);
}
