//********************************************************//
//*  University of Nottingham                            *//
//*  Department of Electrical and Electronic Engineering *//
//*  Nat Dacombe & Alex Ottway                           *//
//*  UoN EEEBot 2022                                     *//
//*  Motor & Servo Basic Test Code                       *//
//********************************************************//

// ASSUMPTION: Channel A is LEFT, Channel B is RIGHT

// Use this code to correctly assign the four pins to move the car forwards, backwards, clockwise & counter-clockwise
// You first need to change the pin numbers for the four motor 'IN' pins below and then decide which go HIGH and LOW in 
// each of the movements, stopMotors has been done for you
// ** marks where you need to insert the pin number or state

// feel free to modify this code to test existing or new functions

#include <Servo.h>    //include the servo library
#define servoPin 4
Servo myservo;        // create servo object to control a servo
#include <Encoder.h>
Encoder myEnc(11,2);
float steeringAngle;  // variable to store the servo position
float distance;

#define enA 5   //EnableA command line - should be a PWM pin
#define enB 6   //EnableB command line - should be a PWM pin

//name the motor control pins - replace the CHANGEME with your pin number, digital pins do not need the 'D' prefix whereas analogue pins need the 'A' prefix
#define INa 14  //Channel A direction 
#define INb 15  //Channel A direction 
#define INc 16  //Channel B direction 
#define INd 17  //Channel B direction 

byte speedSetting = 0;  //initial speed = 0


void setup() {
  // put your setup code here, to run once:
  myservo.write(100);
  myservo.attach(servoPin);  //attach our servo object to pin D4
  //the Servo library takes care of defining the PinMode declaration (libraries/Servo/src/avr/Servo.cpp line 240)

  //configure the motor control pins as outputs
  pinMode(INa, OUTPUT);
  pinMode(INb, OUTPUT);
  pinMode(INc, OUTPUT);
  pinMode(INd, OUTPUT);
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);   

  //initialise serial communication
  Serial.begin(9600);
  Serial.println("Arduino Nano is Running"); //sanity check
}


void loop() {
  myservo.write(52); // sets angle of wheels to straight
  distance = 0;
  long newPosition = 0;
  long oldPosition = -999;
  
  void checkDist()while (distance <=10 )
  {
    newPosition = myEnc.read();
  
  // check if encoder has moved
    if (newPosition != oldPosition) 
  {
    oldPosition = newPosition;
    
  // edit the code below to calculate the distance moved, +1 increment = (diameter*pi)/encoder count per revolution
    distance = newPosition * 1.0;
                  
    Serial.print("Distance(m): ");
    Serial.println(distance);
    }
    stopMotors


void motors(int leftSpeed, int rightSpeed) {
  //set individual motor speed
  //direction is set separately

  analogWrite(enA, leftSpeed);
  analogWrite(enB, rightSpeed);
}

//for each of the below function, two of the 'IN' variables must be HIGH, and two LOW in order to move the wheels - use a trial and error approach to determine the correct combination for your EEEBot
void goForwards() {
  digitalWrite(INa, LOW);
  digitalWrite(INb, HIGH);
  digitalWrite(INc, LOW);
  digitalWrite(INd, HIGH);
}

void stopMotors() {
  digitalWrite(INa, LOW);
  digitalWrite(INb, LOW);
  digitalWrite(INc, LOW);
  digitalWrite(INd, LOW);
}}
