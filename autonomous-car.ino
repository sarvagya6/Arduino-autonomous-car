#include <Wire.h>
#include <Adafruit_MotorShield.h>                              //Import library to control motor shield

Adafruit_MotorShield AFMS = Adafruit_MotorShield();


Adafruit_DCMotor *FRMotor = AFMS.getMotor(1);
Adafruit_DCMotor *FLMotor = AFMS.getMotor(2);
Adafruit_DCMotor *RRMotor = AFMS.getMotor(3);
Adafruit_DCMotor *RLMotor = AFMS.getMotor(4);

byte trig = 2;  //blue wire  //Assign the ultrasonic sensor pins
byte echo = 13; //purple wire
byte stopDist = 45;                               //Minimum distance from an object to stop in cm
float timeOut = 2*(maxDist+10)/100/340*1000000;   //Maximum time to wait for a return signal

byte motorSpeed = 200;                             //The maximum motor speed
int motorOffset = 10;                             //Factor to account for one side being more powerful

void setup() {

  
  Serial.begin(9600);           // set up Serial library at 9600 bps
  //print bunch of empty lines to clear the screen
  for (int i = 0; i < 10; i++) {
    Serial.println();
  }
  Serial.println("Adafruit Motorshield v2 - Autonomous Car");

  AFMS.begin(); 
  
  FLMotor->setSpeed(motorSpeed);
  FRMotor->setSpeed(motorSpeed);
  RLMotor->setSpeed(motorSpeed);
  RRMotor->setSpeed(motorSpeed);

  FLMotor->run(RELEASE);
  FRMotor->run(RELEASE);
  RLMotor->run(RELEASE);
  RRMotor->run(RELEASE);

  pinMode(trig,OUTPUT);                           //Assign ultrasonic sensor pin modes
  pinMode(echo,INPUT);
  delay(3000);

}

void loop() {

  // debug
  // moveForward();
  // delay(1000);
  // stopMove();
  // delay(1000);

  // if reset,

  int distance = getDistance();                   //Check that there are no objects ahead
  if(distance >= stopDist)                        //If there are no objects within the stopping distance, move forward
  {
    moveBackward();

  }
  while(distance >= stopDist)                     //Keep checking the object distance until it is within the minimum stopping distance
  {
    distance = getDistance();
    Serial.println(distance);
    delay(50);
    
  }
  stopMove();                                     //Stop the motors
}

void moveForward() {
  FLMotor->run(FORWARD);
  FRMotor->run(FORWARD);
  RLMotor->run(FORWARD);
  RRMotor->run(FORWARD);
}

void moveBackward() {
  FLMotor->run(BACKWARD);
  FRMotor->run(BACKWARD);
  RLMotor->run(BACKWARD);
  RRMotor->run(BACKWARD);
}

void stopMove() {
  FLMotor->run(RELEASE);
  FRMotor->run(RELEASE);
  RLMotor->run(RELEASE);
  RRMotor->run(RELEASE);
}

int getDistance()                                   //Measure the distance to an object
{
  unsigned long pulseTime;                          //Create a variable to store the pulse travel time
  int distance;                                     //Create a variable to store the calculated distance
  digitalWrite(trig, HIGH);                         //Generate a 10 microsecond pulse
  delayMicroseconds(10);
  digitalWrite(trig, LOW);
  pulseTime = pulseIn(echo, HIGH, timeOut);         //Measure the time for the pulse to return
  distance = (float)pulseTime * 340 / 2 / 10000;    //Calculate the object distance based on the pulse time
  return distance;
}