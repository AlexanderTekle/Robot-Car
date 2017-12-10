#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 

Adafruit_DCMotor *myMotor = AFMS.getMotor(3);
Adafruit_DCMotor *myMotor2 = AFMS.getMotor(4);




void setup()
{
  Serial.begin(9600);
  Serial.println("Testing motors");

  AFMS.begin();

  myMotor->setSpeed(150);
  myMotor2->setSpeed(150);
  
  myMotor->run(FORWARD);
  myMotor->run(RELEASE);
  myMotor2->run(FORWARD);
  myMotor2->run(RELEASE);
}

void loop()
{
  myMotor->run(FORWARD);
  myMotor2->run(FORWARD);
  
  Serial.println("FORWARDLOOP");

  myMotor->run(RELEASE);
  myMotor2->run(RELEASE);

  Serial.println("RELEASELOOP");


}
