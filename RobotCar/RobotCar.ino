#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
#include <QTRSensors.h> //Pololu QTR Sensor Library. First you must download and install QTRSensors library

Adafruit_MotorShield AFMS = Adafruit_MotorShield();
//AF_DCMotor motor1(3, MOTOR12_1KHZ ); //create motor #1 using M1 output on Motor Drive Shield, set to 1kHz PWM frequency
//AF_DCMotor motor2(4, MOTOR12_1KHZ ); //create motor #2 using M2 output on Motor Drive Shield, set to 1kHz PWM frequency
Adafruit_DCMotor *motor1 = AFMS.getMotor(3);
Adafruit_DCMotor *motor2 = AFMS.getMotor(4);


#define KP 2 //experiment to determine this, start by something small that just makes your bot follow the line at a slow speed
#define KD  5//experiment to determine this, slowly increase the speeds and adjust this value. ( Note: Kp < Kd) 
#define M1_minumum_speed 150  //minimum speed of the Motor1
#define M2_minumum_speed 150  //minimum speed of the Motor2
#define M1_maksimum_speed 175 //max. speed of the Motor1
#define M2_maksimum_speed 175 //max. speed of the Motor2
#define MIDDLE_SENSOR 4       //number of middle sensor used
#define NUM_SENSORS 4         //number of sensors used
#define TIMEOUT 2500          //waits for 2500 us for sensor outputs to go low
#define EMITTER_PIN 3         //emitterPin is the Arduino digital pin that controls whether the IR LEDs are on or off. Emitter is controlled by digital pin 2
#define DEBUG 0
//sensors 0 through 5 are connected to analog inputs 0 through 5, respectively
QTRSensorsRC qtrrc((unsigned char[]) {A0,A1,A2,A3} ,NUM_SENSORS, TIMEOUT, EMITTER_PIN);
  
unsigned int sensorValues[NUM_SENSORS];
int Dist = 2;
double distance = 0;
void setup()
{
Serial.begin(9600); 
AFMS.begin(1000); 
delay(1500);
manual_calibration();

set_motors(20,20);
AFMS.begin(1000); 
}
  
int lastError = 0;
int last_proportional = 0;
int integral = 0;
int stopcar = 0;
double dist1=0;
double dist2=0;
double dist3=0;
double dist4=0;
double dist5=0;
  
void loop()
{
  dist1=dist2;
  dist2=dist3;
  dist3=dist4;
  dist4=dist5;
  dist5 = digitalRead(Dist);
  //Serial.print("dist5 ");
  //Serial.println(dist5);
  
  distance = (dist1 + dist2 + dist3 + dist4 + dist5)/5;
  //Serial.println("1. " + dist1 + " 2. " + dist2 + " 3. " + dist3 + " 4. " + dist4 + " 5. " + dist5); 
  Serial.println(distance);
  if (distance > 0)
  {
    //stopcar=1;
    set_motors(0,0);
  }

  else 
  {
    //if (stopcar != 1) 
    //{
      unsigned int sensors[4];
      int position = qtrrc.readLine(sensors); //get calibrated readings along with the line position, refer to the QTR Sensors Arduino Library for more details on line position.
    
      
      int error = position - 2000;
      
      int motorSpeed = KP * error + KD * (error - lastError);
      
      //delay(3000);
      
      lastError = error;
      
      int leftMotorSpeed = M1_minumum_speed + motorSpeed;
      int rightMotorSpeed = M2_minumum_speed - motorSpeed;
        
      // set motor speeds using the two motor speed variables above
      set_motors(leftMotorSpeed, rightMotorSpeed);
        /*
        for (unsigned char i = 0; i < NUM_SENSORS; i++)
        {
          Serial.print(sensors[i]);
          Serial.print('\t');
        }
        Serial.println(position);
      
        delay(250);*/
    //}
  }


}
  
void set_motors(int motor1speed, int motor2speed)
{
//Serial.println(motor1speed);
//Serial.println(motor2speed);
if (motor1speed > M1_maksimum_speed ) motor1speed = M1_maksimum_speed;
if (motor2speed > M2_maksimum_speed ) motor2speed = M2_maksimum_speed;
if (motor1speed < 0) motor1speed = 0; 
if (motor2speed < 0) motor2speed = 0; 

motor1->setSpeed(motor1speed); 
motor2->setSpeed(motor2speed);
motor1->run(FORWARD); 
motor2->run(FORWARD);
motor1->run(RELEASE);
motor2->run(RELEASE);




}

//calibrate for sometime by sliding the sensors across the line, or you may use auto-calibration instead
void manual_calibration() {

int i;
for (i = 0; i < 250; i++)
{
qtrrc.calibrate(QTR_EMITTERS_ON);
delay(20);
}
  

Serial.begin(9600);
for (int i = 0; i < NUM_SENSORS; i++)
{
Serial.print(qtrrc.calibratedMinimumOn[i]);
Serial.print(' ');
}
Serial.println();
  
for (int i = 0; i < NUM_SENSORS; i++)
{
Serial.print(qtrrc.calibratedMaximumOn[i]);
Serial.print(' ');
}
Serial.println();
Serial.println();

}



