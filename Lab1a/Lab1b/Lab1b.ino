// 12 inches is 30.5 cm
// 18 inches is 45.7 cm

#include <Pololu3piPlus32U4.h>

using namespace Pololu3piPlus32U4;

Encoders encoders;
Buzzer buzzer;
Motors motors;
ButtonA buttonA;

/* this just smooths the stop
        if(Sl > 20)
        {
          wheelSpeed = 100 * ((35 - Sl) / 10);
          if(wheelSpeed < 20) wheelSpeed = 30;
        }
        */


// this is only for encoders, may adapt in the future
unsigned long currentMillis;
unsigned long prevMillis;
const unsigned long PERIOD = 10;

long countsLeft = 0;
long countsRight = 0;
long prevLeft = 0;
long prevRight = 0;

// boolean to figure out when it should go forwards/backwards
bool direction = false;
// int forward to track how many times its moved first
int forward = 0;
// tracking if the robot has beeped
bool beep = false;

// turtle edeition 180 RPM
const int CLICKS_PER_ROTATION = 12;
const float GEAR_RATIO = 75.84F;
const float WHEEL_DIAMETER = 3.2;
const int WHEEL_CIRCUMFERENCE = 10.0531;

float Sl = 0.0F;
float Sr = 0.0F;

void setup() 
{
  // put your setup code here, to run once:
  Serial.begin(57600);
  delay(1000);
  buzzer.play("c32");


}

void loop()
{
  // put your main code here, to run repeatedly:
  checkEncoders();
  
}

void checkEncoders()
{
  currentMillis = millis();
  if(currentMillis > prevMillis + PERIOD)
  {
    countsLeft += encoders.getCountsAndResetLeft(); 
    countsRight += encoders.getCountsAndResetRight();

    Sl += ((countsLeft - prevLeft) / (CLICKS_PER_ROTATION * GEAR_RATIO) * WHEEL_CIRCUMFERENCE);
    Sr += ((countsRight - prevRight) / (CLICKS_PER_ROTATION * GEAR_RATIO) * WHEEL_CIRCUMFERENCE);

    // initial wheelSpeed
    int wheelSpeed = 150;

    // first time going forward (12 in)
    if(forward == 0)
    {
      // continues to run the commands below until the robot reaches 12cm
      if(Sl < 29.6)
      {
        // slowing the robot down as it approaches it's target
        if(Sl > 20)
        {
          wheelSpeed = 150 * ((30.5 - Sl) / 10);
          if(wheelSpeed < 20) wheelSpeed = 30;
        }
        // speed of the wheels factoring in the above caculation
        motors.setSpeeds(wheelSpeed, wheelSpeed);
      }
      // counts how many times the robot has moved forward and changes the direction boolean to true, telling the robot it is ready to move backwards
      else
      {
        forward = 1;
        direction = true;
      }
    }
    // checks if it should move backwards 
    if(direction == true)
    {
      // continues to run the commands below until the robot reaches 0cm
      if(Sl > -0.4)
      {
        // slowing the robot down as it approaches it's target
        if(Sl < 10)
        {
          wheelSpeed = 150 * ((1 + Sl) / 10);
          if(wheelSpeed < 20) wheelSpeed = 30;
        }
        // speed of the wheels factoring in the above caculation
        motors.setSpeeds(-wheelSpeed, -wheelSpeed);
      }
      // Changes the direction boolean to false, telling the robot it is ready to move forwards again
      else
      {
        direction = false;
      }
    }

    // checks if the robot has already gone forwards once and if it is done going backwards
    if(forward == 1 && direction == false)
    {
      // continues to run the commands below until the robot reaches 18in
      if(Sl < 45.000)
      {
        // slowing the robot down as it approaches it's target
        if(Sl > 35)
        {
          wheelSpeed = 150 * ((45.5 - Sl) / 10);
          if(wheelSpeed < 20) wheelSpeed = 30;
        }
        // speed of the wheels factoring in the above caculation
        motors.setSpeeds(wheelSpeed, wheelSpeed);
      }
      // if there is nothing else for the robot to do...
      else
      {
        // stopping the robot
        motors.setSpeeds(0, 0);
        // if statements to let the robot beep once and stopping once it has done so
        if(beep == false)
        {
          buzzer.play("c32");
          // delay to let the buzzer beep until changing the beep value to true, stopping the sound
          delay(250);
          beep = true;
        }
        else
        {
          // stopping the buzzer
          buzzer.stopPlaying();
        }

      }
    }


    // printing the motor rotational values
    Serial.print("Left: ");
    Serial.print(Sl);
    Serial.print(" Right: ");
    Serial.println(Sr);

    prevLeft = countsLeft;
    prevRight = countsRight;
    prevMillis = currentMillis;
  }

}




