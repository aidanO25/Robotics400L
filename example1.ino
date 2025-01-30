/* this is one program that doesn't have anything to do with the servo, you just have to make sure your ultrasonic sensor
is positioned properly. currently it gives a reading of the ultrasonic sensor, and depending on how far the reading is, the motors
will speed up or slow down. its basically lab2b. my idea was to take out the if statements for getting it to slow down at a certain point
and just throw in the PID stuff. what im trying to say is just take out everything except what makes the motors move so we can have it moving
strait while the sensor reads 10 and once it starts to deviate, move whatever wheel (most likely the right as the sensor faces left) to get it
back on track. getting it back on track is really the hard part of this whole thing
*/


#include <Pololu3piPlus32U4.h>
using namespace Pololu3piPlus32U4;

Buzzer buzzer;
Motors motors;

// Initialize Ultrasonic
const int echo = 5;
const int trig = 4;

// Ultrasonic Maxs
const int maxD = 100; //(200 cm / 2 meters)

// determine the nomalization factor based on maxD
const float df = maxD / 100;// max distance
const float stopD = 5; // stop distance

// motor constants
const float mbs = 300.0; // motor based speed
const int mms = 50; // motor min speed
// determine the normalization factor based on mbs
const float mf = mbs / 100; // motor factor

// motor compensation (if needed)
const float l_mf= 1.0; // left motor factor
const float r_mf = 1.0; // right motor factor
const float l_mf_thresh = 80; // left motor factor threshold
const float r_mf_thresh = 80; // right motor factor threshold

// ultasonic timing
unsigned long usCm; // ultasonic current millis
unsigned long usPm; // ultasonic previous millies
const unsigned long usPeriod = 50;

// motor timing
unsigned long mCm; // motor current millis
unsigned long mPm; // motor previous millis
const unsigned long mPeriod = 20;

// current US distnace reading
float distance = 0;

void setup() 
{
  // put your setup code here, to run once:

  pinMode(echo, INPUT);
  pinMode(trig, OUTPUT);

  delay(1000);
  //buzzer.play("c32");
}

void loop() {
  // put your main code here, to run repeatedly:

  // update the current distance
  usReadCm();

  // update the motor speeds
  setMotors();

}

void usReadCm()
{
  usCm = millis();
  if(usCm > usPm + usPeriod)
  {
    // clears the TRIP_PIN (set low)
    digitalWrite(trig, LOW);
    delayMicroseconds(2);

    // sets the TRIG_PIN HIGH (ACTIVE) for 10 microseconds
    digitalWrite(trig, HIGH);
    delayMicroseconds(10);
    digitalWrite(trig, LOW);

    // Reads the ECHO)PIN,  returns the sound wave travel time in microseconds
    // note the duration (38,000 microseconds) that will allow for reading up max distance supported by teh sensor
    long duration = pulseIn(echo, HIGH, 38000);

    // Calculating the distance
    distance = duration * 0.034 / 2; // Time of flight equation: speed of sound wave divided by 2

    //apply limits
    if(distance > maxD) distance = maxD;
    if(distance == 0) distance = maxD;

    // displays the distance on the Serial Monitor
    
    Serial.print("Distance: ");
    Serial.print(distance);
    Serial.println(" cm");
    

    // update the prev millis
    usPm = usCm;
  }
}

void setMotors()
{
  mCm = millis();
  if(mCm > mPm + mPeriod)
  {
    // start out with the motor base speed
    float lSpeed = mbs;
    float rSpeed = mbs;

    // check to see if most current distance measurement is less than / equal to max distance
    if(distance <= maxD)
    {
      float magnitude = (float)(maxD - distance) / df;

      lSpeed = mbs - (magnitude * mf);
      rSpeed = mbs - (magnitude * mf);
    }
    


    // lower limit check
    if(lSpeed < mms) lSpeed = mms;
    if(rSpeed < mms) rSpeed = mms;

    // check stop distance
    if(distance <= stopD) lSpeed = 0;
    if(distance <= stopD) rSpeed = 0;

    /*
    Serial.print("Left: ");
    Serial.print(lSpeed);
    Serial.print(" Right ");
    Serial.println(rSpeed);
    */

    motors.setSpeeds(-lSpeed, -rSpeed);

    mPm = mCm;
  }
}
