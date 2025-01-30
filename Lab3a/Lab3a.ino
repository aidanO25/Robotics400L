#include <Pololu3piPlus32U4.h>
#include <Servo.h>

using namespace Pololu3piPlus32U4;

Buzzer buzzer;
Servo headServo; // create servo object to control a servo

// Switches
const boolean head_debug = false;
const boolean timing_debug = false;
const boolean us_debug = true;

const boolean servo_on = true;
const boolean us_on = true;

// head servo timing
unsigned long headCm;
unsigned long headPm;
const unsigned long head_movement_period = 120;

// head servo constants
const int head_servo_pin = 22;
const int num_head_positions = 4;
const int head_positions[num_head_positions] = {130, 100, 70, 40};

// head servo data
boolean headDirectionClockwise = false; 
int currentHeadPosition = 1;

// initialize ultrasonic
const int echo = 5;
const int trig = 4;

// ultrasonic max distance
const float maxD = 300.0;

// determine the nomalization factorbased on max distance
const float distance_factor = maxD / 100;
const float stopD = 5; // stop distance

// ultrasonic timing
unsigned long usCm;
unsigned long usPm;
const unsigned long wait_after_head_starts_moving = 80; // time to wait between
boolean usReadFlag = false; // ensures 1 reading

// current US distance reading
int currentReadPosition = 0; // current head position

float distanceReadings[num_head_positions];




void setup() {
  // put your setup code here, to run once:
  Serial.begin(57600);

  // initialize the head position to start
  headServo.attach(head_servo_pin);
  headServo.write(40);

  // initialize the US pins
  pinMode(echo, INPUT);
  pinMode(trig, OUTPUT);

  // initialize distance readings
  for(int i=0; i < num_head_positions; i++)
  {
    distanceReadings[i] = maxD;
  }

  // start delay
  delay(3000);
  buzzer.play("c32");

}

void loop() {
  // performing head movement
  moveHead();

  // update the current distance
  usReadCm();

}

void moveHead()
{
  headCm = millis();
  if(headCm > headPm + head_movement_period)
  {
    // position head to the current position in the array
    if(servo_on)
    {
      headServo.write(head_positions[currentHeadPosition]);
    }

    // the position the US sensor should read at
    currentReadPosition = currentHeadPosition;

    // check timing debug
    if(timing_debug)
    {
      Serial.print("Move head initiated: ");
      Serial.println(headCm);
    }

    // head debug output
    if(head_debug)
    {
      Serial.print(currentHeadPosition);
      Serial.print(" - ");
      Serial.print(head_positions[currentHeadPosition]);
    }

    /*
    set next head position
    moves servo to the next head psoition and changes direction when needed
    */
    if(headDirectionClockwise)
    {
      if(currentHeadPosition >= (num_head_positions -1))
      {
        headDirectionClockwise = !headDirectionClockwise;
        currentHeadPosition--;
      }
      else
      {
        currentHeadPosition++;
      }
    }
    else
    {
      if(currentHeadPosition <= 0)
      {
        headDirectionClockwise = !headDirectionClockwise;
        currentHeadPosition++;
      }
      else
      {
        currentHeadPosition--;
      }
    }
    // reset pervious millis
    headPm = headCm;
    // reset read flag
    usReadFlag = false;
  }
}

void usReadCm()
{
  usCm = millis();
  if(usCm > headPm + wait_after_head_starts_moving && !usReadFlag)
  {
    // timing debug
    if(timing_debug)
    {
      Serial.print("US read initiated: ");
      Serial.println(usCm);
    }
    if(us_on)
    {
      // clears the trig (set low)
      digitalWrite(trig, LOW);
      delayMicroseconds(2);

      // Sets the trig HIGH (Active) for 10 microseconds
      digitalWrite(trig, HIGH);
      delayMicroseconds(10);
      digitalWrite(trig, LOW);

      // reads the echo, returns the second wave travel time in microseconds
      // note the duration (30000 microseconds) that will allow for reading up to 3 m
      long duration = pulseIn(echo, HIGH, 30000);
      // calculating the distance
      float distance = duration * 0.034 / 2; // time of flight equation 

      // apply limits
      if (distance > maxD)distance = maxD;
      if(distance == 0) distance = maxD;

      // assign the value to the current position in the array
      distanceReadings[currentReadPosition] = distance;
    }

    if(timing_debug)
    {
      Serial.print("US read Finished: ");
      Serial.println(millis());
    }

    // displays the distance on the Serial monitor
    if(us_debug)
    {
      Serial.print("Distance Readings: [ ");
      for(int i = 0; i < num_head_positions; i++)
      {
        Serial.print(distanceReadings[i]);
        if(i < (num_head_positions - 1)) Serial.print(" - ");
      }
      Serial.println(" ]");
    }
    usReadFlag = true;
  }
}

