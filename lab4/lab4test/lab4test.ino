
// 12 inches is 30.5 cm
// 18 inches is 45.7 cm

#include <Pololu3piPlus32U4.h>

using namespace Pololu3piPlus32U4;

Encoders encoders;
Buzzer buzzer;
Motors motors;
ButtonA buttonA;

// debuging
const boolean encoder_debug = true;


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

// wheels and motors
const int CLICKS_PER_ROTATION = 12;
const float GEAR_RATIO = 75.84F;
const float WHEEL_DIAMETER = 3.2;
const int WHEEL_CIRCUMFERENCE = 10.0531;

const double left_motor_base_speed = 80.0;
const double right_motor_base_speed = 80.0;
const double motor_min_speed = 40.0;
const double motor_max_speed = 100.0;

double leftSpeed = left_motor_base_speed;
double rightSpeed = right_motor_base_speed;

// goals
const int number_of_goals = 3;
int goalNumber = 0; 
float xGoals[number_of_goals] = {30, 30, 0};
float yGoals[number_of_goals] = {30, 60, 0};

float Sl = 0.0F;
float Sr = 0.0F;
double prevSl = 0;    //distance along y robot has traveled in the previous iteration
double prevSr = 0;    //distance along y robot has traveled in the previous iteration

double deltaS = 0;
double deltaTheta = 0;
double deltaX = 0;
double deltaY = 0;

// the distance between the 2 drive wheels form the center point of the contact patches
const float B = 8.5;


int x = 0;
int y = 0;

//difference between these values and Sl and Sr is that these are the coordinates, Sl and Sr are strictly distance and has no sense of direction
double currX = 0;    //distance along x robot has traveled for the current goal
double currY = 0;    //distance along y robot has traveled for the current goal
double prevX = 0;    //distance along x robot has traveled in the previous iteration
double prevY = 0;


double thetaGoal = 0;
double currTheta = 0;
double robotDist = 0;

// pid
const double Kp = 0.5;
const double pid_upper_lim = 10000;    //so high that it is irrelevant
const double pid_lower_lim = -10000;   //so low that it is irrelevant
const double integral_max = 1;
const double integral_min = -1;
double totalError = 0;  //used for integral
double prevError = 0;   //used for derivative
double pid = 0;



void setup() 
{
  // put your setup code here, to run once:
  Serial.begin(57600);
  delay(1000);
  
  // Initialize encoder counts and previous millis
  prevMillis = millis();
  
  // play buzzer when ready
  buzzer.play("c32");
  

}

void loop()
{
  // put your main code here, to run repeatedly:
  
  // Check if Goal met
  // Perform Cals
  // Check 

    /*
    for(int i = 0; i < number_of_goals; i++)
    {
      x = xGoals[i];
      y = yGoals[i];

      if(x - deltaX !=0 && y - deltaY !=0)
      {
        // updating values
        checkEncoders();
        calculateGoalOrientation(y, x);
        calculateCurrentHeading();

        // moving the robot
        move();
      }

    }
    */



  if (! ( ( (xGoals[goalNumber] <= currX) && (currX <= xGoals[goalNumber]) ) && ( (yGoals[goalNumber] <= currY) && (currY <= yGoals[goalNumber]) ) ) && goalNumber < number_of_goals) //has not reached goal yet
  {
    checkEncoders();
    computePID();
    slowdown();
    adjustSpeeds();
  }
  else //goal is reached
  {
    goalNumber += 1;  //goal is reached, checking for next goal

    if (number_of_goals == (goalNumber) ) //all goals have been reached
    {
      motors.setSpeeds(0, 0);
      delay(1000);
      buzzer.play("F5");
    }
    else if (number_of_goals < (goalNumber) )  //same as above but don't want the buzzer to be continuous
    {
      motors.setSpeeds(0, 0);
    }
    else   //more goals to be reached, increment goal tracker counter
    {
      double yGoalAdjust = yGoals[goalNumber] - currY;
      double xGoalAdjust = xGoals[goalNumber] - currX;
      thetaGoal = atan2(yGoalAdjust, xGoalAdjust);
      motors.setSpeeds(0, 0);
      buzzer.play("A5");
      delay(1000);

      if(thetaGoal < 0)
      {
        double thetaComp = (PI * 2) - abs(thetaGoal);
        thetaGoal = thetaComp;
      } 
      
    }
  }
}



void checkEncoders()
{
  currentMillis = millis();
  if(currentMillis > prevMillis)
  {
    countsLeft += encoders.getCountsAndResetLeft(); 
    countsRight += encoders.getCountsAndResetRight();


    Sl += ((countsLeft - prevLeft) / (CLICKS_PER_ROTATION * GEAR_RATIO) * WHEEL_CIRCUMFERENCE);
    Sr += ((countsRight - prevRight) / (CLICKS_PER_ROTATION * GEAR_RATIO) * WHEEL_CIRCUMFERENCE);

    deltaS = ((Sr - prevSr) + (Sl - prevSl)) / 2;
    robotDist += deltaS;
    Serial.println(robotDist);

    deltaTheta = ((Sr - prevSr) - (Sl - prevSl)) / B; 
    currTheta += deltaTheta;
    Serial.println(currTheta);

    deltaY = deltaS * cos(thetaGoal + (deltaTheta / 2));
    currX += deltaX;
    Serial.println(currX);

    deltaY = deltaS * sin(thetaGoal + (deltaTheta / 2));
    currY += deltaY;
    Serial.println(currY);
    

    prevLeft = countsLeft;
    prevRight = countsRight;
    prevSl = Sl;
    prevSr = Sr;
    prevX = currX;
    prevY = currY;
  }  

}

/*
void move() {
  int wheelSpeed = 100;
  float error = goal_orientation - current_heading;
  int correction = Kp * error;
  int leftSpeed = wheelSpeed + correction;
  int rightSpeed = wheelSpeed - correction;
  motors.setSpeeds(leftSpeed, rightSpeed); // this is what is actually geting the robot to move
  
}
*/



void computePID()
{
  double error = currTheta - thetaGoal;

  pid = Kp * error;

  if (pid > pid_upper_lim)
  {
    pid = pid_upper_lim;
  }
  if (pid < pid_lower_lim)
  {
    pid = pid_lower_lim;
  }
  leftSpeed = left_motor_base_speed + pid;
  rightSpeed = right_motor_base_speed - pid;
}

void adjustSpeeds()
{
  if (leftSpeed > motor_max_speed)
  {
    leftSpeed = motor_max_speed;
  }
  if (rightSpeed > motor_max_speed)
  {
    rightSpeed = motor_max_speed;
  }

  if (leftSpeed < motor_min_speed)
  {
    leftSpeed = motor_min_speed;
  }
  if (rightSpeed < motor_min_speed)
  {
    rightSpeed = motor_min_speed;
  }

  motors.setSpeeds(leftSpeed, rightSpeed);
}

void slowdown()
{
  double xDist = xGoals[goalNumber] - currX;
  double yDist = currY - yGoals[goalNumber];
  double distToGo = sqrt((xDist * xDist) + (yDist * yDist));

  if (distToGo < 15)
  {
    leftSpeed = leftSpeed - (15 - distToGo);   //if distToGo is 4.9, then will only subtract 5.1 off speed
    rightSpeed = rightSpeed - (15 - distToGo); //when distToGo reached .1, then will subtract 9.9 off speed and robot will almost be at base speed
  }
}

