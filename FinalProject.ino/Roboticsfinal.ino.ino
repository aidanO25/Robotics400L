#include <Pololu3piPlus32U4.h>
#include <Servo.h>

using namespace Pololu3piPlus32U4;

Encoders encoders;
Buzzer buzzer;
Motors motors;
Servo headServo;

// toggle debug
const boolean encoder_debug = false;
const boolean obstacle_debug = true;
const boolean heading_debug = false; // robot heading
const boolean head_debug = false; // servo
const boolean timing_debug = false;
const boolean us_debug = false;
// toggle between on and off for debugging purposes
const boolean servo_on = true;
const boolean us_on = true;

// Head servo variables
const int head_servo_pin = 22;
const int num_head_positions = 3;
const int head_positions[num_head_positions] = {150, 85, 30};
boolean headDirectionClockwise = false;
int currentHeadPosition = 1;
unsigned long headCm;
unsigned long headPm;
const unsigned long head_movement_period = 130;

// Ultrasonic variables
const int echo = 5;
const int trig = 4;
const float maxD = 300.0;
const float distance_factor = maxD / 100;
const float stopD = 5;
unsigned long usCm;
unsigned long usPm;
// think about adjusting between the range of 50 and 100
const unsigned long wait_after_head_starts_moving = 65;
boolean usReadFlag = false;
int currentReadPosition = 0;
float distanceReadings[num_head_positions];

unsigned long currentMillis;
unsigned long prevMillis = 0;
const unsigned long PERIOD = 10;

long counts_left = 0;
long counts_right = 0;
long prev_left = 0;
long prev_right = 0;
bool direction = false;

int forward = 0;

bool beep = false;

const int CLICKS_PER_ROTATION = 12;
const float GEAR_RATIO = 75.84F;
const float WHEEL_DIAMETER = 3.2;
const int WHEEL_CIRCUMFERENCE = 10.0531;

const int NUMBER_OF_GOALS = 1;
float xGoals[NUMBER_OF_GOALS] = {0};
float yGoals[NUMBER_OF_GOALS] = {250};
float current_x = 0.0F;
float current_y = 0.0F;
float current_heading = 0.0F;
const float B = 8.5;  //median distance between both wheels

float Kp = 1.05F;
const float GOAL_THRESHOLD = 2.0F;
int goal_index = 0;
bool goal_x_success = false;
bool goal_y_success = false;
bool all_goals_reached = false;

int wheelSpeed = 150;

void setup() {
  Serial.begin(57600);
  delay(1000);

  buzzer.play("c32");

  prevMillis = millis();
  encoders.init();

  // servo head
  headServo.attach(head_servo_pin);
  headServo.write(40);

  // ultrasonic initialization
  pinMode(echo, INPUT);
  pinMode(trig, OUTPUT);
  // distance reading for each head position
  for (int i = 0; i < num_head_positions; i++) {
    distanceReadings[i] = maxD;
  }
  delay(3000);

  // setup complete
  buzzer.play("c32");
}

void loop() {
  // localization
  currentMillis = millis();
  if (currentMillis - prevMillis >= PERIOD && !all_goals_reached) {

    prevMillis = currentMillis;
    prev_left = counts_left;
    prev_right = counts_right;

    counts_left = -encoders.getCountsLeft();
    counts_right = -encoders.getCountsRight();

    // math for localization
    float Sl = ((counts_left - prev_left) / (CLICKS_PER_ROTATION * GEAR_RATIO) * WHEEL_CIRCUMFERENCE);
    float Sr = ((counts_right - prev_right) / (CLICKS_PER_ROTATION * GEAR_RATIO) * WHEEL_CIRCUMFERENCE);

    float delta_s = (Sr + Sl) / 2;
    current_x += delta_s * cos(current_heading);
    current_y += delta_s * sin(current_heading);
    current_heading += (Sr - Sl) / B;

    current_heading = fmod(current_heading + 2 * M_PI, 2 * M_PI);
    float goal_heading = atan2(yGoals[goal_index] - current_y, xGoals[goal_index] - current_x);
    float error = goal_heading - current_heading;
    error = atan2(sin(error), cos(error));

    // incorporates potential field vased on read position
    float obstacle_potential = calculateObstaclePotential(distanceReadings[currentReadPosition]);

    // updates movement based on goal heading, the error, and obstacle potential
    updateMovement(goal_heading, error, obstacle_potential);

    if (fabs(xGoals[goal_index] - current_x) < GOAL_THRESHOLD && fabs(yGoals[goal_index] - current_y) < GOAL_THRESHOLD) {
      goal_index = (goal_index + 1) % NUMBER_OF_GOALS;
      buzzer.play(">g32");
      if (goal_index == 0) {
        all_goals_reached = true;
        motors.setSpeeds(0, 0);
      }
    }
    // debugging for encoders
    if(encoder_debug){
      Serial.print("Sl: ");
      Serial.print(Sl);
      Serial.print(" Sr: ");
      Serial.println(Sr);
    }

    // debugging for hadings
    if(heading_debug)
    {
      Serial.print("Current heading: ");
      Serial.print(current_heading);
      Serial.print(" Goal heading: ");
      Serial.println(goal_heading);
    }

    // debugging for obstacle potential
    if(obstacle_debug)
    {
      Serial.print(" Obstacle potential: ");
      Serial.println(obstacle_potential);
    }
  }

  // servo head
  headCm = millis();
  if (headCm > headPm + head_movement_period) {
    if (servo_on) {
      headServo.write(head_positions[currentHeadPosition]);
    }
    currentReadPosition = currentHeadPosition;

    if (timing_debug) {
      Serial.print("Move head initiated: ");
      Serial.println(headCm);
    }

    if (head_debug) {
      Serial.print(currentHeadPosition);
      Serial.print(" - ");
      Serial.print(head_positions[currentHeadPosition]);
    }

    if (headDirectionClockwise) {
      if (currentHeadPosition >= (num_head_positions - 1)) {
        headDirectionClockwise = !headDirectionClockwise;
        currentHeadPosition--;
      } else {
        currentHeadPosition++;
      }
    } else {
      if (currentHeadPosition <= 0) {
        headDirectionClockwise = !headDirectionClockwise;
        currentHeadPosition++;
      } else {
        currentHeadPosition--;
      }
    }
    headPm = headCm;
    usReadFlag = false;
  }

  // ultrasonic sensor
  usCm = millis();
  if (usCm > headPm + wait_after_head_starts_moving && !usReadFlag) {
    if (us_on) {
      digitalWrite(trig, LOW);
      delayMicroseconds(2);
      digitalWrite(trig, HIGH);
      delayMicroseconds(10);
      digitalWrite(trig, LOW);
      long duration = pulseIn(echo, HIGH, 30000);
      float distance = duration * 0.034 / 2;
      if (distance > maxD | distance == 0) {
        float sum = 0;
        for (int i = 0; i < num_head_positions; i++) {
          sum += distanceReadings[i];
        }
        distance = sum/num_head_positions;
      }  
      distanceReadings[currentReadPosition] = distance;
    }
    if (us_debug) {
      Serial.print("Distance Readings: [ ");
      for (int i = 0; i < num_head_positions; i++) {
        Serial.print(distanceReadings[i]);
        if (i < (num_head_positions - 1)) Serial.print(" - ");
      }
      Serial.println(" ]");
    }
    usReadFlag = true;
  }
}

// potential fields
float calculateObstaclePotential(float distance) {
  // potential field forces
  const float max_repulsive_force = 80.0;
  const float min_distance = 10.0;
  const float max_distance = 30.0;

  if (distance < min_distance) {
    distance = min_distance;
  }
  if (distance > max_distance) {
    return 0.0;
  }
  float repulsive_force = max_repulsive_force * (1 / distance);
  // returns teh repulsive force which is used in updating the robot's movement
  return repulsive_force;
}

// movement update
void updateMovement(float goal_heading, float error, float obstacle_potential) {
  float correction = Kp * error + obstacle_potential;
  int leftSpeed = wheelSpeed - correction * (180 / M_PI);
  int rightSpeed = wheelSpeed + correction * (180 / M_PI);
  motors.setSpeeds(-leftSpeed, -rightSpeed);
}

