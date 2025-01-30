#include <Pololu3piPlus32U4.h>

using namespace Pololu3piPlus32U4;

// Initialization of components and variables
Encoders encoders;
Buzzer buzzer;
Motors motors;
ButtonA buttonA;


// RefactoredWithA

const boolean encoder_debug = true;

// Encoder timing and count variables
unsigned long currentMillis;
unsigned long prevMillis = 0;
const unsigned long PERIOD = 10;

long counts_left = 0;
long counts_right = 0;
long prev_left = 0;
long prev_right = 0;

// Movement and goal management
bool direction = false;
int forward = 0;
bool beep = false;
const int CLICKS_PER_ROTATION = 12;
const float GEAR_RATIO = 75.84F;
const float WHEEL_DIAMETER = 3.2;
const int WHEEL_CIRCUMFERENCE = 10.0531;

// Goals
const int NUMBER_OF_GOALS = 4;
//float xGoals[NUMBER_OF_GOALS] = {60, 80, -60, 0};
//float yGoals[NUMBER_OF_GOALS] = {0, 50, -30, 0};
float xGoals[NUMBER_OF_GOALS] = {30, 30, 0};
float yGoals[NUMBER_OF_GOALS] = {30, 60, 0};
// Current position and heading
float current_x = 0.0F;
float current_y = 0.0F;
float current_heading = 0.0F; // in radians
const float B = 8.5; // wheel base in cm
float Kp = .85F; // .80F tighter than .95F, 

// Threshold for reaching a goal
const float GOAL_THRESHOLD = 2.0F;
int goal_index = 0; 
bool goal_x_success = false;
bool goal_y_success = false;

int wheelSpeed = 70;

void setup() {
    Serial.begin(57600);
    delay(1000);
    buzzer.play("c32");
    prevMillis = millis();
    encoders.init();
}

void loop() {
  currentMillis = millis();
  if (currentMillis - prevMillis >= PERIOD) {
    prevMillis = currentMillis;
    // Get encoder counts and compute wheel displacements
    prev_left = counts_left;
    prev_right = counts_right;
    counts_left = encoders.getCountsLeft();
    counts_right = encoders.getCountsRight(); 

    // Compute displacements
    float Sl = ((counts_left - prev_left) / (CLICKS_PER_ROTATION * GEAR_RATIO) * WHEEL_CIRCUMFERENCE);
    float Sr = ((counts_right - prev_right) / (CLICKS_PER_ROTATION * GEAR_RATIO) * WHEEL_CIRCUMFERENCE);
    float delta_s = (Sr + Sl) / 2;

    // Update current position
    current_x += delta_s * cos(current_heading);
    current_y += delta_s * sin(current_heading);

    // Update heading
    current_heading += (Sr - Sl) / B;
    current_heading = fmod(current_heading + 2 * M_PI, 2 * M_PI); // Normalize heading

    // Compute goal heading
    float goal_heading = atan2(yGoals[goal_index] - current_y, xGoals[goal_index] - current_x);
    float error = goal_heading - current_heading;
    error = atan2(sin(error), cos(error)); // Normalize error

    // Control motors based on heading error
    float correction = Kp * error;
    int leftSpeed = wheelSpeed - correction * (180 / M_PI); // Convert radian error to degrees for scaling
    int rightSpeed = wheelSpeed + correction * (180 / M_PI);
    motors.setSpeeds(leftSpeed, rightSpeed);

    // Check if current goal is reached
    if (fabs(xGoals[goal_index] - current_x) < GOAL_THRESHOLD && fabs(yGoals[goal_index] - current_y) < GOAL_THRESHOLD) {
     
      if ( (goal_index + 1)  >= NUMBER_OF_GOALS) {
        motors.setSpeeds(0,0);
        buzzer.play(">g32"); // Signal goal reached
        
        
      } else {
        goal_index = (goal_index + 1) % NUMBER_OF_GOALS; // Move to next goal
        buzzer.play(">g32"); // Signal goal reached
      }
    }

    if (goal_index > NUMBER_OF_GOALS) {
      
    }
      
    
  }

  // Debug outputs
  Serial.print("Current X: "); Serial.print(current_x);
  Serial.print(" Current Y: "); Serial.print(current_y);
  Serial.print(" Heading: "); Serial.print(current_heading * (180 / M_PI));
  Serial.print(" Goal X: "); Serial.print(xGoals[goal_index]);
  Serial.print(" Goal Y: "); Serial.println(yGoals[goal_index]);
}