#include <Pololu3piPlus32U4.h>
#include <Servo.h>
// Utilizing namespaces for easier use of Pololu classes
using namespace Pololu3piPlus32U4;

Motors motors;
Buzzer buzzer;
Servo headServo;

const boolean DEBUG = true; // MANUALLY toggle


// servo
const int HEAD_SERVO_PIN = 22;
const int head_positions[2] = {130, 170};
int head_positions_distances[2] = {0, 0};


// ultrasonic
const int ECHO_PIN = 5;
const int TRIG_PIN = 4;
const float MAX_DISTANCE = 100.0;
const float DISTANCE_FACTOR = MAX_DISTANCE / 100;
const float STOP_DISTANCE = 30;
const unsigned long US_PERIOD = 100;
int distance; 



// pid
int integral = 0;
float totalErr = 0; // Integral component
float prevErr = 0; // Previous error, for the derivative component
float pid = 0;
// PID control constants
const float Kp = 1.5;  // Proportional gain
const float Ki = 0.015;  // Integral gain
const float Kd = 3.0;  // Derivative gain
float startingPID = -200.0;
unsigned long pidMillisCurrent;
unsigned long pidMillisPrev;
const float PID_MAX = 10000;    //so high that it is irrelevant
const float PID_MIN = -10000;   //so low that it is irrelevant
const float INTEGRAL_MAX = 1;
const float INTEGRAL_MIN = 1;


// motors
const float LEFT_MOTOR_BASE_SPEED = 50.0;  // Base speed for left motor
const float RIGHT_MOTOR_BASE_SPEED = 50.0; // Base speed for right motor
int leftAngleIndex = 0;
int forwardLeftAngleIndex = 1;
unsigned long currentMillis;
unsigned long previousMillis;



void setup() {
  // put your setup code here, to run once:
  Serial.begin(57600); // Start the serial communication
  //motors.flipLeftMotor(true);
  // Attach servo and initialize positions
  headServo.attach(HEAD_SERVO_PIN);
  headServo.write(head_positions[0]); // Initial position
  // Set up ultrasonic sensor pins
  pinMode(ECHO_PIN, INPUT);
  pinMode(TRIG_PIN, OUTPUT);

  buzzer.play("c32");   
  //getStartingReading();
}

void loop() {
  Serial.println("  -- LOOP ---------------------------------");
  moveHead(); // Move and Read
  computePID(); // Compute and Set Motors
  delay(500);
}

// servo
void moveHead() {
  headServo.write(head_positions[leftAngleIndex]);
  delay(200);
  int leftAngleDistance = usReadCm();
  head_positions_distances[leftAngleIndex] = leftAngleDistance;
  headServo.write(head_positions[forwardLeftAngleIndex]);
  delay(200);
  int forwardLeftAngleDistance = usReadCm();
  head_positions_distances[forwardLeftAngleIndex] = forwardLeftAngleDistance;
}


// pid
float computePID() {

  pidMillisCurrent = millis();
  distance = (head_positions_distances[0] + head_positions_distances[1]) / 2; // Average of distances
  
  float err = distance - STOP_DISTANCE;
  float p = computeProportional(err);
  float i = computeIntegral(err);
  float d = computeDerivative(err);
  pid = p + i + d;


  if(pid > PID_MAX) { pid = PID_MAX; }
  if(pid < PID_MIN) { pid = PID_MIN; }
  
  pidMillisPrev = pidMillisCurrent;

  if (startingPID < -40.0) {
    startingPID = pid;
  }

  Serial.print(" | Err: "); Serial.println(err);
  Serial.print(" | P, I, D: "); Serial.print(p); Serial.print(", "); Serial.print(i); Serial.print(", "); Serial.print(d); 
  Serial.print(" => "); Serial.println(pid);

  setMotors(pid);

}

float computeProportional(float err) {
  float prop = Kp * err;
  return prop;
}


float computeIntegral(float err) {
  totalErr += err * (pidMillisCurrent - pidMillisPrev);
  float integral = Ki * totalErr;
  if ( integral > INTEGRAL_MAX ) { integral = INTEGRAL_MAX; }
  if ( integral < INTEGRAL_MIN ) { integral = INTEGRAL_MIN; }
  return integral;   
}


float computeDerivative(float err) {
  float r = (err - prevErr) / (pidMillisCurrent - pidMillisPrev); //rate
  float derivative = Kd * r;
  prevErr = err;
  return derivative;
}



// motors
void setMotors(float output) {
  float speedAdjustment = constrain(output, -50, 50);
                       // where constrain limits
                       // number to range between x&y

  Serial.print(" | constrained speedAdjustment: " ); Serial.println(speedAdjustment);

  Serial.print(" | Starting PID: "); Serial.println(startingPID);

  int recalc_distance = (head_positions_distances[0] + head_positions_distances[1]) / 2; // Because it is not in global scope
  
  if (recalc_distance < STOP_DISTANCE) {
    Serial.println(" | (?) LESS THAN STOP DISTANCE - RIGHT"); Serial.println(" | (?) MOVE: RIGHT");
        float leftSpeed = LEFT_MOTOR_BASE_SPEED + speedAdjustment*3;
        float rightSpeed = RIGHT_MOTOR_BASE_SPEED;
        if (DEBUG) {
                  Serial.print(" | Left Speed: "); Serial.print(leftSpeed);
                  Serial.print(", Right Speed: "); Serial.println(rightSpeed);
        } motors.setSpeeds(-leftSpeed, -rightSpeed);
        delay(200);
  
  } else {
  
    if (output > startingPID) {
      // overs shot to the right, correct it
      Serial.print(" | (?) MOVE: LEFT");
      float leftSpeed = LEFT_MOTOR_BASE_SPEED + speedAdjustment;
      float rightSpeed = RIGHT_MOTOR_BASE_SPEED;
      if( DEBUG ) {
          Serial.print(" | Left Speed: "); Serial.print(leftSpeed);
          Serial.print(", Right Speed: "); Serial.println(rightSpeed);
      } motors.setSpeeds(-leftSpeed, -rightSpeed);
    } 
    
    else if ( output < startingPID) {
      // over shot to the left, correct it
      Serial.print(" | (?) MOVE: RIGHT");
      float rightSpeed = RIGHT_MOTOR_BASE_SPEED - speedAdjustment;
      float leftSpeed = LEFT_MOTOR_BASE_SPEED;
      
      if( DEBUG ) {
        Serial.print(" | Left Speed: "); Serial.print(leftSpeed);
        Serial.print(", Right Speed: "); Serial.println(rightSpeed);
      } motors.setSpeeds(-leftSpeed, -rightSpeed);
    } 

    else {
      // otherwise, stay straight
      Serial.print(" | (?) MOVE: STRAIGHT");
      float leftSpeed = LEFT_MOTOR_BASE_SPEED;
      float rightSpeed = RIGHT_MOTOR_BASE_SPEED;
      if ( DEBUG ) {
        Serial.print(" | Left Speed: "); Serial.print(leftSpeed);
        Serial.print(", Right Speed: "); Serial.println(rightSpeed);
      } motors.setSpeeds(-leftSpeed, -rightSpeed);
    }

  }

}

// ultrasonic sensor
int usReadCm() {
 
 currentMillis = millis();
 
 if(currentMillis > previousMillis + US_PERIOD) {
   // clears the TRIP_PIN (set low)
   digitalWrite(TRIG_PIN, LOW);
   delayMicroseconds(2);

   // sets the TRIG_PIN HIGH (ACTIVE) for 10 microseconds
   digitalWrite(TRIG_PIN, HIGH);
   delayMicroseconds(10);
   digitalWrite(TRIG_PIN, LOW);

   // Reads the ECHO)PIN,  returns the sound wave travel time in microseconds
   // note the duration (38,000 microseconds) that will allow for reading up max distance supported by teh sensor
   long duration = pulseIn(ECHO_PIN, HIGH, 38000);

   // Calculating the distance
   distance = duration * 0.034 / 2; // Time of flight equation: speed of sound wave divided by 2

   //apply limits
   if(distance > MAX_DISTANCE) distance = MAX_DISTANCE;
   if(distance == 0) distance = MAX_DISTANCE;

   // displays the distance on the Serial Monitor
   if(DEBUG) {
     Serial.print(" | Distance: ");
     Serial.print(distance);
     Serial.println(" cm  ");
   }
   // update the prev millis
   previousMillis = currentMillis;

   return distance;
 }
}