#include <Pololu3piPlus32U4.h>
#include <Servo.h>

// Utilizing namespaces for easier use of Pololu classes
using namespace Pololu3piPlus32U4;

Motors motors;
Buzzer buzzer;
Servo headServo;



// PID From Scratch


const boolean DEBUG = true; // MANUALLY toggle


// servo
const int HEAD_SERVO_PIN = 22;
const int head_positions[2] = {140, 170};
int head_positions_distances[2] = {0, 0};

float pid_storage[3] = {0.0, 0.0, 0.0};
int pid_storage_pointer = 0;

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
const float Kp = 0.58;  // Proportional gain
//const float Kp = 0.58;  // Proportional gain
const float Ki = 0.55;  // Integral gain
const float Kd = 6.0;  // Derivative gain
float startingPID = -200.0;
unsigned long pidMillisCurrent;
unsigned long pidMillisPrev;
const float PID_MAX = 10000;    //so high that it is irrelevant
const float PID_MIN = -10000;   //so low that it is irrelevant
const float INTEGRAL_MAX = 10;
const float INTEGRAL_MIN = 0;




// motors
const float LEFT_MOTOR_BASE_SPEED = 35.0;  // Base speed for left motor
const float RIGHT_MOTOR_BASE_SPEED = 35.0; // Base speed for right motor
int leftAngleIndex = 0;
int forwardLeftAngleIndex = 1;
unsigned long currentMillis;
unsigned long previousMillis;

boolean invalid_pid = false;

//boolean all_angles_checked = false;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(57600); // Start the serial communication
  headServo.attach(HEAD_SERVO_PIN);
  headServo.write(140); // Initial position
  pinMode(ECHO_PIN, INPUT);
  pinMode(TRIG_PIN, OUTPUT);
  //motors.flipRightMotor(true);
  buzzer.play("c32");   
  //getStartingReading();
  motors.setSpeeds(100, 100);
}


void loop() {
  Serial.println("  -- LOOP ---------------------------------");
  
  delay(100);
  
  moveHead(140); // Left pos
  delay(100);
  int left_distance = ultrasonicRead();
  
  delay(100);
  
  
  moveHead(160); // Forward Left pos
  delay(100);
  int forward_left_distance = ultrasonicRead();
  
  pid = computePID(left_distance, forward_left_distance); // Compute and Set Motors
  pid_storage[pid_storage_pointer] = pid; // Store pid, will be used for checks on integral windup
  
  setMotors(pid); // set motors with pid
  if (pid_storage_pointer == 3) {pid_storage_pointer = 0;}
  delay(100);
}


void moveHead(int angle) {
  headServo.write(angle);
}


float computePID(int l_distance, int f_l_distance) {
  pidMillisCurrent = millis();
  distance = (l_distance + f_l_distance) / 2;
  float err = distance - STOP_DISTANCE; // Could be either positive or negative
  float p = computeProportional(err); // Probably want this value to be around 20 when significant turn required, letting integral and derivative handle the rest. Pid above 30 is constrained eventually 
  float i = computeIntegral(err);
  float d = computeDerivative(err);
  pid = p + i + d; 

  // Sensor is experiencing an error
  if (pid > 250) {
    buzzer.play("c32");
    buzzer.play("c32");
    buzzer.play("c32");
    invalid_pid = true; // Going to ignore this PID once it gets down to setMotors
  }

  if(pid > PID_MAX) { pid = PID_MAX; }
  if(pid < PID_MIN) { pid = PID_MIN; }

  Serial.print(" | Err: "); Serial.println(err);
  Serial.print(" | P: "); Serial.print(p); Serial.print(", I: "); Serial.print(i); Serial.print(", D: "); Serial.print(d); 
  Serial.print(" => "); Serial.println(pid);

  return pid;
}




float computeProportional(float err) {
  float multiplier = 1;
  
  // When distance goes wide when looking forward-left, we have hone in runaway of the proportional
  if (( Kp * err) > 10.0) {
    multiplier = 0.65; 
  } else if (( Kp * err) > 20.0) {
    multiplier = 0.25;
  }

  float prop = Kp * err * multiplier; 
  return prop;

}


float computeIntegral(float err) {
  totalErr += err; //* (pidMillisCurrent - pidMillisPrev);
  float integral = Ki * totalErr;
  if ( integral >= INTEGRAL_MAX ) { Serial.print("Total Err"); Serial.println(totalErr); integral = INTEGRAL_MIN; }
  //if ( integral < INTEGRAL_MIN ) { integral = INTEGRAL_MIN; }
  return integral;   
}


float computeDerivative(float err) {
  float r = (err - prevErr) / (pidMillisCurrent - pidMillisPrev); //rate
  float derivative = Kd * r;
  prevErr = err;
  return derivative;
}



// motors
void setMotors(float pid_value) {
  float speedAdjustment = constrain(pid_value, -30, 45);

                       // where constrain limits
                       // number to range between x&y

  

  if (!invalid_pid) {
    Serial.print(" | constrained speedAdjustment: " ); Serial.println(speedAdjustment);
     float leftSpeed = LEFT_MOTOR_BASE_SPEED - speedAdjustment ;
     float rightSpeed = RIGHT_MOTOR_BASE_SPEED + speedAdjustment ;
     Serial.print(" | Left Speed: "); Serial.print(leftSpeed);
     Serial.print(", Right Speed: "); Serial.println(rightSpeed);
     motors.setSpeeds(-rightSpeed, -leftSpeed); // reversed l and r, negated both
   
  } else {
    Serial.print(" | Mainting Current Course" ); Serial.println("");
    motors.setSpeeds(-RIGHT_MOTOR_BASE_SPEED, -LEFT_MOTOR_BASE_SPEED); // Go straight until issue (hopefully) resolves itself
    invalid_pid = false;
  }

  float first_pid_value = pid_storage[0];
  float second_pid_value = pid_storage[1];
  float third_pid_value = pid_storage[2];

  bool all_same_sign = true; 
  bool first_pid_is_negative = (first_pid_value * -1 > 0.0); // will be true if negative
  Serial.println(""); Serial.print("First PID Value: "); Serial.print(first_pid_value); Serial.print(", Second PID Value: "); Serial.print(second_pid_value);  Serial.print(", Third PID Value: "); Serial.print(third_pid_value); Serial.print(" | First PID Negative? "); Serial.println(first_pid_is_negative);
  Serial.println("For-Loop");
  for (int i = 0; i <= 2; i++) {
    
    float pid_val = pid_storage[i];
    bool current_negative = (pid_val < 0.0);

    //Serial.print("(i "); Serial.print(i); Serial.print(") PID Storage Value: "); Serial.print(pid_val); Serial.print(" | Current Negative: "); Serial.println(current_negative);
    //Serial.print("Evaluating Bool Expression... "); Serial.print(", First Pid Is Negative: "); Serial.print(first_pid_is_negative); Serial.print(", Current Negative: "); Serial.println(current_negative);
    
    // Same sign
    if (first_pid_is_negative == current_negative) {
      //Serial.print("Equal"); Serial.println("");
      //break;
    } else {
      // Not same sign 
      all_same_sign = false;
    }
  }

  if (all_same_sign) {
    totalErr = 0.0;
    Serial.print("Reset Total Error");
    //buzzer.play(">g32>>c32");
  }

  pid_storage_pointer = pid_storage_pointer + 1; // increment pid storage pointer

}

/**
 boolean are_either_both_negative_or_positive = first_pid_is_negative && (pid_val < 0.0); // true if first is pos and pid_val is pos, true if first is neg and pid_val is neg
    Serial.print("(i "); Serial.print(i); Serial.print(") PID Storage Value: "); Serial.print(pid_val); Serial.print(" | Either Both Negative or Pos: "); Serial.print(first_pid_is_negative);
    if ( (are_either_both_negative_or_positive) && (i == 2)) {
      totalErr = 0.0; // Essentially resets integral, because integral builds off of an accumulated total error value
    } else break; // Have not found three consecutive either positive or negatives
  }
  pid_storage_pointer = pid_storage_pointer + 1; // increment pid storage pointer
**/

// ultrasonic sensor, returns values in cm
int ultrasonicRead() {
 
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
   //if(distance > MAX_DISTANCE) distance = MAX_DISTANCE;
   //if(distance == 0) distance = MAX_DISTANCE;

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
