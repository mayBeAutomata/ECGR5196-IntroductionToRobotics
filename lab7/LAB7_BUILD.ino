//*******************************************************************
//  Modified by Somto Anyaegbu & Ajay Sankar Chundi 04/20/2021
//*******************************************************************


#include <Servo.h>
#include "SimpleRSLK.h"

Servo myservo;  // create servo object to control a servo
// a maximum of eight servo objects can be created

// Various constants defining the physical characteristics
// of the vehicle
#define WHEEL_DIAMETER 7.0 // in centimeters
#define WHEEL_BASE 14.3 // in centimeters
#define CNT_PER_REV 360 // Number of encoder (rising) pulses every time the wheel turns completely
#define RBT_POS_SETPOINT 16 //Set distance of robot left side from wall

// default, max, and min speed of the wheels
// the correction speed is used if the encoder
// values are out of sync and a wheel needs to sped up
#define WHEEL_SPEED 15
#define MAX_SPEED 20
#define MIN_SPEED 8
#define TURN_SPEED 10
#define CORRECTION_SPEED_DELTA 4

// Percent of journey to stop acceleration
// and start deceleration
#define SPEED_RAMP_ACCEL 0.10 // 0% to 10% accelerate
#define SPEED_RAMP_DECEL 0.40 // 40% to 100% decelerate
#define SPEED_STEP_ACCEL 0.25
#define SPEED_STEP_DECEL 0.75

const int trigPin = 32;       //This is Port Pin 3.5 on the MSP432 Launchpad
const int echoPin = 33;       //This is Port Pin 5.1 on the MSP432 Launchpad
const int x = 5;              //Sonar Ping Sort Array Length


////////////////////////////////////////////////////////////////////////////////
/// Setup code here
///////////////////////////////////////////////////////////////////////////////
void setup() {  // put your setup code here, to run once:

  pinMode(76, OUTPUT);    //RGB LED - GREEN_LED
  pinMode(77, OUTPUT);    //RGB LED - BLUE_LED
  pinMode(trigPin, OUTPUT);  //Trig Signal Pin Set To Output
  pinMode(echoPin, INPUT);   //Echo Signal Pin Set To Output

  myservo.attach(38);       //Attaches the servo on Port 2.4 to the servo object
  myservo.write(90);  // Set to the default position
  Serial.begin(115200);       //Set baudrate and initialize Rx & Tx

  setupRSLK();              // Set up all of the pins & functions needed to
  //   be used by the TI bot
  setupWaitBtn(LP_LEFT_BTN);   // Left button on Launchpad
  setupLed(RED_LED);           // Red LED of the RGB LED

  String btnMsg = "Push left button on Launchpad to start demo.\n";
  waitBtnPressed(LP_LEFT_BTN, btnMsg, RED_LED);
  delay(2000);

}

////////////////////////////////////////////////////////////////////////////////////////////////////
/// Main code here
///
///
/// @param dist_2_wall Distance of sonar to wall
/// @param dist_2_trav Distance for robot to travel.
///         The difference of dist_2_wall and @constant 14.
///         14cm accounts for safe margin(10cm) from wall and distance of sonar to front of bot(4cm).
/// @param WHEEL_SPEED The default wheel speed when no transform is used
/// @params check and check2 are used to decide when and where robot should turn.
///////////////////////////////////////////////////////////////////////////////////////////////////
void loop() {
  // put your main code here, to run repeatedly:

  float dist_2_wall = 0;
  servo_control(95);
  Serial.println("Servo Turn North");
  delay(500);
  dist_2_wall = accurate_sonar();
  Serial.println("Distance To Wall");
  float dist_2_trav = dist_2_wall - 14;
  Serial.println("Dist To Trav");
  drive(dist_2_trav);
  Serial.println("Done Drive");

  servo_control(95);
  float check = accurate_sonar();
  Serial.println("checking");
  delay(50);

  if (check >= 14 and check <= 17) {
    Serial.println("check done");
    delay(50);

    servo_control(180);
    Serial.println("servo to 0d");
    delay(50);
    float check2 = fast_sonar_ping();
    Serial.println("check2 done");
    delay(50);

    if (check2 < 20) {

      turn(-90);
      Serial.println("turn right");
      delay(500);
    } else {
      turn(90);
      Serial.println("turn left");
      delay(500);
    }
  }
  delay (500);
}

////////////////////////////////////////////////////////////////////////////////
///Returns a sonar ping used during navigation. Doesn't include a sort algorithm
///It isn't most accurate but precise enough for navigation.
///////////////////////////////////////////////////////////////////////////////

float fast_sonar_ping() {

  digitalWrite(trigPin, LOW);               // send low to get a clean pulse
  delayMicroseconds(2);                     // let it settle
  digitalWrite(trigPin, HIGH);              // send high to trigger device
  delayMicroseconds(10);                    // let it settle
  digitalWrite(trigPin, LOW);               // send low to get a clean pulse
  return pulseIn(echoPin, HIGH) / 58;       // Return distance from right side of robot

}

//////////////////////////////////////////////////////////////////////////////////////
///This function returns an accurate sonar ping reading by implementing a sort algorithm.
///
///@param pulseLength[x] array used to store pulseIn readings. It's an array of 'x' values(5)
///@param temp is used to store pulse readings temporarily during sorting
///@params i and j are used for identifying pulseLength during sort
float accurate_sonar() {

  float pulseLength[x], tmp;          //declare variables

  /* Sort five readings */
  for (int i = 0; i < 5; i++) {               //Loop for ascending ordering

    digitalWrite(trigPin, LOW);               // send low to get a clean pulse
    delayMicroseconds(2);                     // let it settle
    digitalWrite(trigPin, HIGH);              // send high to trigger device
    delayMicroseconds(10);                    // let it settle
    digitalWrite(trigPin, LOW);               // send low to get a clean pulse

    pulseLength[i] = pulseIn(echoPin, HIGH) / 58;  // measure pulse coming back, convert pulse to cm
    delay(1000);                              //delay for 1000ms

    for (int j = 0; j < 5; j++) {         //Loop for comparing other values
      if (pulseLength[j] > pulseLength[i]) { //Comparing other array elements
        tmp = pulseLength[i];             //Using temp var for storing last value
        pulseLength[i] = pulseLength[j];  //replacing value
        pulseLength[j] = tmp;             //storing last value
      }
    }
  }

  return pulseLength[2];                       //Return median value

}


void servo_control(int new_dir) {
  static int def_pos = 90;

  int direction = (new_dir < def_pos) ? -1 : 1;

  for (; def_pos != new_dir; def_pos += direction) {
    myservo.write(def_pos);                   // tell servo to go to position in variable 'pos'
    delay(15);                                // waits 15ms for the servo to reach the position
  }

}


/// Keeps the motor straight using the fast sonar ping until encoder count is reached
/// @param cnt The number of encoder counts to keep sync until
/// @param isBumpSwitchPressed(x) stores the bool condition of the bump switches
/// 
/// @param WHEEL_SPEED The default wheel speed when no transform is used
void sync_motors_until_cnt(uint16_t cnt, uint8_t (*speed_ctrl_func)(float)) {
  uint16_t leftCount = getEncoderLeftCnt();
  uint16_t rightCount = getEncoderRightCnt();
  int delta_pos = 0;
  
  while (leftCount < cnt and rightCount < cnt) {
    uint16_t avgCount = (leftCount + rightCount) / 2;
    float dist_covered = float(avgCount) / cnt;

    for (int x = 0; x < 6; x++) {
      if (isBumpSwitchPressed(x) == true) {
        // === DRIVE Straight ==========================
        disableMotor(BOTH_MOTORS);
        delay(1000);
        setMotorDirection(BOTH_MOTORS, MOTOR_DIR_BACKWARD); // Cause the robot to drive forward
        enableMotor(BOTH_MOTORS);                           // "Turn on" the motor
        setMotorSpeed(BOTH_MOTORS, WHEEL_SPEED);            // Set motor speed
        delay(250); 
        disableMotor(BOTH_MOTORS);                          // Halt motors
        delay(500);

        // === Turn 90degrees CCW ==========================
        setMotorDirection(RIGHT_MOTOR, MOTOR_DIR_FORWARD);  // Cause the robot to drive 90 degrees CCW
        setMotorDirection(LEFT_MOTOR, MOTOR_DIR_BACKWARD);  //   by setting the L motor and R motor CW
        enableMotor(BOTH_MOTORS);                           // "Turn on" the motor
        setMotorSpeed(BOTH_MOTORS, WHEEL_SPEED);            // Set motor speed
        delay(750);
        disableMotor(BOTH_MOTORS);                          // Halt motors
        delay(1000);
        setMotorDirection(BOTH_MOTORS, MOTOR_DIR_FORWARD);  // Cause the robot to drive forward
        enableMotor(BOTH_MOTORS);                           // "Turn on" the motor
      }
    }

    leftCount = getEncoderLeftCnt();
    rightCount = getEncoderRightCnt();

    delta_pos = fast_sonar_ping();

    if (delta_pos < 16) {       //Turn Left

      setMotorSpeed(LEFT_MOTOR, WHEEL_SPEED);
      setMotorSpeed(RIGHT_MOTOR, MAX_SPEED);

    } else if (delta_pos > 16 && delta_pos < 50) {    //Turn right

      setMotorSpeed(LEFT_MOTOR, MAX_SPEED);
      setMotorSpeed(RIGHT_MOTOR, WHEEL_SPEED);

    } else if (delta_pos > 50) {    //Sharp right turn

      setMotorSpeed(LEFT_MOTOR, MAX_SPEED);
      setMotorSpeed(RIGHT_MOTOR, MIN_SPEED);

    } else {        //Keep straight

      setMotorSpeed(LEFT_MOTOR, WHEEL_SPEED);
      setMotorSpeed(RIGHT_MOTOR, WHEEL_SPEED);

    }

  }
}

void sync_motors_until_turn(uint16_t cnt) {
  uint16_t leftCount = getEncoderLeftCnt();
  uint16_t rightCount = getEncoderRightCnt();
  uint8_t navSpeed = TURN_SPEED;
  uint8_t leftMotorSpeed = navSpeed;
  uint8_t rightMotorSpeed = navSpeed;

  while (leftCount < cnt and rightCount < cnt) {
    uint16_t avgCount = (leftCount + rightCount) / 2;
    float dist_covered = float(avgCount) / cnt;

    // first get base speed using the transform function
    //navSpeed = speed_ctrl_func(dist_covered);

    leftCount = getEncoderLeftCnt();
    rightCount = getEncoderRightCnt();

    // then adjust the left and right wheel speeds to sync up
    // the wheel. only gets synced up if the encoder counts there
    // is more than 10 counts of difference otherwise reset the
    // wheel speeds back to default
    int abs_diff = abs(int(leftCount) - rightCount);
    if (abs_diff > 10) {

      if (leftCount < rightCount) { // left wheel lagging
        leftMotorSpeed = navSpeed + CORRECTION_SPEED_DELTA;
        rightMotorSpeed = navSpeed - CORRECTION_SPEED_DELTA;

      }

      if (rightCount < leftCount) { // right wheel lagging
        leftMotorSpeed = navSpeed - CORRECTION_SPEED_DELTA;
        rightMotorSpeed = navSpeed + CORRECTION_SPEED_DELTA;

      }

    } else {
      leftMotorSpeed = navSpeed;
      rightMotorSpeed = navSpeed;
    }
    setMotorSpeed(LEFT_MOTOR, leftMotorSpeed);
    setMotorSpeed(RIGHT_MOTOR, rightMotorSpeed);
  }
}

///Function that drives motor forward
///@param centimeters The number of meters to travel straight. Use
///        a negative value to travel backwards
void drive(float centimeters) {
  servo_control(0);

  uint16_t encoder_cnt_needed = calc_encoder_count(abs(centimeters));

  resetLeftEncoderCnt();
  resetRightEncoderCnt();

  if (centimeters > 0) {
    setMotorDirection(BOTH_MOTORS, MOTOR_DIR_FORWARD);
  } else {
    setMotorDirection(BOTH_MOTORS, MOTOR_DIR_BACKWARD);
  }

  enableMotor(BOTH_MOTORS);
  setMotorSpeed(BOTH_MOTORS, WHEEL_SPEED);

  sync_motors_until_cnt(encoder_cnt_needed, calc_speed_ramp);

  disableMotor(BOTH_MOTORS);
}


/// Turns the vehicle using both wheels \param degrees number
/// of degrees.
///
/// @param degrees The number of degrees to spin the vehicle, positive
///        numbers mean CCW, negative numbers mean CW
void turn(float degrees) {
  float distance_to_turn = calc_degrees_to_distance_spin(abs(degrees));
  uint16_t encoder_cnt_needed = calc_encoder_count(distance_to_turn);

  resetLeftEncoderCnt();
  resetRightEncoderCnt();

  if (degrees > 0) {
    setMotorDirection(RIGHT_MOTOR, MOTOR_DIR_FORWARD);
    setMotorDirection(LEFT_MOTOR, MOTOR_DIR_BACKWARD);
  } else {
    setMotorDirection(RIGHT_MOTOR, MOTOR_DIR_BACKWARD);
    setMotorDirection(LEFT_MOTOR, MOTOR_DIR_FORWARD);
  }

  enableMotor(BOTH_MOTORS);
  setMotorSpeed(BOTH_MOTORS, WHEEL_SPEED);

  sync_motors_until_turn(encoder_cnt_needed);

  disableMotor(BOTH_MOTORS);
}


/// Calculates the speed (in percent) the vehicle
/// should travel at using the ramp function
///
/// @param progress The progress of the vehicle in a percentage
/// @param SPEED_RAMP_ACCEL A constant defined var containing the
///        at what percent to stop acceleration
/// @param SPEED_RAMP_DECEL A constant defined var containing the
///        at what percent to stat deceleration
/// @return The speed of the vehicle
uint8_t calc_speed_ramp(float progress) {
  Serial.println();
  Serial.print("progress: ");
  Serial.print(progress);

  if (progress < SPEED_RAMP_ACCEL) { // acceleration ramp
    float slope = float(MAX_SPEED - MIN_SPEED) / SPEED_RAMP_ACCEL;

    Serial.print("\t slope: ");
    Serial.print(slope);

    return slope * progress + MIN_SPEED;
  } else if (progress > SPEED_RAMP_DECEL) { // deceleration ramp
    progress -= (1 - SPEED_RAMP_DECEL);
    float slope = float(MAX_SPEED - MIN_SPEED) / -(SPEED_RAMP_DECEL);

    Serial.print("\t slope: ");
    Serial.print(slope);

    return slope * progress + MAX_SPEED;
  } else { // plauteu section of ramp
    Serial.print("\t slope: ");
    Serial.print("platue");

    return MAX_SPEED;
  }
}

/// Calculates the speed (in percent) the vehicle
/// should travel at using a simple step function.
/// Will travel vehicle slowly for some time, jump to
/// normal speed, then jump back to slow speed for the
/// last leg of the journey
///
/// @param progress The progress of the vehicle in a percentage
/// @param SPEED_STEP_ACCEL A constant defined var containing the
///        at what percent to resume normal speed
/// @param SPEED_STEP_DECEL A constant defined var containing the
///        at what percent to go back to slow speed
/// @return The speed of the vehicle
uint8_t calc_speed_step(float progress) {
  if (progress < SPEED_STEP_ACCEL) {
    return MIN_SPEED;
  } else if (progress > SPEED_STEP_DECEL) {
    return MIN_SPEED;
  } else {
    return WHEEL_SPEED;
  }
}


/// Converts the an angle of rotation to the amount the
/// wheels will travel. Assumes vehicle will use both wheels
/// to rotate
///
/// @param degrees The degrees of rotation to convert
/// @param WHEEL_BASE The distance between the 2 wheels
///        Should be renamed to WHEEL_TRACK
/// @return The distance one wheel travels in meters
float calc_degrees_to_distance_spin (float degrees) {
  float wheel_base_circum = WHEEL_BASE * PI;
  float angle_ratio = degrees / 360.0;
  return angle_ratio * wheel_base_circum;
}

/// Converts wheel travel distance to an encoder count
/// Useful for calculateing how many encoder counts needed
/// to travel a distance
///
/// @param distance The distance to convert
/// @param WHEEL_DIAMETER The diameter of the wheel
/// @param CNT_PER_REV The number of encoder counts per
///        one revolution of the wheel
/// @return The number of encoder counts it should take
///         to travel \param distance distance
uint16_t calc_encoder_count (float distance) {
  float temp = (WHEEL_DIAMETER * PI) / CNT_PER_REV;
  temp = distance / temp;
  return int(temp);
}
