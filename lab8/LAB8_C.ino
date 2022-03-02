/*==========================================================================================
   File: Localisation
   04/26/2021 - Written by Somto Anyaegbu
      (some of this original code by Franklin S. Cooper Jr. & A. Gupta)

   Summary:
   This Example will demostrate localisation using various features of the Encoder and Servo.H Library.
   The robot will spin and take 'LOCALIZE_SAMPLES' number of measurements per '360 / LOCALIZE_SAMPLES' angle.
   The robot will proceed to go forward towards the angle that is more or equal to Maximum distance sensor
   can measure then drive a distance of 'distY' and check distance at angles East and West then drive to angle
   that is more or equal to Maximum distance sensor can measure for 'distZ'.

   --Press Left Button For Task C
   ===========================================================================================
*/


#include <Servo.h>
#include "SimpleRSLK.h"

Servo myservo;  // create servo object to control a servo
// a maximum of eight servo objects can be created

// Various constants defining the physical characteristics
// of the vehicle
#define WHEEL_DIAMETER 7.0 // in centimeters
#define WHEEL_BASE 14.3 // in centimeters
#define CNT_PER_REV 360.0 // Number of encoder (rising) pulses every time the wheel turns completely

// default, max, and min speed of the wheels
// the correction speed is used if the encoder
// values are out of sync and a wheel needs to sped up
#define WHEEL_SPEED 10.0
#define MAX_SPEED 17.0
#define SONARSPEED 13
#define MIN_SPEED 8.0
#define CORRECTION_SPEED_DELTA 5.0

#define NORTH 95      //Servo Position North
#define EAST 0        //Servo Position East
#define WEST 180      //Servo Position West
#define DEFAULT_POS 90      //Servo Position Default

#define MAX_SONAR_DIST 350        //Maximum distance sensor can measure
#define SONAR_SAMPLES 5           //Length of UltraSound Ping Samples Array For Sort Algorithm
#define LOCALIZE_SAMPLES 4        //Number of Localize Samples for A, B & C. Just change variables

#define xDist  89    //185             //distance Z from notes
#define yDist  94    //119              //distance Y from notes
#define RdiffSonarRbt 6       //distance between position of HC-SR04 and right side of Robot

#define PID_Kp  1.144 //1.144      //Kp Proportional Gain
#define PID_Ki  0 //0.005      //Ki Integral Gain
#define PID_Kd  0              //Kd Derivative Gain
#define PID_CONT_MAX  25       //Maximum PID Output
#define PID_CONT_MIN  -25      //Minimum PID Output

#define LOWER_BIND(X, Y) if ((X) < (Y)) { (X) = (Y); }
#define UPPER_BIND(X, Y) if ((X) > (Y)) { (X) = (Y); }


const int trigPin = 32;       //This is Port Pin 3.5 on the MSP432 Launchpad
const int echoPin = 33;       //This is Port Pin 5.1 on the MSP432 Launchpad



/// This is a PID control function that calculates the compensation inorder to control the position of the robot
///
///@param error is the difference between the initial measured distance to wall and current position of robot.
///@param PID_Output is the speed compensation for the robot
int16_t pidControl(float current_pos, float PID_target) {

  static int last_error = 0;
  static int integral = 0;
  static int derivative = 0;

  if (current_pos == -1 and PID_target == -1) {
    integral = 0;
    last_error = 0;
  }

  float error = PID_target - current_pos;

  integral = integral + error;
  derivative = error - last_error;
  last_error = error;

  float PID_Output = PID_Kp * error + PID_Ki * integral + PID_Kd * derivative;

  UPPER_BIND(PID_Output, PID_CONT_MAX);
  LOWER_BIND(PID_Output, PID_CONT_MIN);

  return PID_Output;

}



/// Converts the an angle of rotation to the amount the
/// wheels will travel. Assumes vehicle will use both wheels
/// to rotate
///
/// @param degrees The degrees of rotation to convert
/// @param WHEEL_BASE The distance between the 2 wheels
///        Should be renamed to WHEEL_TRACK
/// @return The distance one wheel travels in meters
float calc_degrees_to_distance_turn (float degrees) {
  float wheel_base_circum = WHEEL_BASE * PI;
  float angle_ratio = degrees / 360.0;
  return angle_ratio * wheel_base_circum;
}



/// Converts wheel travel distance to an encoder count
/// Useful for calculateing how many encoder counts needed
/// to travel a distance
///
/// @param distance The distance to convert
/// @param WHEEL_DIAM The diameter of the wheel
/// @param CNT_PER_REV The number of encoder counts per
///        one revolution of the wheel
/// @return The number of encoder counts it should take
///         to travel \param distance distance
uint16_t calc_encoder_count (float distance) {
  float wheel_circum = WHEEL_DIAMETER * PI;
  float wheel_rotations = distance / wheel_circum;
  return uint16_t( wheel_rotations * CNT_PER_REV );
}



/// Keeps the motor in sync using the encoders until both
/// encoders have reached \param cnt count. Also will
/// run a speed control function to adjust the speed as it
/// travels
///
/// @todo Make speed control function optional
///
/// @param cnt The number of encoder counts to keep sync until
/// @param speed_ctrl_func A functional pointer used to determine the
///        base speed of the motors, used to transform the speed thoughout
///        the journey of the vehicle
/// @param WHEEL_SPEED The default wheel speed when no transform is used
void sync_motors_until_cnt(uint16_t cnt) {
  uint16_t leftCount = getEncoderLeftCnt();
  uint16_t rightCount = getEncoderRightCnt();
  uint8_t navSpeed = WHEEL_SPEED;
  uint8_t leftMotorSpeed = navSpeed;
  uint8_t rightMotorSpeed = navSpeed;

  while (leftCount < cnt and rightCount < cnt) {
    uint16_t avgCount = (leftCount + rightCount) / 2;
    float dist_covered = float(avgCount) / cnt;


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



/// Turns the vehicle using both wheels \param degrees number
/// of degrees.
///
/// @param degrees The number of degrees to spin the vehicle, positive
///        numbers mean CCW, negative numbers mean CW
void turn(float degrees) {
  float distance_to_turn = calc_degrees_to_distance_turn(abs(degrees));
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

  sync_motors_until_cnt(encoder_cnt_needed);

  disableMotor(BOTH_MOTORS);
}



///Function that drives motor forward
///@param centimeters The number of meters to travel straight. Use
///        a negative value to travel backwards
void drive (float centimeters) {
  
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

  sync_motors_until_cnt(encoder_cnt_needed);

  disableMotor(BOTH_MOTORS);

}



///Function that drives motor forward
///@param centimeters The number of meters to travel straight. Use
///        a negative value to travel backwards
///
///@param hwW is the width of the hallway
void drive_sonar(float centimeters) {

  servo_control(EAST);
  float navSonar = 0;
  float eastDist = centimeters;
  uint8_t navSpeed = SONARSPEED;
  uint8_t leftMotorSpeed = navSpeed;
  uint8_t rightMotorSpeed = navSpeed;

  resetLeftEncoderCnt();
  resetRightEncoderCnt();

  if (centimeters > 0) {
    setMotorDirection(BOTH_MOTORS, MOTOR_DIR_FORWARD);
  } else {
    setMotorDirection(BOTH_MOTORS, MOTOR_DIR_BACKWARD);
  }

  enableMotor(BOTH_MOTORS);
  setMotorSpeed(BOTH_MOTORS, SONARSPEED);

  while (navSonar < (2.5 * eastDist)) {

    for (int x = 0; x < 6; x++) {
      if (isBumpSwitchPressed(x) == true) { 
        disableMotor(BOTH_MOTORS);                          // Halt motors
        delay(3000);
      }
    }

    navSonar = fast_sonar_ping();

    //delay(5);                       //to compensate for the sonar sensor conical shape ping

    uint16_t deltaSpeed = pidControl(navSonar, eastDist);

    leftMotorSpeed = navSpeed - deltaSpeed;
    rightMotorSpeed = navSpeed + deltaSpeed;

    if (leftMotorSpeed < 0) {
      rightMotorSpeed += -leftMotorSpeed;
      leftMotorSpeed = 0;

    }

    if (rightMotorSpeed < 0) {
      leftMotorSpeed += -rightMotorSpeed;
      rightMotorSpeed = 0;

    }

    setMotorSpeed(LEFT_MOTOR, leftMotorSpeed);
    setMotorSpeed(RIGHT_MOTOR, rightMotorSpeed);

  }

  disableMotor(BOTH_MOTORS);
}



///This function controls servo position.
///
///@param def_pos is the default position of the servo mpotor
///@param new_dir is the degree to turn to passed into the funtion
void servo_control(int new_dir) {
  static int def_pos = 90;

  int direction = (new_dir < def_pos) ? -1 : 1;

  for (; def_pos != new_dir; def_pos += direction) {
    myservo.write(def_pos);                   // tell servo to go to position in variable 'pos'
    delay(15);                                // waits 15ms for the servo to reach the position
  }

}



///This function returns an accurate sonar ping reading by implementing a sort algorithm.
///
///@param pulseLength[x] array used to store pulseIn readings. It's an array of 'x' values(5)
///@param temp is used to store pulse readings temporarily during sorting
///@params i and j are used for identifying pulseLength during sort
float accurate_sonar_ping() {

  float pulseLength[SONAR_SAMPLES], tmp;          //declare variables

  /* Sort five readings */
  for (int i = 0; i < SONAR_SAMPLES; i++) {               //Loop for ascending ordering

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



///Returns a sonar ping used during navigation. Doesn't include a sort algorithm
///It isn't most accurate but precise enough for navigation.
float fast_sonar_ping() {

  digitalWrite(trigPin, LOW);               // send low to get a clean pulse
  delayMicroseconds(2);                     // let it settle
  digitalWrite(trigPin, HIGH);              // send high to trigger device
  delayMicroseconds(10);                    // let it settle
  digitalWrite(trigPin, LOW);               // send low to get a clean pulse

  float fast_sonar = pulseIn(echoPin, HIGH) / 58;
  if (fast_sonar <= 0) {
    fast_sonar = 0 ;
  }
  if (fast_sonar >= 400) {
    fast_sonar = 400;       //4m is the maximum distance sonar can measure
  }
  return fast_sonar;       // Return distance from right side of robot

}




////////////////////////////////////////////////////////////////////////////////
///Put code setup here.
///
///////////////////////////////////////////////////////////////////////////////
void setup() {
  // put your setup code here, to run once:

  pinMode(76, OUTPUT);    //RGB LED - GREEN_LED
  pinMode(77, OUTPUT);    //RGB LED - BLUE_LED
  pinMode(trigPin, OUTPUT);  //Trig Signal Pin Set To Output
  pinMode(echoPin, INPUT);   //Echo Signal Pin Set To Output

  myservo.attach(38);       //Attaches the servo on Port 2.4 to the servo object
  myservo.write(DEFAULT_POS);  // Set to the default position
  Serial.begin(115200);       //Set baudrate and initialize Rx & Tx

  setupRSLK();              // Set up all of the pins & functions needed to
  //   be used by the TI bot
  setupWaitBtn(LP_LEFT_BTN);   // Left button on Launchpad
  setupLed(RED_LED);           // Red LED of the RGB LED

  String btnMsg = "Push left button on Launchpad to start ABC Demo.\n";
  waitBtnPressed(LP_LEFT_BTN, btnMsg, RED_LED);

  delay(2000);

}

void loop() {
  // put your main code here, to run repeatedly:

  float dist_2_wall[LOCALIZE_SAMPLES];        //distance to each side of the wall
  int maxDist = 0;

  servo_control(NORTH);                       //servo turn north

  for (int i = 0; i < LOCALIZE_SAMPLES; i++ ) {

    dist_2_wall[i] = accurate_sonar_ping();       //populate localization array with ditance to
    turn(360 / LOCALIZE_SAMPLES);                 //wall measurement by calling median sonar algorithm
    delay(500);

    for (int j = 0; j < LOCALIZE_SAMPLES; j++ ) {

      if (dist_2_wall[j] > dist_2_wall[maxDist]) {    //sort localization array and find largest distance
        maxDist = j;

      }
    }
  }

  if (dist_2_wall[maxDist] > MAX_SONAR_DIST) {

    dist_2_wall[maxDist] == MAX_SONAR_DIST;               //set to longest distance to maximum dist measurable by sonar

  }

  turn((CNT_PER_REV / LOCALIZE_SAMPLES) * maxDist);         //turn robot to angle with largest distance
  delay(1000);

  servo_control(EAST);                      //Turn servo right
  delay(1000);

  float eDist = fast_sonar_ping();             //store distance east of robot in float eDist variable
  delay(250);

  drive_sonar (eDist);              //call drive function and pass east distance and width of hallway to the drive function

  uint16_t y = yDist + RdiffSonarRbt;       //store "distance Y" plus distance between position of HC-SR04 and right side of Robot in 'y' variable
  uint16_t x = xDist + eDist;      //store "distance Z" plus measured distance on right side of bot in 'x' variable
  Serial.println("Sonar done");

  drive(y);                              //drive y distance offset
  delay(1000);

  float check1 = fast_sonar_ping();
  if (check1 > (eDist * 2) ) {             //decide where to turn

    turn((-1) * (CNT_PER_REV / LOCALIZE_SAMPLES));
    drive(x);

  } else {

    turn(CNT_PER_REV / LOCALIZE_SAMPLES);       //Turn right
    drive(x);                                   //drive remainder of distance

  }
}
