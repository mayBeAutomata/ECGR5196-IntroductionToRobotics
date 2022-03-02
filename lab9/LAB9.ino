/***********************************************************************
   FILE NAME: LAB9

   This code reads the three line following sensors on A3, A6, and A7
   and controls the motor drive direction. If bumper is activated it reverses
   in a straight line 50cm.

   Written by Somto Anyaegbu & Ajay Chundi 05/01/2021

 ***********************************************************************/

#include <RedBot.h>           //redbot library

RedBotMotors motors;          //bind 'motors' variable

RedBotBumper lBumper = RedBotBumper(3);  // initialzes bumper object on pin 3
RedBotBumper rBumper = RedBotBumper(11); // initialzes bumper object on pin 11

RedBotSensor left = RedBotSensor(A3);   // initialize a left sensor object on A3
RedBotSensor center = RedBotSensor(A6); // initialize a center sensor object on A6
RedBotSensor right = RedBotSensor(A7);  // initialize a right sensor object on A7

RedBotEncoder encoder = RedBotEncoder(A2, 10);  // define encoder pin

//const int buttonPin = 12; // variable to store the button Pin
const int buzzerPin = 9;  //variable to store buzzer pin


int lBumperState;  // state variable to store the bumper value
int rBumperState;  // state variable to store the bumper value
int countsPerRev = 192;   // 4 pairs of N-S x 48:1 gearbox = 192 ticks per wheel rev

float wheelDiam = 6.5; //6.5cm = 65mm / 10
float wheelCirc = PI * wheelDiam; // Redbot wheel circumference = pi*D

// constants that are used in the code. LINETHRESHOLD is the level to detect
// if the sensor is on the line or not. If the sensor value is greater than this
// the sensor is above a DARK line.
//
// SPEED sets the nominal speed

#define LINETHRESHOLD 800
#define SPEED 70  // sets the nominal speed. Set to any number from 0 - 255.

int leftSpeed;   // variable used to store the leftMotor speed
int rightSpeed;  // variable used to store the rightMotor speed

void setup()
{
  Serial.begin(9600);


  //pinMode(buttonPin, INPUT_PULLUP);
  pinMode(buzzerPin, OUTPUT);  // configures the buzzerPin as an OUTPUT

  Serial.println("Welcome to Lab 9 Test");
  Serial.println("------------------------------------------");
  delay(500);
}

void loop() {

  static bool done = false; //boolean variable to determine when motors reverses

  if (done == true) {

    Serial.println("bumber active");
    motors.stop();
    tone(buzzerPin, 2000);  // Play a 2kHz tone on the buzzer pin
    delay(500);    //delay 1 seconds
    //drive_backwards(20, -SPEED);    //call drive backwards function
    drive_backwards(50, -SPEED);    //call drive backwards function
    delay(500);   // delay for 500 ms
    noTone(buzzerPin);       // Stop playing the tone.
    Serial.println("bumper deactivated");
    motors.stop();
    done = false;

  }


  lBumperState = lBumper.read();  // default INPUT state is HIGH, it is LOW when bumped
  rBumperState = rBumper.read();  // default INPUT state is HIGH, it is LOW when bumped


  // if on the line drive left and right at the same speed (left is CCW / right is CW)
  if (center.read() > LINETHRESHOLD)
  {
    leftSpeed = -SPEED;
    rightSpeed = SPEED;
  }

  // if the line is under the right sensor, adjust relative speeds to turn to the right
  else if (right.read() > LINETHRESHOLD)
  {
    leftSpeed = -(SPEED + 50);
    rightSpeed = SPEED - 50;
  }

  // if the line is under the left sensor, adjust relative speeds to turn to the left
  else if (left.read() > LINETHRESHOLD)
  {
    leftSpeed = -(SPEED - 50);
    rightSpeed = SPEED + 50;
  }

  // if all sensors are on black or up in the air, stop the motors.
  // otherwise, run motors given the control speeds above.
  if ((left.read() > LINETHRESHOLD) && (center.read() > LINETHRESHOLD) && (right.read() > LINETHRESHOLD) )
  {

    motors.stop();

  } else if ((lBumperState == LOW) || (rBumperState == LOW)) {

    done = true;
    return;

  } else  {
    motors.leftMotor(leftSpeed);
    motors.rightMotor(rightSpeed);

  }
  delay(0);  // add a delay to decrease sensitivity.

}



void drive_backwards(float distance, int motorPower) {

  //int motorPower = -SPEED;
  //uint8_t distance = 50;
  long lCount = 0;
  long rCount = 0;
  long targetCount;
  float numRev;

  // variables for tracking the left and right encoder counts
  long prevlCount, prevrCount;

  long lDiff, rDiff;  // diff between current encoder count and previous count

  // variables for setting left and right motor power
  int leftPower = motorPower;
  int rightPower = motorPower;

  // variable used to offset motor power on right vs left to keep straight.
  int offset = 5;  // offset amount to compensate Right vs. Left drive

  numRev = distance / wheelCirc;  // calculate the target # of rotations
  targetCount = numRev * countsPerRev;    // calculate the target count

  // debug
  Serial.print("driveStraight() ");
  Serial.print(distance);
  Serial.print(" inches at ");
  Serial.print(motorPower);
  Serial.println(" power.");

  Serial.print("Target: ");
  Serial.print(numRev, 3);
  Serial.println(" revolutions.");
  Serial.println();

  // print out header
  Serial.print("Left\t");   // "Left" and tab
  Serial.print("Right\t");  // "Right" and tab
  Serial.println("Target count");
  Serial.println("============================");

  encoder.clearEnc(BOTH);    // clear the encoder count
  delay(100);  // short delay before starting the motors.

  motors.drive(motorPower);  // start motors

  while ((-1 * rCount) < targetCount)
  {
    // while the right encoder is less than the target count -- debug print
    // the encoder values and wait -- this is a holding loop.
    lCount = encoder.getTicks(LEFT);
    rCount = encoder.getTicks(RIGHT);
    Serial.print(lCount);
    Serial.print("\t");
    Serial.print(rCount);
    Serial.print("\t");
    Serial.println(targetCount);

    motors.leftDrive(leftPower);
    motors.rightDrive(rightPower);

    // calculate the rotation "speed" as a difference in the count from previous cycle.
    lDiff = (lCount - prevlCount);
    rDiff = (rCount - prevrCount);

    // store the current count as the "previous" count for the next cycle.
    prevlCount = lCount;
    prevrCount = rCount;

    // if left is faster than the right, slow down the left / speed up right
    if (lDiff > rDiff)
    {
      leftPower = leftPower - offset;
      rightPower = rightPower + offset;
    }
    // if right is faster than the left, speed up the left / slow down right
    else if (lDiff < rDiff)
    {
      leftPower = leftPower + offset;
      rightPower = rightPower - offset;
    }
    delay(50);  // short delay to give motors a chance to respond.
  }
  // now apply "brakes" to stop the motors.
  motors.brake();
}
