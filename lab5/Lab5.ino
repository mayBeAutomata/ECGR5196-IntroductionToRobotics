//*******************************************************************
// ServoExample - Run an inexpensive Servo Motor
// James Conrad, 2020-06-10
//  Modified by Somto Anyaegbu & Ajay Sankar Chundi 03/19/2021
//*******************************************************************
#include <Servo.h>
Servo myservo;  // create servo object to control a servo
              // a maximum of eight servo objects can be created
 
const int trigPin = 32;              //This is Port Pin 3.5 on the MSP432 Launchpad
const int echoPin = 33;             //This is Port Pin 5.1 on the MSP432 Launchpad
 
 
void setup() {  // put your setup code here, to run once:
   pinMode(75, OUTPUT);  //RGB LED - RED_LED
   pinMode(76, OUTPUT);  //RGB LED - GREEN_LED
   pinMode(77, OUTPUT);  //RGB LED - BLUE_LED
   pinMode(trigPin, OUTPUT);  //Trig Signal Pin Set To Output
   pinMode(echoPin, INPUT);   //Echo Signal Pin Set To Output
 
   myservo.attach(38);  //Attaches the servo on Port 2.4 to the servo object
   myservo.write(0);  // Set to the default position
   Serial.begin(9600); //Set baudrate and initialize Rx & Tx
   delay(5000);
  
}
 
void loop() {               // put your main code here, to run repeatedly:
  Serial.println(" ");
  Serial.println("Start Servo-uSound Test");
  int pos = 0;              // variable to store the servo position
  digitalWrite(75, HIGH);     //turn red LED on
  delay(15);                  //waits 15ms for the servo to reach the position
  Serial.println("___________________________________________");
  Serial.println("Measuring Distance at 0 deg");
  float avgDistA = usound();  //call ultraSound function and print return value
  Serial.print("Median Distance at 0 deg = ");
  Serial.print(avgDistA);
  Serial.println(" cm");
  Serial.println("___________________________________________");
 
  digitalWrite(75, LOW);       //turn red LED off
  delay(500);
 
  digitalWrite(76, HIGH);      //turn green LED on
  for(pos = 0; pos < 90; pos += 1) { // goes from 0 degrees to 90 degrees
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(15);                   // waits 15ms for the servo to reach the position
  }

    Serial.println("___________________________________________");
    Serial.println("Measuring Distance at 90 deg");
    float avgDistB = usound(); //call ultraSound function and print return value
    Serial.print("Median Distance at 90 deg = ");
    Serial.print(avgDistB);
    Serial.println(" cm");
    Serial.println("___________________________________________");

    digitalWrite(76, LOW);          //turn green LED off
    delay(500);

    digitalWrite(77, HIGH);         //turn blue LED on
    for(pos = 90; pos < 180; pos += 1)  {   // goes from 90 degrees to 180 degrees
      myservo.write(pos);            // tell servo to go to position in variable 'pos'
      delay(15);                   // waits 15ms for the servo to reach the position
  }
 
    Serial.println("___________________________________________");
    Serial.println("Measuring Distance at 180 deg");
    float avgDistC = usound();        //call uSound function and print return value
    Serial.print("Median Distance at 180 deg = ");
    Serial.print(avgDistC);
    Serial.println(" cm");
    Serial.println("___________________________________________");
    Serial.println(" ");
    digitalWrite(77, LOW);          //turn blue LED off
    delay(500);                       // waits 500ms
 
    Serial.println("End Servo-uSound Test.");
    Serial.println("___________________________________________");

    digitalWrite(76, HIGH);           //turn green LED on
    digitalWrite(75, LOW);            //turn red LED on
    for(pos = 180; pos>=1; pos-=1)  {   // goes from 180 degrees to 0 degrees
      myservo.write(pos);         // tell servo to go to position in variable 'pos'
      delay(15);               // waits 15ms for the servo to reach the position
  }

  digitalWrite(76, LOW);        //turn green LED off
  digitalWrite(75, LOW);        //turn red LED off
  delay(500);
 
}
 
 
 
float usound(){     //perception function
 
  long Midcm;           //declare median val variable
  long centimeters;       //declare centimeter variable
 
  int pulseLength[5],x,i,j,tmp;   //declare variables
 
  /* Sort five readings */
 
  for (i=0; i<5; i++) {                   //Loop for ascending ordering
 
      digitalWrite(trigPin, LOW);               // send low to get a clean pulse
      delayMicroseconds(2);                     // let it settle
      digitalWrite(trigPin, HIGH);              // send high to trigger device
      delayMicroseconds(10);                    // let it settle

      pulseLength[x] = pulseIn(echoPin, HIGH);  // measure pulse coming back
      centimeters = pulseLength[x] / 58;        //convert pulse to centimeters
 
      Serial.print("Distance = ");              //Print distance
      Serial.print(centimeters);
      Serial.println(" cm");
      delay(1000);                              //delay for 1000ms

      for (int j = 0; j < 5; j++) {         //Loop for comparing other values
        if (pulseLength[j] > pulseLength[i]) { //Comparing other array elements
          tmp = pulseLength[i];             //Using temp var for storing last value
          pulseLength[i] = pulseLength[j];  //replacing value
          pulseLength[j] = tmp;             //storing last value
        }
      }
  }

   /* Print middle one */
   Midcm = pulseLength[2] / 58;        //convert pulselength to cm
   return Midcm;                       //Return median
 
}
