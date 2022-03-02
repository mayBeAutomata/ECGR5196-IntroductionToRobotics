/* 
HC-SR04 Ultrasonic Distance Sensor Example
Demonstrates sensing distance with the HC-SR04 using Texas Instruments
LaunchPads.
                
Created by Frank Milburn 5 Jun 2015
Released into the public domain.

Modified by James Conrad 8 Jun 2020
Modified by Somtochukwu Anyaegbu 28 March 2020

*/

const int trigPin = 32;                       //This is Port Pin 3.5 on the MSP432 Launchpad
const int echoPin = 33;                       //This is Port Pin 5.1 on the MSP432 Launchpad 

void setup() {
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);      
   
  Serial.begin(9600);
  delay(5000);
  Serial.println("Start HC-SR04 Test");  
}

void loop() {
  long Midcm;
  long centimeters;

  int pulseLength[5],x,i,j,tmp;
  
  for (x=0; x<5; x++) {                         //Loop for ascending ordering

    digitalWrite(trigPin, LOW);                 // send low to get a clean pulse
    delayMicroseconds(5);                       // let it settle
    digitalWrite(trigPin, HIGH);                // send high to trigger device
    delayMicroseconds(10);                      // let it settle

    pulseLength[x] = pulseIn(echoPin, HIGH);    // measure pulse coming back
    centimeters = pulseLength[x] / 58;

    Serial.print("Distance = ");
    Serial.print(centimeters);
    Serial.println(" centimeters");
    delay(1000);
  }

    /* Sort five readings */
  
    for (i=0; i<5; i++) {                      //Loop for ascending ordering
      for (int j = 0; j < 5; j++) {            //Loop for comparing other values
        if (pulseLength[j] > pulseLength[i]) { //Comparing other array elements
          tmp = pulseLength[i];                //Using temp var for storing last value
          pulseLength[i] = pulseLength[j];     //replacing value
          pulseLength[j] = tmp;                //storing last value
        }
      }      
    }

      /* Print middle one */
      Midcm = pulseLength[2] / 58;
      Serial.print("Median Distance = ");
      Serial.print(Midcm);
      Serial.println(" centimeters");
      delay(5000);

      Serial.println("End HC-SR04 Test.");  
      
}
