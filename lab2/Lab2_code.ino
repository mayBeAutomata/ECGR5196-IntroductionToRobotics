//*******************************************************************
// Blink - verify my MSP432 board works by blinking one LED
// Anyaegbu Somto, 2021-02-21
// Off -> red -> blue -> green -> red&blue -> red&green -> blue&green -> red&blue&green -> Off (repeat)
//*******************************************************************


#define RED 75                    // Define RED of the tri-color LED as pin 75
#define GREEN 76                  // Define GREEN of the tri-color LED as pin 76
#define BLUE 77                   // Define BLUE of the tri-color LED as pin 77

  
void setup() {                    // put your setup code here, to run once:
  
  // initialize one digital pin as outputs.
  pinMode(RED, OUTPUT);           //RED LED
  pinMode(GREEN, OUTPUT);         //GREEN LED
  pinMode(BLUE, OUTPUT);          //BLUE LED
  
}

void loop() {    // put your main code here, to run repeatedly: 
  digitalWrite(RED, HIGH);        // turn the RBG (RED) LED on (HIGH is the voltage level)
  delay(500);                     // wait for half a second
  digitalWrite(RED, LOW);         // turn the RBG (RED) LED off by making the voltage LOW
  delay(500);                     // wait for half a second
  
  digitalWrite(BLUE, HIGH);       // turn the RBG (BLUE) LED on (HIGH is the voltage level)
  delay(500);                     // wait for half a second
  digitalWrite(BLUE, LOW);        // turn the RBG (BLUE) LED off by making the voltage LOW
  delay(500);                     // wait for half a second
  
  digitalWrite(GREEN, HIGH);      // turn the RBG (GREEN) LED on (HIGH is the voltage level)
  delay(500);                     // wait for half a second
  digitalWrite(GREEN, LOW);       // turn the RBG (GREEN) LED off by making the voltage LOW
  delay(500);                     // wait for half a second
  
  digitalWrite(RED, HIGH);        // turn the RBG (RED) LED on (HIGH is the voltage level)
  digitalWrite(BLUE, HIGH);       // turn the RBG (BLUE) LED on (HIGH is the voltage level)
  delay(500);                     // wait for half a second
  digitalWrite(RED, LOW);         // turn the RBG (RED) LED off by making the voltage LOW
  digitalWrite(BLUE, LOW);        // turn the RBG (BLUE) LED off by making the voltage LOW
  delay(500);                     // wait for half a second
  
  digitalWrite(RED, HIGH);        // turn the RBG (RED) LED on (HIGH is the voltage level)
  digitalWrite(GREEN, HIGH);      // turn the RBG (GREEN) LED on (HIGH is the voltage level)
  delay(500);                     // wait for half a second
  digitalWrite(RED, LOW);         // turn the RBG (RED) LED off by making the voltage LOW
  digitalWrite(GREEN, LOW);       // turn the RBG (GREEN) LED off by making the voltage LOW
  delay(500);                     // wait for half a second
  
  digitalWrite(BLUE, HIGH);       // turn the RBG (BLUE) LED on (HIGH is the voltage level)
  digitalWrite(GREEN, HIGH);      // turn the RBG (GREEN) LED on (HIGH is the voltage level)
  delay(500);                     // wait for half a second
  digitalWrite(BLUE, LOW);        // turn the RBG (BLUE) LED off by making the voltage LOW
  digitalWrite(GREEN, LOW);       // turn the RBG (GREEN) LED off by making the voltage LOW
  delay(500);                     // wait for half a second
  
  digitalWrite(RED, HIGH);        // turn the RBG (RED) LED on (HIGH is the voltage level)
  digitalWrite(BLUE, HIGH);       // turn the RBG (BLUE) LED on (HIGH is the voltage level)
  digitalWrite(GREEN, HIGH);      // turn the RBG (GREEN) LED on (HIGH is the voltage level)
  delay(500);                     // wait for half a second
  digitalWrite(RED, LOW);         // turn the RBG (RED) LED off by making the voltage LOW
  digitalWrite(BLUE, LOW);        // turn the RBG (BLUE) LED off by making the voltage LOW
  digitalWrite(GREEN, LOW);       // turn the RBG (GREEN) LED off by making the voltage LOW
  delay(500);                     // wait for half a second
}
