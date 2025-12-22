/*
UNPHAYZED SCIENCE OLYMPIAD
ELECTRIC VEHICLE CHAMPION KIT 2024-2025
*/


/* L298n Motor Driver Pins */
//Pins that control direction of movement
#define MotorDirPin1 9
#define MotorDirPin2 10

//Pin that control motor speed
#define MotorSpeedPin 11


/* Encoder Pins + Variables */
volatile unsigned long counter = 0;  //This variable will increase or decrease depending on the rotation of encoder.

double pulsesPerRev = 102; //This vairable sets the pulses/revolution of the encoder. 

//Encoder Pins
//This code uses attachInterrupt 0 and 1 which are pins 2 and 3 moust Arduino.
#define EncPinA 2 //Pin for input A
#define EncPinB 3 //Pin for input B

//Laser Pins and Variables
#define LaserPin 7
#define LaserButtonPin 5
int laser = 0;

/* Start Button Pin + Variables */
#define StartButtonPin 8 //Start Button Pin

int start = 0; //Used to start vehicle when button is pressed


/* Distance + Speed Control Variables */

//This kit uses a 2in diameter (~5.08 cm) wheel. 
//Adjust variable to wheel that you are using.
double wheelDiameterCM = 7.26; //Wheel diameter in cm

//Adjust variables to change the distance your vehicle travels.
double targetDistanceM = 8.00; //Target Distance in m
double slowDownDistance = 4.00; //Distance where vehicle begins to slow down in m

//Variables that convert distance values to encoder pules
double targetDistEncVal; //Encoder Value for Target Distance
double slowDownDistEncVal; //Encoder Value for Slow Down Distance

//Variables used to enter different loops during the vehicle's run
int vehicleMoved = 0;
int vehicleSlowedDown = 0;
int vehicleReachedTargetDistance = 0;


void setup() {
  // put your setup code here, to run once:

  Serial.begin(9600); //Starts Serial Monitor

  //Initalize motor pins
  pinMode(MotorDirPin1, OUTPUT);
  pinMode(MotorDirPin2, OUTPUT);
  pinMode(MotorSpeedPin, OUTPUT);

  //Initalize Encoder pins
  pinMode(EncPinA, INPUT);
  pinMode(EncPinB, INPUT);
  digitalWrite(EncPinA, HIGH); // turn on pullup resistors
  digitalWrite(EncPinB, HIGH); // turn on pullup resistors

  //Setting up interrupt
  //A rising pulse from encodenren activated ai0(). AttachInterrupt 0 is DigitalPin 2 on moust Arduino.
  attachInterrupt(0, ai0, RISING);
  //B rising pulse from encodenren activated ai1(). AttachInterrupt 1 is DigitalPin 3 on moust Arduino.
  attachInterrupt(1, ai1, RISING);

  //Initalize button pins
  pinMode(StartButtonPin, INPUT);
  digitalWrite(StartButtonPin, HIGH); //enable pullups to make pin high

  pinMode(LaserButtonPin, INPUT);
  digitalWrite(LaserButtonPin, HIGH); //enable pullups to make pin high

  //Initalize Laser pins
  pinMode(LaserPin, OUTPUT);
  digitalWrite(LaserPin, LOW);

  //Reset Booleans
  start = 0;
  vehicleMoved = 0;
  vehicleSlowedDown = 0;
  vehicleReachedTargetDistance = 0;
  laser = 0;

  //Get Encoder Values
  targetDistEncVal = getEncoderValue (targetDistanceM, wheelDiameterCM, pulsesPerRev);
  slowDownDistEncVal = getEncoderValue (slowDownDistance, wheelDiameterCM, pulsesPerRev);

}

void loop() {
  // put your main code here, to run repeatedly:

  if (digitalRead(LaserButtonPin) == LOW){
      if(laser == 0){
        Serial.println("Laser On");
        digitalWrite(LaserPin, HIGH);
        laser = 1;
        delay (300);
      }
      else{
        Serial.println("Laser Off");
        digitalWrite(LaserPin, LOW);
        laser = 0;
        delay (300);
      }
  }

  if (digitalRead(StartButtonPin) == LOW){
      Serial.println("pressed");
      Serial.println(targetDistEncVal);
      Serial.println(slowDownDistEncVal);
      counter = 0;
      vehicleMoved = 0;
      vehicleSlowedDown = 0;
      vehicleReachedTargetDistance = 0;
      start = 1;
  }

  if (start == 1){
    Serial.println("start");
    //Set motor Direction
    digitalWrite(MotorDirPin1, HIGH);
    digitalWrite(MotorDirPin2, LOW);
    //Set motor speed
    analogWrite(MotorSpeedPin, 255); //(0  = off and 255 = max speed)
    vehicleMoved = 1;
    start = 0;
  }

  if (vehicleMoved == 1){
    if (counter >= slowDownDistEncVal && counter < targetDistEncVal){
      
      //loop to decelerate motor
      /*
      for (int i = 255; i > 0; i--){
        analogWrite(MotorSpeedPin, i);
        delay(1);
      }
      */
      analogWrite(MotorSpeedPin, 0); // Turn off Motor
      vehicleSlowedDown = 1;
      vehicleMoved = 0;
    }
  }

  if (vehicleSlowedDown == 1){
    if(counter >= targetDistEncVal){
      Serial.println("DONE");
      analogWrite(MotorSpeedPin, 0);
      //Reverse motor direction for 300 ms to stop vehicle
      digitalWrite(MotorDirPin1, LOW);
      digitalWrite(MotorDirPin2, HIGH);
      //Set motor speed
      analogWrite(MotorSpeedPin, 255); //(0  = off and 255 = max speed)
      delay(300);
      analogWrite(MotorSpeedPin, 0); //(0  = off and 255 = max speed)
      vehicleReachedTargetDistance = 1;
      vehicleSlowedDown = 0;
    }
  
  }

  if (vehicleReachedTargetDistance == 1){
    //Reset variables
    Serial.println(counter);
    counter = 0;
    vehicleMoved = 0;
    vehicleSlowedDown = 0;
    vehicleReachedTargetDistance = 0;
    start = 0;
  }

}


void ai0() {
  // ai0 is activated if DigitalPin nr 2 is going from LOW to HIGH
  // Check pin 3 to determine the direction
  if(digitalRead(3)==LOW) {
    counter++;
  }else{
    counter--;
  }
}

void ai1() {
  // ai0 is activated if DigitalPin nr 3 is going from LOW to HIGH
  // Check with pin 2 to determine the direction
  if(digitalRead(2)==LOW) {
    counter--;
  }else{
    counter++;
  }
}


double getEncoderValue (double targetDistanceM, double WD, double PR){
    double WheelCircumfrence = 3.14159 * WD;
    double tgtENCval = ((targetDistanceM * 100) / WheelCircumfrence) * PR;
    return tgtENCval;
  }

