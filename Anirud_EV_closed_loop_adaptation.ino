/* Ashwin R.
 *  
 * This program is for an ev w/ stepper motor(nema17)
 * Aims to go a given distance at an exact time.
 * Velocity and acceleration can both be limited
 * 
 * Program always prioritizes distance over time, meaning
 * time is automatically adjusted when it is not sufficient
 * for distance
 * 
 * This is a closed loop system using as5600 for position
 * control.
 * 
*/

/* L298n Motor Driver Pins */
//Pins that control direction of movement
#define MotorDirPin1 9
#define MotorDirPin2 10

//Pin that control motor speed
#define MotorSpeedPin 11

volatile unsigned long counter = 0;  //This variable will increase or decrease depending on the rotation of encoder.

double pulsesPerRev = 102; //This vairable sets the pulses/revolution of the encoder. 

//Encoder Pins
//This code uses attachInterrupt 0 and 1 which are pins 2 and 3 moust Arduino.
#define EncPinA 2 //Pin for input A
#define EncPinB 3 //Pin for input B


/* Start Button Pin + Variables */
#define StartButtonPin 8 //Start Button Pin

int start = 0; //Used to start vehicle when button is pressed

float targetTime = 6.0; //s
const float targetDistance = 699.0; //steps
float maxVelocity = 600.0; //steps/s
float maxAccel = 30.0; //steps/s^2

float cruiseTime = 0;
float accelTime = (targetTime - cruiseTime )/2;
float decelTime = accelTime;
float a = maxVelocity/accelTime; 

float accelDist = 0.5*a*accelTime*accelTime;
float decelDist = accelDist;
float cruiseDist = targetDistance - (accelDist+decelDist);


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

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

  while(! start) {
    if (digitalRead(StartButtonPin) == LOW){
        Serial.println("pressed");
        counter = 0;
        start = 1;
    }
  }
  
  
  if(cruiseDist < 0) { //triangular profile
    accelTime = targetTime/2;
    a = 4*targetDistance/(targetTime*targetTime);
    maxVelocity = a * accelTime;
    decelTime = accelTime;
    cruiseTime = 0;
  }
  else { //trapezoidal profile
    cruiseTime = cruiseDist/maxVelocity;
    accelTime = (targetTime - cruiseTime)/2;
    decelTime = accelTime;
    a = maxVelocity/accelTime;
  }
  //adjust target time if necessary
  if(accelDist + cruiseDist + decelDist < targetDistance) {
    a = maxAccel;

    accelTime = maxVelocity/a;
    decelTime = accelTime;

    accelDist = 0.5 * a * accelTime * accelTime;
    decelDist = accelDist;

    cruiseDist = targetDistance - (accelDist + decelDist);
    cruiseTime = cruiseDist/maxVelocity;

    targetTime = accelTime + cruiseTime + decelTime;
  }

  Serial.print("accel:"); Serial.println(a);
  Serial.print("vel:"); Serial.println(maxVelocity);
  Serial.print("accelTime:"); Serial.println(accelTime);

  float pos, vel, t = 0;
  int currentSteps = 0;
  long startTime = 0;
  

  while(t<=targetTime){

    startTime = micros();

    Serial.println(counter);

    if(t < accelTime) { //accel phase
      vel = a *t;
      pos = 0.5*a*t*t;
    }
    else if(t < accelTime+cruiseTime) { //const phase
      vel = maxVelocity;
      float cruiseElapsedTime = t-accelTime;
      pos = (0.5*a*accelTime*accelTime) + (maxVelocity * cruiseElapsedTime);
    }
    else{ //decel phase
      float decelElapsedTime = t - accelTime - cruiseTime;
      vel = maxVelocity - a * decelElapsedTime;
      pos = (0.5 * a * accelTime*accelTime) + (maxVelocity * cruiseTime) + (maxVelocity * decelElapsedTime - 0.5 * a * decelElapsedTime*decelElapsedTime);
    }
    //Serial.println(pos);
    /*
    if(currentSteps < pos){
      stepMotor();
      currentSteps++;
    }
    */
    analogWrite(MotorSpeedPin, 255); 

    
    delayMicroseconds(500);
    t+=(micros()-startTime)/1000000.0;
  }

}

void loop() {
  // put your main code here, to run repeatedly:

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
  // ai1 is activated if DigitalPin nr 3 is going from LOW to HIGH
  // Check with pin 2 to determine the direction
  if(digitalRead(2)==LOW) {
    counter--;
  }else{
    counter++;
  }
}