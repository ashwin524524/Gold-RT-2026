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
#include "AS5600.h"
AS5600 as5600; 

const int STEP_PIN = 7;
const int DIR_PIN = 6;

int buttonState;
int lastButtonState = LOW;
unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 50;

byte buttonPin = 4;

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

  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);

  Wire.begin();

  as5600.begin();  //  set direction pin.
  as5600.setDirection(AS5600_COUNTERCLOCK_WISE);  //  default, just be explicit.

  int b = as5600.isConnected();
  Serial.print("Connect: ");
  Serial.println(b);

  
  
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
  while(!button());

  Serial.print("accel:"); Serial.println(a);
  Serial.print("vel:"); Serial.println(maxVelocity);
  Serial.print("accelTime:"); Serial.println(accelTime);

  float pos, vel, t = 0;
  int currentSteps = 0;
  long startTime = 0;
  

  as5600.getCumulativePosition();
  as5600.resetCumulativePosition();

  int sensedSteps;

  while(t<=targetTime){

    startTime = micros();

    sensedSteps = (int) as5600.getCumulativePosition()/20.48;

    Serial.println(sensedSteps);

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
    if((sensedSteps+currentSteps)/2 < pos){
      stepMotor();
      sensedSteps++;
    }

    
    delayMicroseconds(500);
    t+=(micros()-startTime)/1000000.0;
  }

}

void loop() {
  // put your main code here, to run repeatedly:

}

void stepMotor() {
  digitalWrite(DIR_PIN, HIGH);
  digitalWrite(STEP_PIN, HIGH);
  delayMicroseconds(4);  // minimal pulse width for A4988
  digitalWrite(STEP_PIN, LOW);
  //delayMicroseconds(1000);
}

bool button() {
  while (true) {
    int reading = digitalRead(buttonPin);

    // check to see if you just pressed the button
    // (i.e. the input went from LOW to HIGH), and you've waited long enough
    // since the last press to ignore any noise:

    // If the switch changed, due to noise or pressing:
    if (reading != lastButtonState) {
      // reset the debouncing timer
      lastDebounceTime = millis();
    }

    if ((millis() - lastDebounceTime) > debounceDelay) {
      // if the button state has changed:
      if (reading != buttonState) {
        return true;
      }
    }
    lastButtonState = reading;
  }
}
