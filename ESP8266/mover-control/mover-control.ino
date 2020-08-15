// Caravan twin motor mover control - wired version
// ST 2020

#include <Wire.h>
#include <Adafruit_ADS1015.h>
Adafruit_ADS1115 ads;
int16_t adc1;
int16_t adc2;

int ledPin = 10;
int switchPin = 16;
int leftPin = 10;
int rightPin = 11;
int leftDirPinFwd = 13;
int leftDirPinBkwd = 15;
int rightDirPinFwd = 0;
int rightDirPinBkwd = 2;
int leftOnPin = 1;
int rightOnPin = 3;

int xPosition = 0;
int yPosition = 0;
bool isOn = false;

int r_max = 17451;
int r_min = 0;
int deadOffset = 2;
int motorValues[2] = {0,0};

void setup() {
  pinMode(ledPin, OUTPUT);
  pinMode(switchPin, INPUT);
  pinMode(leftDirPinFwd, OUTPUT);
  pinMode(leftDirPinBkwd, OUTPUT);
  pinMode(rightDirPinFwd, OUTPUT);
  pinMode(rightDirPinBkwd, OUTPUT);
  pinMode(leftOnPin, OUTPUT);
  pinMode(rightOnPin, OUTPUT);

  Serial.begin(9600);
  Serial.println("Starting");
  digitalWrite(ledPin, HIGH); // TURN OFF
}

void loop() {

  if (digitalRead(switchPin) == LOW) {
    if (isOn) {
      digitalWrite(ledPin, HIGH);
      Serial.println("OFF - LOW");
    } else {
      digitalWrite(ledPin, LOW);
      Serial.println("ON - HIGH");
    }
    isOn = !isOn;
    delay(500); // de-bounce
  }

  if (isOn) {
    adc1 = ads.readADC_SingleEnded(1);
    adc2 = ads.readADC_SingleEnded(2);
    
    xPosition = reduceRange(adc1); 
    yPosition = reduceRange(adc1); 
    //Serial.print(xPosition); Serial.print(" : "); Serial.println(yPosition);    

    int left = 0;
    int right = 0;

    if (xPosition > deadOffset || xPosition < deadOffset*-1
      || yPosition > deadOffset || yPosition < deadOffset*-1) {
        setMotorValues(xPosition, yPosition);    
        bool leftFwd = false;
        bool rightFwd = false;
        
        leftFwd = motorValues[0] > 0;
        rightFwd = motorValues[1] > 0;

        direction(leftFwd, rightFwd);
       
        if (leftFwd) {
          left = motorValues[0];
        } else {
          left = motorValues[0]*-1;
        }
        if (rightFwd) {
          right = motorValues[1];
        } else {
          right = motorValues[1]*-1;
        }

        if (yPosition < 0) {
          int h = left;
          left = right;
          right = h;
        }
      } else {
        left = 0;
        right = 0;
      }

      Serial.print("MOTOR: "); Serial.print(motorValues[0]); Serial.print(" : "); Serial.print(motorValues[1]); 
      Serial.print(" : "); Serial.print(left); Serial.print(" : "); Serial.println(right);  

      analogWrite(leftPin, left/2);
      analogWrite(rightPin, right/2);
  }
}


void direction(bool left, bool right) {
  if (left) {
    digitalWrite(leftDirPinBkwd, LOW);  
    digitalWrite(leftDirPinFwd, HIGH);  
  } else {
    digitalWrite(leftDirPinFwd, LOW);  
    digitalWrite(leftDirPinBkwd, HIGH);  
  }

  if (right) {
    digitalWrite(rightDirPinBkwd, LOW);  
    digitalWrite(rightDirPinFwd, HIGH);  
  } else {
    digitalWrite(rightDirPinFwd, LOW);  
    digitalWrite(rightDirPinBkwd, HIGH);  
  }
}

int reduceRange(int pos) {
  // Val from 0 - max (17451)
  // Convert to -128 - +127
  float s = pos-r_max/2;
  return s/(r_max/256);  
}

void setMotorValues(int nJoyX, int nJoyY) {
    // Differential Steering Joystick Algorithm
  // ========================================
  //   by Calvin Hass
  //   https://www.impulseadventure.com/elec/
  //
  // Converts a single dual-axis joystick into a differential
  // drive motor control, with support for both drive, turn
  // and pivot operations.
  //
  
  // INPUTS
  //int     nJoyX;              // Joystick X input                     (-128..+127)
  //int     nJoyY;              // Joystick Y input                     (-128..+127)
  
  // OUTPUTS
  int     nMotMixL;           // Motor (left)  mixed output           (-128..+127)
  int     nMotMixR;           // Motor (right) mixed output           (-128..+127)
  
  // CONFIG
  // - fPivYLimt  : The threshold at which the pivot action starts
  //                This threshold is measured in units on the Y-axis
  //                away from the X-axis (Y=0). A greater value will assign
  //                more of the joystick's range to pivot actions.
  //                Allowable range: (0..+127)
  float fPivYLimit = 32.0;
        
  // TEMP VARIABLES
  float   nMotPremixL;    // Motor (left)  premixed output        (-128..+127)
  float   nMotPremixR;    // Motor (right) premixed output        (-128..+127)
  int     nPivSpeed;      // Pivot Speed                          (-128..+127)
  float   fPivScale;      // Balance scale b/w drive and pivot    (   0..1   )
  
  
  // Calculate Drive Turn output due to Joystick X input
  if (nJoyY >= 0) {
    // Forward
    nMotPremixL = (nJoyX>=0)? 127.0 : (127.0 + nJoyX);
    nMotPremixR = (nJoyX>=0)? (127.0 - nJoyX) : 127.0;
  } else {
    // Reverse
    nMotPremixL = (nJoyX>=0)? (127.0 - nJoyX) : 127.0;
    nMotPremixR = (nJoyX>=0)? 127.0 : (127.0 + nJoyX);
  }
  
  // Scale Drive output due to Joystick Y input (throttle)
  nMotPremixL = nMotPremixL * nJoyY/128.0;
  nMotPremixR = nMotPremixR * nJoyY/128.0;
  
  // Now calculate pivot amount
  // - Strength of pivot (nPivSpeed) based on Joystick X input
  // - Blending of pivot vs drive (fPivScale) based on Joystick Y input
  nPivSpeed = nJoyX;
  fPivScale = (abs(nJoyY)>fPivYLimit)? 0.0 : (1.0 - abs(nJoyY)/fPivYLimit);
  
  // Calculate final mix of Drive and Pivot
  nMotMixL = (1.0-fPivScale)*nMotPremixL + fPivScale*( nPivSpeed);
  nMotMixR = (1.0-fPivScale)*nMotPremixR + fPivScale*(-nPivSpeed);

  motorValues[0] = nMotMixL;
  motorValues[1] = nMotMixR;
}
