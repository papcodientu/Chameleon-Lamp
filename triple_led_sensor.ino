/*
Copyright (c) 2015, Son Vu
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the Fablab Saigon nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

// enable this line to use debug function
// #define DEBUG

// These are colors pre-defined
#define WHITE  { 255, 255, 255}
#define RED    { 255,   0,   0}
#define GREEN  {   0, 255,   0}
#define BLUE   {   0,   0, 255}

// This is the pre-defined color sequence
static const uint8_t colorTable[][3] = {
  WHITE,
  RED,
  GREEN,
  BLUE,
};
static const int colorTableSize = sizeof(colorTable) / sizeof(uint8_t) / 3;

// This is table of states required
typedef enum {
  STATE_IDLE,    // = 0; this is the main idle state
  
  // these three states handle when swiping from left to right
  STATE_WAIT1,   
  STATE_WAIT2,
  STATE_CHNG1,
  
  // these three states handle when swiping from right to left
  STATE_WAIT3,
  STATE_WAIT4,
  STATE_CHNG2,
  
  // these states handle the shutdown and startup sequence
  STATE_SHUTDOWN,
  STATE_DOOFF,
  STATE_OFF,
  STATE_DOON,
  STATE_STARTUP,
} state_t;

state_t currentState, nextState;

// constant limiting time for swiping (in swiping gesture)
#define TIMEOUT        400
// constant limiting time for holding (in holding gesture)
#define TIMESHUTDOWN   2000

// two variables to count how much time was spent in each state
long int currentTime;
long int lastTime;

// function changeColor
//   input i: offset of color to change, according to the color table
//   input bool: turn on or off to the color
void changeColor(int i, bool off) {
  static int currentColor = 0;  
  
  // we want to turn off the light
  if (off == true) {
    // turn off the pwm
    analogWrite(9 , 0);
    analogWrite(10, 0);
    analogWrite(11, 0); 
    return;
  }
  
  // some wrap around to prevent overflowing
  currentColor += i;
  while (currentColor >= colorTableSize)
    currentColor -= colorTableSize;
  while (currentColor < 0)
    currentColor += colorTableSize;
  
#ifdef DEBUG  
  Serial.print("Current state: ");
  Serial.println(currentColor);
  
  Serial.print("Color: ");
  Serial.print(colorTable[currentColor][0]);
  Serial.print(", ");
  Serial.print(colorTable[currentColor][1]);
  Serial.print(", ");
  Serial.println(colorTable[currentColor][2]);
#endif

  // output the color, with GRB channel
  analogWrite(9 , colorTable[currentColor][1]);
  analogWrite(10, colorTable[currentColor][0]);
  analogWrite(11, colorTable[currentColor][2]);
}

// threshold for each infrared sensors
int threshold0;
int threshold1;
int threshold2;

void setup() {
  // begin the serial debug
  Serial.begin(38400);
  
  // initialized the state machine
  currentState = STATE_IDLE;
  currentTime = millis();
  
  // calibrate the sensor
  threshold0 = 600;
  threshold1 = 600;
  threshold2 = 600;
  
  // turn on the lamp
  changeColor(0, true);
}

void loop() {
  // continuouly read the analog pins...
  int a0 = analogRead(A0);
  int a1 = analogRead(A1);
  int a2 = analogRead(A2);

#ifdef DEBUG
  Serial.print("Read: ");
  Serial.print(a0);
  Serial.print(", ");
  Serial.print(a1);
  Serial.print(", ");
  Serial.println(a2);  
#endif

  // as well as measure the current time  
  currentTime = millis();
  
  switch (currentState) {
    // if we are in the idle state
    //    change to state WAIT1 if only the leftmost sensor is blocked
    //    or change to state WAIT3 if only the rightmost sensor is blocked
    //    or change to state SHUTDOWN if all three sensors are blocked 
    case STATE_IDLE:
      if ((a0 > threshold0) && (a1 < threshold1) && (a2 < threshold2))
        nextState = STATE_WAIT1;
      if ((a2 > threshold2) && (a1 < threshold1) && (a0 < threshold0))
        nextState = STATE_WAIT3;
      if ((a0 > threshold0) && (a1 > threshold1) && (a2 > threshold2))
        nextState = STATE_SHUTDOWN;
      break;
    
    // this served as an intermediate state
    // if we are in this state
    //   back to idle is the middle sensor does not detect anything
    //   move to state WAIT2 if the middle sensor detect something
    case STATE_WAIT1:
      if (a1 > threshold1) {
        nextState = STATE_WAIT2;
      }
     
      if ((currentTime - lastTime) > TIMEOUT) {
        nextState = STATE_IDLE;
      }
      break;
    
    // this served as an intermediate state
    // if we are in this state
    //   back to idle is the rightmost sensor does not detect anything
    //   move to state CHNG1 if the rightmost sensor detect something  
    case STATE_WAIT2:
      if (a2 > threshold2) {
        nextState = STATE_CHNG1;
      }
     
      if ((currentTime - lastTime) > TIMEOUT) {
        nextState = STATE_IDLE;
      }
      break;

    // if we are in this state
    //   back to idle is the middle sensor does not detect anything
    //   move to state WAIT2 if the middle sensor detect something
    case STATE_WAIT3:
      if (a1 > threshold1) {
        nextState = STATE_WAIT4;
      }
     
      if ((currentTime - lastTime) > TIMEOUT) {
        nextState = STATE_IDLE;
      }
      break;

    // if we are in this state
    //   back to idle is the leftmost sensor does not detect anything
    //   move to state CHNG2 if the leftmost sensor detect something        
    case STATE_WAIT4:
      if (a0 > threshold0) {
        nextState = STATE_CHNG2;
      }
     
      if ((currentTime - lastTime) > TIMEOUT) {
        nextState = STATE_IDLE;
      }
      break;

    // complete motion detected, so we change color
    case STATE_CHNG1:
      Serial.println("Left to Right");
      changeColor(1, false);
      nextState = STATE_IDLE; 
      break;
    
    // complete motion detected, so we change color
    case STATE_CHNG2:
      Serial.println("Right to Left");
      changeColor(-1, false);
      nextState = STATE_IDLE; 
      break;
    
    // if any of the sensors are unblocked before timing out, return to idle
    // if all sensors are blocked till the end, init the DO_OFF sequence  
    case STATE_SHUTDOWN:
      if ((a0 < threshold0) || (a1 < threshold1) || (a2 < threshold2))
        nextState = STATE_IDLE;
      if ((currentTime - lastTime) > TIMESHUTDOWN)
        nextState = STATE_DOOFF;
      break;
    
    // turn off the light, and return to the OFF state when all three sensors are 
    //    unblocked. This is necessary to prevent looping between state IDLE and OFF
    case STATE_DOOFF:
      changeColor(0, true);
      if ((a0 < threshold0) && (a1 < threshold1) && (a2 < threshold2))
        nextState = STATE_OFF;
      break;
    
    // in this state, all LED are off, we only wait for the startup sequence to happen 
    case STATE_OFF:
      if ((a0 > threshold0) && (a1 > threshold1) && (a2 > threshold2))
        nextState = STATE_STARTUP;
      break;

    // if any of the sensors are unblocked before timing out, return to idle
    // if all sensors are blocked till the end, init the DO_ON sequence        
    case STATE_STARTUP:
      if ((a0 < threshold0) || (a1 < threshold1) || (a2 < threshold2))
        nextState = STATE_OFF;
      if ((currentTime - lastTime) > TIMESHUTDOWN)
        nextState = STATE_DOON;
      break;

    // turn on the light, and return to the IDLE state when all three sensors are 
    //    unblocked. This is necessary to prevent looping between state IDLE and OFF    
    case STATE_DOON:
      changeColor(0, false);
      if ((a0 < threshold0) && (a1 < threshold1) && (a2 < threshold2))
        nextState = STATE_IDLE;
      break;
 
    // invalid machine state, should direct it back to IDLE state   
    default: nextState = STATE_IDLE; break;
  }
  
  // update the state machine
  if (currentState != nextState) { 
#ifdef DEBUG
    Serial.print("State: ");
    Serial.print(currentState);
    Serial.print(", ");
    Serial.println(nextState);
#endif    
    // backup the last time since state entered
    currentState = nextState;
    lastTime = millis();
  }
}
