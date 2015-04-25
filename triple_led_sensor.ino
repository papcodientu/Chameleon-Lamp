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

Change log
Rev 1.0 18/04/2015: First release
Rev 1.1 25/04/2015: Added command shell
*/

#include <string.h>
#include <EEPROM.h>

// change debug level to 3 to see all debug information
#define DEBUGLEVEL 0

// These are colors pre-defined
#define WHITE  { 0xff, 0xff, 0xff }
#define RED    { 0xff,    0,    0 }
#define GREEN  {    0, 0xff,    0 }
#define BLUE   {    0,    0, 0xff }
#define MAGENTA{ 0xff,    0, 0xff }
#define YELLOW { 0xff, 0xff,    0 }
#define CYAN   {    0, 0xff, 0xff }
#define BGREEN { 0x66, 0xff,    0 }
#define CRIMSON{ 0xdc, 0x14, 0x3c }

#define MAX_NUMBER_OF_COLOR 9
// This is the pre-defined color sequence
uint8_t colorTable[MAX_NUMBER_OF_COLOR][3] = {
  WHITE,
  RED,
  GREEN,
  BLUE,
  MAGENTA,
  YELLOW,
  CYAN,
  BGREEN,
  CRIMSON
};

uint8_t colorTableSize = MAX_NUMBER_OF_COLOR;

#define EEPROM_COLOR_TABLE_SIZE 0x00
#define EEPROM_COLOR_TABLE      0x10

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

// define a simple command line parsing facility
#define SHELL_MAX_LINE_LENGTH       64
#define SHELL_MAX_ARGUMENTS         4

// utility function: equal to strtok

char *_strtok(char *str, const char *delim, char **saveptr) {
  char *token;
  if (str)
    *saveptr = str;
  token = *saveptr;

  if (!token)
    return NULL;

  token += strspn(token, delim);
  *saveptr = strpbrk(token, delim);
  if (*saveptr)
    *(*saveptr)++ = '\0';

  return *token ? token : NULL;
}

// utility function: convert a string to hex number
int _atoh(char *a) {
  int tmp = 0;
  int i = 0;

  while (1) {
    if ((a[i] >= '0') && (a[i] <= '9')) {
      tmp = tmp * 16 + (a[i] - '0');
      i++; continue;
    } 

    if ((a[i] >= 'A') && (a[i] <= 'F')) {
      tmp = tmp * 16 + (a[i] - 'A' + 10);
      i++; continue;
    }

    if ((a[i] >= 'a') && (a[i] <= 'f')) {
      tmp = tmp * 16 + (a[i] - 'a' + 10);
      i++; continue;
    }

    break;
  }
  return tmp;
}

// utility function: read a line from Serial
int _readline(int readch, char *buffer, int len)
{
  static int pos = 0;
  int rpos;

  if (readch > 0) {
    switch (readch) {
      case '\n': // Ignore new-lines
        break;
      case '\r': // Return on CR
        rpos = pos;
        pos = 0;  // Reset position index ready for next time
        return rpos;
      default:
        if (pos < len-1) {
          buffer[pos++] = readch;
          buffer[pos] = 0;
        }
    }
  }
  // No end of line has been found, so return -1.
  return -1;
}

static int currentColor = 0; 
// function changeColor
//   input i: offset of color to change, according to the color table
//   input bool: turn on or off to the color
void changeColor(int i, bool off) {
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
  
#if DEBUGLEVEL >= 3
  Serial.print("Current state: ");
  Serial.println(currentColor);
  
  Serial.print("Color: ");
  Serial.print(colorTable[currentColor][0]);
  Serial.print(", ");
  Serial.print(colorTable[currentColor][1]);
  Serial.print(", ");
  Serial.println(colorTable[currentColor][2]);
#endif

  // output the color, with BRG channel
  analogWrite(9 , colorTable[currentColor][2]);
  analogWrite(10, colorTable[currentColor][0]);
  analogWrite(11, colorTable[currentColor][1]);
}

void cmd_getcolor(int argc, char *argv[]) {
  if (argc != 0) {
    Serial.println("Error");
    return; 
  }
  
  Serial.println(colorTableSize);
  for (int i = 0; i < colorTableSize; i++) {
    Serial.print(colorTable[i][0], HEX);
    Serial.print(" ");
    Serial.print(colorTable[i][1], HEX);
    Serial.print(" ");
    Serial.println(colorTable[i][2], HEX);
  }
}

void cmd_setcolor(int argc, char *argv[]) {
  if (argc != 4) {
    Serial.println("Error"); 
  }
  
  int n = _atoh(argv[0]) - 1;
  if (n >= colorTableSize) {
    Serial.println("Error");
    return;
  }
  
  colorTable[n][0] = _atoh(argv[1]);
  colorTable[n][1] = _atoh(argv[2]);
  colorTable[n][2] = _atoh(argv[3]);
  changeColor(0, false);
}

void cmd_ncolor(int argc, char *argv[]) {
  if (argc != 1) {
    Serial.println("Error");
    return; 
  }
  
  // change the number of color
  int n = _atoh(argv[0]);
  if (n > 10) {
    Serial.println("Error");
    return; 
  }
  
  colorTableSize = n;
  changeColor(0, false);
}


void cmd_save(int argc, char *argv[]) {
  if (argc != 0) {
    Serial.println("Error");
    return; 
  }
  
  EEPROM.write(EEPROM_COLOR_TABLE_SIZE, colorTableSize);
  for (int i = 0; i < MAX_NUMBER_OF_COLOR; i++) {
    EEPROM.write(EEPROM_COLOR_TABLE + i*3 + 0, colorTable[i][0]);
    EEPROM.write(EEPROM_COLOR_TABLE + i*3 + 1, colorTable[i][1]);
    EEPROM.write(EEPROM_COLOR_TABLE + i*3 + 2, colorTable[i][2]);
  }
}

void cmd_color(int argc, char *argv[]) {
  if (argc != 3) {
    Serial.println("Error");
    return; 
  }
  
  colorTable[currentColor][0] = _atoh(argv[0]);
  colorTable[currentColor][1] = _atoh(argv[1]);
  colorTable[currentColor][2] = _atoh(argv[2]);
  analogWrite(9 , _atoh(argv[2]));
  analogWrite(10, _atoh(argv[0]));
  analogWrite(11, _atoh(argv[1]));
  changeColor(0, false);
}

// threshold for each infrared sensors
int threshold0;
int threshold1;
int threshold2;

void setup() {
  // begin the serial debug
  Serial.begin(9600);
  
  // load the table from ROM
  // check if the EEPROM is virgin
  int n = EEPROM.read(EEPROM_COLOR_TABLE_SIZE);
  
  // the EEPROM was written, therefore we read the old value from it
  if (n != 0xFF) {
    colorTableSize = EEPROM.read(EEPROM_COLOR_TABLE_SIZE); 
    for (int i = 0; i < MAX_NUMBER_OF_COLOR; i++) {
      colorTable[i][0] = EEPROM.read(EEPROM_COLOR_TABLE + i*3 + 0);
      colorTable[i][1] = EEPROM.read(EEPROM_COLOR_TABLE + i*3 + 1);
      colorTable[i][2] = EEPROM.read(EEPROM_COLOR_TABLE + i*3 + 2);
    }
  } 
  
  // initialized the state machine
  currentState = STATE_IDLE;
  currentTime = millis();
  
  // calibrate the sensor
  threshold0 = 600;
  threshold1 = 600;
  threshold2 = 600;
  
  // turn on the lamp
  changeColor(0, false);
}

void loop() {
  static char line[SHELL_MAX_LINE_LENGTH];
  
  int n;
  char *lp, *cmd, *tokp; 
  char *args[SHELL_MAX_ARGUMENTS + 1];
  
  // try to parse the command line
  if (_readline(Serial.read(), line, SHELL_MAX_LINE_LENGTH) > 0) {
    lp = _strtok(line, " \t", &tokp);
    cmd = lp;
    n = 0;
    while ((lp = _strtok(NULL, " \t", &tokp)) != NULL) {
      if (n >= SHELL_MAX_ARGUMENTS) {
        Serial.println("too many arguments");
        cmd = NULL;
        break;
      }
      args[n++] = lp;
    }
    args[n] = NULL;
    
    // comparing command and execute command
    if (cmd != NULL) {
      // compare it with a number of predetermined command
      if (0 == strcasecmp("getcolor", cmd)) {
         cmd_getcolor(n, args); 
      } else
      
      if (0 == strcasecmp("setcolor", cmd)) {
         cmd_setcolor(n, args); 
      } else
      
      if (0 == strcasecmp("save", cmd)) {
         cmd_save(n, args); 
      } else
      
      if (0 == strcasecmp("ncolor", cmd)) {
         cmd_ncolor(n, args); 
      } else 
      
      if (0 == strcasecmp("color", cmd)) {
         cmd_color(n, args); 
      } else 
      {
        // invalid command
        Serial.print(cmd);
        Serial.println(" ?");
      }
    }
  }
  
  // continuouly read the analog pins...
  int a0 = analogRead(A3);
  int a1 = analogRead(A4);
  int a2 = analogRead(A5);

#if DEBUGLEVEL >= 3
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
      if ((a1 > threshold1) && (a2 < threshold2)) {
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
      if ((a1 > threshold1) && (a0 < threshold0)) {
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
#if DEBUGLEVEL >= 1
      Serial.println("Left to Right");
#endif
      changeColor(1, false);
      nextState = STATE_IDLE; 
      break;
    
    // complete motion detected, so we change color
    case STATE_CHNG2:
 #ifdef DEBUGLEVEL >= 1
      Serial.println("Right to Left");
 #endif
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
#if DEBUGLEVEL >=1
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

