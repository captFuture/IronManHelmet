/*
 
MIT License

Copyright (c) 2024 captFuture

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

DESCRIPTION
  ====================
  The purpose of this code is to automate the servos and LED eyes for the Iron Man helmet on an esp32
  Motivation and inspiration comes from the early work by "XL97" of The RPF Community and the code is based on https://github.com/crashworks3d/Iron_Man_Servo

DEVELOPED BY
  ====================
  captFuture

 */

// Definitions for FastLED Library for rgb eyes
// Count of Leds for Repulsor/arc and eyes
#define NUM_LEDS_1 20
#define NUM_LEDS_2 4

#define TIMER_INTERRUPT_DEBUG       1
#define ISR_SERVO_DEBUG             1

// Select different ESP32 timer number (0-3) to avoid conflict
#define USE_ESP32_TIMER_NO          3
#define NUM_SERVOS    2
int servoIndex1  = -1;
int servoIndex2  = -1;

bool battlemode = false;

// sound board pins
#define RX_PIN 16 // set pin for receive (RX) communications
#define TX_PIN 17 // set pin for transmit (TX) communications

#define VOLUME 25 // sound board volume level (30 is max)

// Servo configuration types.  Use one of these values to set the SERVO_TYPE definition below
#define TPMG90S 0
#define GENERIC 1
#define MANUAL  2

// Defines which servo type is used
//#define SERVO_TYPE TPMG90S // Uncomment this line if you are using genuine Tower Pro MG90S servos
//#define SERVO_TYPE GENERIC // Uncomment this line if you are using generic servos
#define SERVO_TYPE MANUAL // Uncomment this line if you are manually configuring your servos in the manual configuration below

#if  (SERVO_TYPE == TPMG90S)
static const int PWM_HIGH = 2400; // Authentic Tower Pro MG90s Servo using 12% Duty Cycle
static const int PWM_LOW = 400; // Authentic Tower Pro MG90s Servo using 2% Duty Cycle

#elif (SERVO_TYPE == GENERIC)
static const int PWM_HIGH = 2500;// Generic MG90s Servo using 13% Duty Cycle
static const int PWM_LOW = 200; // Generic MG90s Servo using 1% Duty Cycle

// Use these settings for manual configuration of servos
#elif (SERVO_TYPE == MANUAL)
static const int PWM_HIGH = 2400;
static const int PWM_LOW = 400;
#endif

// Declare pin settings
static const int SERVO1_PIN = 18; // set the pin for servo 1
static const int SERVO2_PIN = 19; // set the pin for servo 2

// Declare variables for servo speed control
#define SERVO_CLOSE_SPEED 175 // set the speed of the servo close function
#define SERVO_OPEN_SPEED 255 // set the speed of the servo opening recommend set to max speed to aid in lift

// In Dual Servo Configuration the servos move in opposing directions, so the angles of the servos will be opposite to each other. 
// Normal Servo range is 0° ~ 180°, for initial setup the range has been adjusted to 20° ~ 160°, this allows for a 20° adjustment at both ends of the servo range.
// See Helmet tutorial for further information on servo setup.
#define SERVO1_OPEN_POS 20  // set the open position of servo 1
#define SERVO2_OPEN_POS 160 // set the open position of servo 2
#define SERVO1_CLOSE_POS 160 // set the closed position of servo 1
#define SERVO2_CLOSE_POS 20 // set the closed position of servo 2

#define BUTTON_PIN 32 // the pin that the pushbutton is attached to
#define BUTTON2_PIN 12 // the pin that the repulsor pushbutton is attached to

#define DATA_PIN_1 21 // Eye Leds
#define DATA_PIN_2 22 // Arc+Repulsor Leds

// Declare variables for setup special effects (applies to LED eyes only for now)
// Declare variables for LED eyes special effects (applies to LED eyes only for now)
#define EYES_NONE 0 // No special effects, just turn on the LED eyes
#define EYES_MOVIE_BLINK 1 // Blink LED eyes on setup, sequence based on Avengers Movie
#define EYES_FADE_ON 2 // Slowly brighten LED eyes until fully lit

// To use the specific feature below
// use double slashes "//" to comment, or uncomment (remove double slashes) in the code below

// Uncomment this line if you don't want any special effect during startup, comment this line to disable this effect
///#define SETUP_FX EYES_NONE

// Uncomment this line if you want the movie blink special effect during startup, comment this line to disable this effect
#define SETUP_FX EYES_MOVIE_BLINK

// Uncomment this line if you want the fade on special effect during startup, comment this line to disable this effect
// #define SETUP_FX EYES_FADE_ON

// To use the specific feature below
// use double slashes "//" to comment, or uncomment (remove double slashes) in the code below

// Uncomment this line if you don't want any special effect during operation, comment this line to disable this effect
//#define EYES_FX EYES_NONE

// Uncomment this line if you want the movie blink special effect during operation, comment this line to disable this effect
// #define EYES_FX EYES_MOVIE_BLINK

// Uncomment this line if you want the fade on special effect during operation, comment this line to disable this effect
#define EYES_FX EYES_FADE_ON
