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
#include <Arduino.h>
#include <SD.h>

#include <M5Unified.h>
#include "config.h"
#include "ESP32_New_ISR_Servo.h"
#include <FastLED.h>

// Declare variables for sound control
#define SND_CLOSE 1    // sound track for helmet closing sound
#define SND_JARVIS 2   // sound track for JARVIS sound
#define SND_OPEN 3     // sound track for helmet opening sound
#define SND_REPULSOR 4 // sound track for repulsor sound effect
#define SND_FRIDAY 5   // sound track for FRIDAY sound
#define SND_JARVIS2 6  // sound track for JARVIS sound

#define SND_IAMIRONMAN 7 // sound track for i am ironman
#define SND_REPULSOR2 8  // sound track for repulsor sound effect
#define SND_REPULSOR3 9  // sound track for repulsor sound effect
#define SND_REPULSOR4 10 // sound track for repulsor sound effect

#define VERSION "0.2"
#include "DFRobotDFPlayerMini.cpp"
void printDetail(uint8_t type, int value); // header method for implementation below; affects C++ compilers
DFRobotDFPlayerMini mp3Obj;                // Create object for DFPlayer Mini

// Array für die LED-Daten
CRGB leds1[NUM_LEDS_1];
CRGB leds2[NUM_LEDS_2];

// State of the faceplate 1 = open, 0 = closed
#define FACEPLATE_CLOSED 0
#define FACEPLATE_OPEN 1

#define ISRSERVO_CLOSED 0
#define ISRSERVO_OPEN 1

int facePlateCurMode = FACEPLATE_OPEN; // Keep track if the faceplate is open or closed
int facePlateIsrMode = ISRSERVO_OPEN;

// State of the LED eyes 1 = on, 2 = off
#define LED_EYES_OFF 0
#define LED_EYES_ON 1

// State of the LED eyes for dimming/brightening 1 = brighten, 2 = dim
#define LED_EYES_DIM_MODE 0
#define LED_EYES_BRIGHTEN_MODE 1

int ledEyesCurMode = LED_EYES_DIM_MODE; // Keep track if we're dimming or brightening
int ledEyesMinPwm = 0;
int ledEyesCurPwm = 0;           // Tracking the level of the LED eyes for dim/brighten feature
int ledEyesMaxPwm = 128;         // limiting max brightness of leds
const int ledEyesIncrement = 20; // Define the increments to brighten or dim the LED eyes

/**
 * Helper Method
 * Simulate a delay in processing without disabling the processor completely
 *
 * @param[out] period - the amount of time in milliseconds to delay
 *
 * See: https://randomnerdtutorials.com/why-you-shouldnt-always-use-the-arduino-delay-function/
 */
void simDelay(long period)
{
  long delayMillis = millis() + period;
  while (millis() <= delayMillis)
  {
    int x = 0; // dummy variable, does nothing
  }
}

/**
 * Simulate the eyes slowly blinking until fully lit
 */
void movieblink()
{
  Serial.println(F("Start Movie Blink.."));

  // pause for effect...
  simDelay(300);
  if (battlemode)
  {
    fill_solid(leds2, NUM_LEDS_2, CHSV(0, 255, ledEyesMaxPwm));
  }
  else
  {
    fill_solid(leds2, NUM_LEDS_2, CHSV(160, 255, ledEyesMaxPwm));
  }
  FastLED.show();

  int lowValue = ledEyesMaxPwm / 2;
  int delayInterval[] = {210, 126, 84};
  int delayVal = 0;

  // First blink on
  for (int i = 0; i <= lowValue; i++)
  {
    setLedEyes(i);
    delayVal = delayInterval[0] / lowValue;
    simDelay(delayVal);
  }

  // Turn off
  setLedEyes(0);
  simDelay(delayInterval[0]);

  // Second blink on
  for (int i = 0; i <= lowValue; i++)
  {
    setLedEyes(i);
    delayVal = delayInterval[1] / lowValue;
    simDelay(delayVal);
  }

  // Turn off
  setLedEyes(0);
  simDelay(delayInterval[1]);

  // Third blink on
  setLedEyes(lowValue);
  simDelay(delayInterval[2]);

  // Turn off
  setLedEyes(0);
  simDelay(delayInterval[2]);

  // All on
  setLedEyes(ledEyesMaxPwm);

  playSoundEffect(SND_JARVIS);
  simDelay(1000);

  mp3Obj.sleep();
}

/*
 * Simulate LED eyes slowly brightening until fully lit
 */
void fadeEyesOn()
{
  ledEyesCurMode = LED_EYES_BRIGHTEN_MODE;
  Serial.println(F("Brightening LED eyes"));

  if (battlemode)
  {
    fill_solid(leds2, NUM_LEDS_2, CHSV(0, 255, ledEyesMinPwm));
  }
  else
  {
    fill_solid(leds2, NUM_LEDS_2, CHSV(160, 255, ledEyesMinPwm));
  }
  // FastLED.show();
  ledEyesCurPwm = 0;
  // loop until fully lit
  while (ledEyesCurPwm < ledEyesMaxPwm)
  {
    setLedEyes(ledEyesCurPwm);
    simDelay(200);
    ledEyesBrighten();
    Serial.println();
  }
}

/**
 * Initialization method for DFPlayer Mini board
 */
void init_player()
{
  Serial2.begin(9600);

  if (!Serial2.available())
  {
    Serial.println(F("Serial object not available."));
  }

  Serial.println(F("Initializing DFPlayer ... (May take 3~5 seconds)"));

  bool dfInit = mp3Obj.begin(Serial2, false, true);

  simDelay(1000);

  if (!dfInit)
  {
    Serial.println(F("Unable to begin:"));
    Serial.println(F("1.Please recheck the connection!"));
    Serial.println(F("2.Please insert the SD card!"));

    dfInit = mp3Obj.begin(Serial2, false, true);
    simDelay(400); // originally 1000ms
  }

  Serial.println(F("DFPlayer Mini online."));

  mp3Obj.setTimeOut(500); // Set serial communictaion time out 500ms

  Serial.println(F("Setting volume"));
  mp3Obj.volume(VOLUME);
  simDelay(100); // DFRobot Timing 9-9-2022
  mp3Obj.EQ(DFPLAYER_EQ_NORMAL);
  mp3Obj.outputDevice(DFPLAYER_DEVICE_SD);
  simDelay(100); // DFRobot Timing 9-9-2022
}

/**
 * Method to play the sound effect for a specified feature
 */
void playSoundEffect(int soundEffect)
{
  mp3Obj.volume(VOLUME);
  simDelay(100); // DFRobot Timing 9-9-2022
  Serial.print(F("Playing sound effect: "));
  Serial.print(soundEffect);
  Serial.print(F("\tVolume: "));
  Serial.println(mp3Obj.readVolume());
  simDelay(100); // DFRobot Timing 9-9-2022
  mp3Obj.play(soundEffect);
  printDetail(mp3Obj.readType(), mp3Obj.read()); // Print the detail message from DFPlayer to handle different errors and states.
}

/**
 * Method to open face plate
 */
void facePlateOpen()
{
  Serial.println(F("Servo Up!"));
  ESP32_ISR_Servos.enableAll();

  // Send data to the servos for movement
  ESP32_ISR_Servos.setPosition(0, SERVO1_OPEN_POS);
  ESP32_ISR_Servos.setPosition(1, SERVO2_OPEN_POS);

  simDelay(1000); // wait doesn't wait long enough for servos to fully complete...
  // Detach so motors don't "idle"
  ESP32_ISR_Servos.disableAll();
  facePlateCurMode = FACEPLATE_OPEN;
}

/**
 * Method to close face plate
 */
void facePlateClose()
{
  Serial.println(F("Servo Down"));
  ESP32_ISR_Servos.enableAll();

  // Send data to the servos for movement
  ESP32_ISR_Servos.setPosition(0, SERVO1_CLOSE_POS);
  ESP32_ISR_Servos.setPosition(1, SERVO2_CLOSE_POS);
  simDelay(1000); // wait doesn't wait long enough for servos to fully complete...

  // Detach so motors don't "idle"
  ESP32_ISR_Servos.disableAll();
  facePlateCurMode = FACEPLATE_CLOSED;
}

/**
 * Set the brightness of the LED eyes
 *
 * @param[out] pwmValue - the PWM value (0-ledEyesMaxPwm) for the LED brightness
 */
void setLedEyes(int pwmValue)
{
  if (battlemode)
  {
    fill_solid(leds2, NUM_LEDS_2, CHSV(0, 255, pwmValue));
  }
  else
  {
    fill_solid(leds2, NUM_LEDS_2, CHSV(160, 255, pwmValue));
  }
  FastLED.show();
  ledEyesCurPwm = pwmValue;
}

/**
 * Method to turn on LED eyes
 */
void ledEyesOn()
{
  Serial.println(F("Turning LED eyes on..."));
  if (battlemode)
  {
    fill_solid(leds2, NUM_LEDS_2, CHSV(0, 255, ledEyesMaxPwm));
  }
  else
  {
    fill_solid(leds2, NUM_LEDS_2, CHSV(160, 255, ledEyesMaxPwm));
  }
  // FastLED.show();
  ledEyesCurMode = LED_EYES_DIM_MODE;
}

/**
 * Method to turn off LED eyes
 */
void ledEyesOff()
{
  Serial.println(F("Turning LED eyes off..."));
  if (battlemode)
  {
    fill_solid(leds2, NUM_LEDS_2, CHSV(0, 255, ledEyesMinPwm));
  }
  else
  {
    fill_solid(leds2, NUM_LEDS_2, CHSV(160, 255, ledEyesMinPwm));
  }
  // FastLED.show();
  ledEyesCurMode = LED_EYES_BRIGHTEN_MODE;
}

/**
 * Method to turn LED eyes on/off
 */
void ledEyesOnOff()
{
  if (ledEyesCurPwm > 0)
  {
    ledEyesOff();
    arcMode(1);
    ledEyesCurPwm = 0;
  }
  else
  {
    ledEyesOn();
    arcMode(1);
    ledEyesCurPwm = ledEyesMaxPwm;
  }
}

void ledEyesDim()
{
  Serial.print(F("."));
  ledEyesCurPwm = ledEyesCurPwm - ledEyesIncrement; // Decrease the brightness
  // Make sure we don't go over the limit
  Serial.print(F("."));
  if (ledEyesCurPwm <= 0)
  {
    ledEyesCurPwm = 0;
  }
}

void ledEyesBrighten()
{

  Serial.print(F("."));
  ledEyesCurPwm = ledEyesCurPwm + ledEyesIncrement; // Increase the brightness
  // Make sure we don't go over the limit
  if (ledEyesCurPwm >= ledEyesMaxPwm)
  {
    ledEyesCurPwm = ledEyesMaxPwm;
  }
}

/**
 * Method to dim or brighten both LED eyes
 */
void ledEyesFade()
{
  if (battlemode)
  {
    fill_solid(leds2, NUM_LEDS_2, CHSV(0, 255, ledEyesMaxPwm));
  }
  else
  {
    fill_solid(leds2, NUM_LEDS_2, CHSV(160, 255, ledEyesMaxPwm));
  }

  if (ledEyesCurPwm == ledEyesMaxPwm)
  {
    ledEyesCurMode = LED_EYES_DIM_MODE;
  }
  else if (ledEyesCurPwm == 0)
  {
    ledEyesCurMode = LED_EYES_BRIGHTEN_MODE;
  }

  if (ledEyesCurMode == LED_EYES_BRIGHTEN_MODE)
  {
    ledEyesBrighten();
  }
  else
  {
    ledEyesDim();
  }

  setLedEyes(ledEyesCurPwm);
  arcMode(1);
  simDelay(200);
  // FastLED.show();
}

void arcMode(uint8_t mode)
{
  if (battlemode)
  {
    fill_solid(leds1, NUM_LEDS_1, CHSV(0, 255, ledEyesMaxPwm));
  }
  else
  {
    fill_solid(leds1, NUM_LEDS_1, CHSV(160, 255, ledEyesMaxPwm));
  }
  // FastLED.show();
}

/**
 * Method to run sequence of sppecial effects when system first starts or sets up
 */
void startupFx()
{
  playSoundEffect(SND_CLOSE);
  simDelay(500); // Timing for Helmet Close Sound and delay to servo closing

  facePlateClose();

  switch (SETUP_FX)
  {
  case EYES_NONE:
    ledEyesOn();
    arcMode(1);
    break;
  case EYES_MOVIE_BLINK:
    movieblink();
    arcMode(1);
    break;
  case EYES_FADE_ON:
    fadeEyesOn();
    arcMode(1);
    break;
  }

  simDelay(500);
  playSoundEffect(SND_JARVIS);
}

/**
 * Method to execute special effects when the faceplate opens
 */
void facePlateOpenFx()
{
  playSoundEffect(SND_OPEN);
  ledEyesOff();
  facePlateOpen();
}

/**
 * Method to execute special effects when the faceplate closes
 */
void facePlateCloseFx()
{
  playSoundEffect(SND_CLOSE);
  simDelay(1200); // Timing for Helmet Close Sound and delay to servo closing
  facePlateClose();
  switch (EYES_FX)
  {
  case EYES_NONE:
    ledEyesOn();
    break;
  case EYES_MOVIE_BLINK:
    movieblink();
    break;
  case EYES_FADE_ON:
    fadeEyesOn();
    break;
  }
}

/**
 * Handle faceplate special effects
 */
void facePlateFx()
{
  if (facePlateCurMode == FACEPLATE_OPEN)
  {
    facePlateCloseFx();
  }
  else
  {
    facePlateOpenFx();
  }
}

/**
 * Event handler for when the primary button is pressed and held
 */
void handlePrimaryButtonLongPress()
{
  ledEyesFade(); // Dim or brighten the LED eyes
}

/**
 * Event handler for when the primary button is pressed multiple times
 */
/*void handlePrimaryButtonMultiPress()
{
  switch (primaryButton.getNumberClicks())
  {
  case 3:
    Serial.println("SND_IAMIRONMAN");
    playSoundEffect(SND_IAMIRONMAN);
    break;

  case 4:
    Serial.println("SND_JARVIS2");
    playSoundEffect(SND_JARVIS2);
    battlemode = !battlemode;

    if (battlemode)
    {
      fill_solid(leds2, NUM_LEDS_2, CHSV(0, 255, ledEyesMaxPwm));
    }
    else
    {
      fill_solid(leds2, NUM_LEDS_2, CHSV(160, 255, ledEyesMaxPwm));
    }
    break;

  default:
    break;
  }
}*/

/**
 * Initializes the primary button for multi-functions
 */
/*void initPrimaryButton()
{
  primaryButton.attachClick(handlePrimaryButtonSingleTap);
  primaryButton.attachDoubleClick(handlePrimaryButtonDoubleTap);
  primaryButton.attachDuringLongPress(handlePrimaryButtonLongPress);
  primaryButton.attachMultiClick(handlePrimaryButtonMultiPress);
}

void initRepulsorButton()
{
  repulsorButton.attachClick(handleRepulsorButtonSingleTap);
  repulsorButton.attachDoubleClick(handleRepulsorButtonDoubleTap);
  repulsorButton.attachMultiClick(handleRepulsorButtonMultiPress);
}*/

/**
 * Method to run special repulsor effects
 */
void repulsorFx(int sound)
{
  playSoundEffect(sound);
  simDelay(600);
  Serial.println("Repulsorflash");
  for (int i = 0; i < ledEyesMaxPwm; i = i + 20)
  {
    fill_solid(leds1, NUM_LEDS_1, CHSV(160, 255, i));
    FastLED.show();
    simDelay(50);
  }
  fill_solid(leds1, NUM_LEDS_1, CHSV(100, 0, 255));
  FastLED.show();
  simDelay(200);
  fill_solid(leds1, NUM_LEDS_1, CHSV(160, 255, ledEyesMaxPwm));
  FastLED.show();
}

void setup()
{
  auto cfg = M5.config();
  M5.begin(cfg);
  while (false == SD.begin(GPIO_NUM_4, SPI, 25000000))
  {
    M5.delay(500);
  }

  { /// custom setting
    auto spk_cfg = M5.Speaker.config();
    spk_cfg.sample_rate = 96000;
    M5.Speaker.config(spk_cfg);
  }

  M5.Speaker.begin();
  M5.Speaker.setVolume(0);
  init_player(); // initializes the sound player

  M5.Display.setEpdMode(epd_mode_t::epd_fastest);
  M5.Display.fillScreen(TFT_BLACK);
  M5.Display.setCursor(30, 10);
  M5.Display.print("J.A.R.V.I.S.");

  Serial.begin(115200);
  simDelay(2000); // Give the serial service time to initialize
  Serial.print(F("Initializing Iron Man Servo version: "));
  Serial.println(VERSION);

  FastLED.addLeds<WS2812B, DATA_PIN_1, GRB>(leds1, NUM_LEDS_1);
  FastLED.addLeds<WS2812B, DATA_PIN_2, GRB>(leds2, NUM_LEDS_2);
  fill_solid(leds1, NUM_LEDS_1, CHSV(96, 255, ledEyesMaxPwm)); // Arcstart
  fill_solid(leds2, NUM_LEDS_2, CHSV(0, 255, ledEyesMinPwm));  // Eyestart
  FastLED.show();

  ESP32_ISR_Servos.useTimer(USE_ESP32_TIMER_NO);
  servoIndex1 = ESP32_ISR_Servos.setupServo(SERVO1_PIN, PWM_LOW, PWM_HIGH);
  servoIndex2 = ESP32_ISR_Servos.setupServo(SERVO2_PIN, PWM_LOW, PWM_HIGH);

  if (servoIndex1 != -1)
  {
    Serial.println(F("Setup Servo1 OK"));
  }
  else
  {
    Serial.println(F("Setup Servo1 failed"));
  }
  if (servoIndex2 != -1)
  {
    Serial.println(F("Setup Servo2 OK"));
  }
  else
  {
    Serial.println(F("Setup Servo2 failed"));
  }

  ESP32_ISR_Servos.setPosition(0, SERVO1_OPEN_POS);
  ESP32_ISR_Servos.setPosition(1, SERVO2_OPEN_POS);

  startupFx(); // Run the initial features
}

/**
 * Main program exeucution
 * This method will run perpetually on the board
 */
void loop()
{
  if (M5.BtnA.wasClicked())
  {
    Serial.println(F("Click A"));
    facePlateFx();
  }
  if (M5.BtnA.wasDoubleClicked())
  {
    Serial.println(F("DblClick A - free"));
  }
  if (M5.BtnA.pressedFor(1000))
  {
    Serial.println(F("Longpress A - free"));
  }

  if (M5.BtnB.wasClicked())
  {
    Serial.println("Click B - free");
  }
  if (M5.BtnB.wasDoubleClicked())
  {
    Serial.println(F("DblClick B - free"));
  }
  if (M5.BtnB.pressedFor(1000))
  {
    Serial.println("Longpress B");
    ledEyesOnOff();
  }

  if (M5.BtnC.wasClicked())
  {
    Serial.println("Click C");
    repulsorFx(4);
  }
  if (M5.BtnC.wasDoubleClicked())
  {
    Serial.println(F("DblClick C - free"));
  }
  if (M5.BtnC.pressedFor(1000))
  {
    Serial.println("LongPress C");
    playSoundEffect(SND_JARVIS2);
    battlemode = !battlemode;

    if (battlemode)
    {
      fill_solid(leds2, NUM_LEDS_2, CHSV(0, 255, ledEyesMaxPwm));
    }
    else
    {
      fill_solid(leds2, NUM_LEDS_2, CHSV(160, 255, ledEyesMaxPwm));
    }
  }

  FastLED.show();
  M5.update();
}

/**
 * Method to output any issues with the DFPlayer
 */
void printDetail(uint8_t type, int value)
{
  switch (type)
  {
  case TimeOut:
    Serial.println(F("Time Out!"));
    break;
  case WrongStack:
    Serial.println(F("Stack Wrong!"));
    break;
  case DFPlayerCardInserted:
    Serial.println(F("Card Inserted!"));
    break;
  case DFPlayerCardRemoved:
    Serial.println(F("Card Removed!"));
    break;
  case DFPlayerCardOnline:
    Serial.println(F("Card Online!"));
    break;
  case DFPlayerPlayFinished:
    Serial.print(F("Number:"));
    Serial.print(value);
    Serial.println(F(" Play Finished!"));
    break;
  case DFPlayerError:
    Serial.print(F("DFPlayerError:"));
    switch (value)
    {
    case Busy:
      Serial.println(F("Card not found"));
      break;
    case Sleeping:
      Serial.println(F("Sleeping"));
      break;
    case SerialWrongStack:
      Serial.println(F("Get Wrong Stack"));
      break;
    case CheckSumNotMatch:
      Serial.println(F("Check Sum Not Match"));
      break;
    case FileIndexOut:
      Serial.println(F("File Index Out of Bound"));
      break;
    case FileMismatch:
      Serial.println(F("Cannot Find File"));
      break;
    case Advertise:
      Serial.println(F("In Advertise"));
      break;
    default:
      break;
    }
    break;
  default:
    break;
  }
}