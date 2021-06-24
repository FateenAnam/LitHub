#include <FastLED.h>

// How many leds in your strip?
#define NUM_LEDS 300
#define DATA_PIN 2
#define BRIGHTNESS          1
#define FRAMES_PER_SECOND  120
#define MIDDLE_LED 111
const int sampleWindow = 50; // Sample window width in mS (50 mS = 20Hz)
unsigned int sample;
bool beat = false;
CRGB leds[NUM_LEDS];
int sensorPin = A5;    // select the input pin for the potentiometer
int ledPin = 13;      // select the pin for the LED
int sensorValue = 0;  // variable to store the value coming from the sensor
int ledOn = 0 ;
unsigned int globL = 0;
unsigned int globR = 0;
int ti = 0;
uint32_t timebase = 0;
uint32_t phase = 49151;
bool finished = true;
bool running = false;
int blueToothVal = 0;           //value sent over via bluetooth
int Mode = 2;    //stores mode
int Col = 6;    //stores Color
int trueCol;

typedef void (*SimplePatternList[])();

uint8_t gCurrentPatternNumber = 0; // Index number of which pattern is current
uint8_t gHue = 100; // rotating "base color" used by many of the patterns


void setup() {
  Serial.begin(9600); // open the serial port at 9600 bps:
  FastLED.addLeds<WS2812B, DATA_PIN>(leds, 300);
  FastLED.setBrightness(BRIGHTNESS);

  for (int i = 0; i < NUM_LEDS; ++i) {
    leds[i] = CRGB::White;
  }
}

void loop() {
}
