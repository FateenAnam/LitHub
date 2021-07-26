#include <FastLED.h>

// How many leds in your strip?
#define NUM_LEDS 600
#define DATA_PIN 5
#define BRIGHTNESS        100
const int sampleWindow = 50; // Sample window width in mS (50 mS = 20Hz)
CRGB leds[NUM_LEDS];
int sensorPin = A5;    // select the input pin for the potentiometer
int ledOn = NUM_LEDS;

typedef void (*SimplePatternList[])();

uint8_t gCurrentPatternNumber = 0; // Index number of which pattern is current
uint8_t gHue = 100; // rotating "base color" used by many of the patterns

void setup() {
  Serial.begin(9600); // open the serial port at 9600 bps:
  FastLED.addLeds<NEOPIXEL, DATA_PIN>(leds, NUM_LEDS);
  FastLED.setBrightness(BRIGHTNESS);
}

void loop() {
  // Call the reactive to sounds function
  rainbow();
}

//**********************************************************************
// Helper method - Sets color to a rainbow
//**********************************************************************
void rainbow() {
  for (int i = 0; i < NUM_LEDS; ++i) {
    leds[i] = CRGB::Black;
  }

  // Cut-off at NUM_LEDS
  if (ledOn > NUM_LEDS) {
    ledOn = NUM_LEDS;
  }

  for (int i = 0; i < ledOn; ++i) {
    leds[i] = CHSV(i + gHue, 255, 192);
  }
  ++gHue;
  FastLED.show();
}
