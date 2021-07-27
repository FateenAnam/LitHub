// Reacts to sounds using LEDs

#include <WS2812Serial.h>
#define USE_WS2812SERIAL
#include <FastLED.h>

// How many leds in your strip?
#define NUM_LEDS_1 600
#define NUM_LEDS_2 600
#define NUM_LEDS_3 600

// Usable pins:
//   Teensy LC:   1, 4, 5, 24
//   Teensy 3.2:  1, 5, 8, 10, 31   (overclock to 120 MHz for pin 8)
//   Teensy 3.5:  1, 5, 8, 10, 26, 32, 33, 48
//   Teensy 3.6:  1, 5, 8, 10, 26, 32, 33
//   Teensy 4.0:  1, 8, 14, 17, 20, 24, 29, 39
//   Teensy 4.1:  1, 8, 14, 17, 20, 24, 29, 35, 47, 53

#define DATA_PIN_1 1
#define DATA_PIN_2 8
#define DATA_PIN_3 14

#define BRIGHTNESS 100
CRGB leds_1[NUM_LEDS_1];
CRGB leds_2[NUM_LEDS_2];
CRGB leds_3[NUM_LEDS_3];
int ledNewOn = NUM_LEDS_1;
int ledOldOn = 0;
const int sampleWindow = 50; // Sample window width in mS (50 mS = 20Hz)
int microphonePin = A8;    // select the input pin for the potentiometer
unsigned int sample;

typedef void (*SimplePatternList[])();

uint8_t gCurrentPatternNumber = 0; // Index number of which pattern is current
uint8_t gHue = 100; // rotating "base color" used by many of the patterns

void setup() {
  Serial.begin(9600); // open the serial port at 9600 bps:
  LEDS.addLeds<WS2812SERIAL, DATA_PIN_1, RGB>(leds_1, NUM_LEDS_1);
  LEDS.addLeds<WS2812SERIAL, DATA_PIN_2, RGB>(leds_2, NUM_LEDS_2);
  LEDS.addLeds<WS2812SERIAL, DATA_PIN_3, RGB>(leds_3, NUM_LEDS_3);
  LEDS.setBrightness(BRIGHTNESS);
}

void loop() {
  //  **********************************************************************
  //   Read from microphone
  //  **********************************************************************
  unsigned long startMillis = millis(); // Start of sample window
  unsigned int peakToPeak = 0;   // peak-to-peak level

  unsigned int signalMax = 0;
  unsigned int signalMin = 1024;

  // collect data for 50 mS
  while (millis() - startMillis < sampleWindow)
  {
    sample = analogRead(microphonePin);
    //    Serial.println(analogRead(microphonePin));
    if (sample < 1024)  // Sanitize input
    {
      if (sample > signalMax) // Save just the max levels
      {
        signalMax = sample;
      }
      else if (sample < signalMin) // Save just the min levels
      {
        signalMin = sample;
      }
    }
  }

  peakToPeak = signalMax - signalMin;  // max - min = peak-peak amplitude

  double volts = (peakToPeak * 5.0) / 1024;  // convert to volts

  ledNewOn = ceil(volts * 50);



  //  Serial.println(peakToPeak);

  // Cut-off at NUM_LEDS
  if (ledNewOn > NUM_LEDS_1) {
    ledNewOn = NUM_LEDS_1;
  }

  // Call the reactive to sounds function
  reactive();

  Serial.println(millis() - startMillis);
  //  Serial.println(ledNewOn);
  //  for (int i = 0; i < NUM_LEDS_1; ++i) {
  //    leds_1[i] = CRGB::Red;
  //  }
  //  FastLED.show();
}

//**********************************************************************
// Helper method - reactive()
// Reacts to sounds
//**********************************************************************
void reactive() {
  for (int i = 0; i < NUM_LEDS_1; ++i) {
    leds_1[i] = CRGB::Black;
    leds_2[i] = CRGB::Black;
    leds_3[i] = CRGB::Black;
  }

  for (int i = 0; i < ledNewOn; ++i) {
    leds_1[i] = CHSV(i + gHue, 255, 192);
    leds_2[i] = CHSV(i + gHue, 255, 192);
    leds_3[i] = CHSV(i + gHue, 255, 192);
    //    leds_1[i] = CRGB::White; // set to full white!
  }

  ledOldOn = ledNewOn;
  gHue = gHue + 1;
  FastLED.show();
}
