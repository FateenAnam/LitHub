#include <FastLED.h>

// How many leds in your strip?
#define NUM_LEDS_1 600
#define NUM_LEDS_2 600
#define NUM_LEDS_3 600
#define DATA_PIN_1 2
#define DATA_PIN_2 6
#define DATA_PIN_3 7
#define BRIGHTNESS 100
CRGB leds_1[NUM_LEDS_1];
CRGB leds_2[NUM_LEDS_2];
CRGB leds_3[NUM_LEDS_3];
int ledNewOn = NUM_LEDS_1;
int ledOldOn = 0;
const int sampleWindow = 50; // Sample window width in mS (50 mS = 20Hz)
int microphonePin = A9;    // select the input pin for the potentiometer
unsigned int sample;

typedef void (*SimplePatternList[])();

uint8_t gCurrentPatternNumber = 0; // Index number of which pattern is current
uint8_t gHue = 100; // rotating "base color" used by many of the patterns

void setup() {
    Serial.begin(9600); // open the serial port at 9600 bps:
  FastLED.addLeds<NEOPIXEL, DATA_PIN_1>(leds_1, NUM_LEDS_1);
  //  FastLED.addLeds<NEOPIXEL, DATA_PIN_2>(leds_2, NUM_LEDS_2);
  //  FastLED.addLeds<NEOPIXEL, DATA_PIN_3>(leds_3, NUM_LEDS_3);
  FastLED.setBrightness(BRIGHTNESS);
  pinMode(microphonePin, INPUT);
}

void loop() {
  //**********************************************************************
  // Read from microphone
  //**********************************************************************
  unsigned long startMillis = millis(); // Start of sample window
  unsigned int peakToPeak = 0;   // peak-to-peak level

  unsigned int signalMax = 0;
  unsigned int signalMin = 1024;

  // collect data for 50 mS
  while (millis() - startMillis < sampleWindow)
  {
    sample = analogRead(microphonePin);
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

//  Serial.println(millis() - startMillis - sampleWindow);
  Serial.println(ledNewOn);

  //  for (int i = 0; i < NUM_LEDS_1; ++i) {
  //    leds_1[i] = CRGB::White;
  //  }
  //  digitalWrite(0, HIGH);
}

//**********************************************************************
// Helper method - reactive()
// Reacts to sounds
//**********************************************************************
void reactive() {

  //  int ledsToTurnOff = ledOldOn - ledNewOn;
  //  if (ledsToTurnOff > 0) { // If we need to turn off LEDs
  //    // Only turn off what we need to
  //    for (int i = ledOldOn; i > ledNewOn; --i) {
  //      leds_1[i] = CRGB::Black;
  //      //      leds_2[i] = CRGB::Black;
  //      //      leds_3[i] = CRGB::Black;
  //    }
  //  }

  for (int i = 0; i < NUM_LEDS_1; ++i) {
    leds_1[i] = CRGB::Black;
  }

  for (int i = 0; i < ledNewOn; ++i) {
    leds_1[i] = CHSV(i + gHue, 255, 192);
    //    leds_2[i] = CHSV(newVal, 255, 192);
    //    leds_3[i] = CHSV(newVal, 255, 192);
    //    leds[i] = CRGB::White; // set to full white!
  }

  ledOldOn = ledNewOn;
  gHue = gHue + 5;
  FastLED.show();
}
