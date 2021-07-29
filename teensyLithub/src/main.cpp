// Reacts to sounds using LEDs

#include <WS2812Serial.h>
#define USE_WS2812SERIAL
#include <FastLED.h>

// How many leds in your strip?
#define NUM_LEDS_1 800
#define NUM_LEDS_2 800
#define NUM_LEDS_3 800

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

int Front_Left_Bottom = 0;
int Front_Left_Top = 163;
int Rear_Left_Bottom = 600;
int Rear_Left_TopL = Rear_Left_Bottom - 163 + 67;
CRGB leds_1[NUM_LEDS_1]; // Left wall

int Front_Right_Bottom = 340;
int Front_Right_Top = 340 - 163;
CRGB leds_2[NUM_LEDS_2]; // Center wall

int Rear_Right_Bottom = 506;
int Rear_Right_TopR = 506 - 149; // 357
int Rear_Right_TopL = 507;
CRGB leds_3[NUM_LEDS_3]; // Right / Back wall

int ledNewOn = NUM_LEDS_1;
int ledOldOn = 0;
const int sampleWindow = 50; // Sample window width in mS (50 mS = 20Hz)
int microphonePin = A8;    // select the input pin for the potentiometer
unsigned int sample;

uint32_t lastColorChange;

typedef void (*SimplePatternList[])();

uint8_t gCurrentPatternNumber = 0; // Index number of which pattern is current
uint8_t gHue = 100; // rotating "base color" used by many of the patterns

void reactive();
void movingRainbow();

void setup() {
  Serial.begin(9600); // open the serial port at 9600 bps:
  LEDS.addLeds<WS2812SERIAL, DATA_PIN_1, BGR>(leds_1, NUM_LEDS_1);
  LEDS.addLeds<WS2812SERIAL, DATA_PIN_2, BGR>(leds_2, NUM_LEDS_2);
  LEDS.addLeds<WS2812SERIAL, DATA_PIN_3, BGR>(leds_3, NUM_LEDS_3);
  LEDS.setBrightness(BRIGHTNESS);
}

void loop() {
  // reactive();
  // movingRainbow();

  fill_rainbow(leds_3 + 640, 20, 100, 50);
  FastLED.show();
}
//**********************************************************************
// Helper method - movingRainbow()
// Reacts to sounds
//**********************************************************************
void movingRainbow() {
  // for (int i = 0; i < 800; ++i) {
  //   leds_1[i] = CHSV(gHue + i*5, 255, 192);
  //   leds_2[i] = CHSV(gHue + i*5, 255, 192);
  //   leds_3[i] = CHSV(gHue + i*5, 255, 192);
  // }
  fill_rainbow(leds_1, NUM_LEDS_1, gHue, 1);
  fill_rainbow(leds_2, NUM_LEDS_2, gHue, 1);
  fill_rainbow(leds_3, NUM_LEDS_3, gHue, 1);
  gHue += 1;
  FastLED.show();
}

//**********************************************************************
// Helper method - reactive()
// Reacts to sounds
//**********************************************************************
void reactive() {
    //  **********************************************************************
  //   Read from microphone
  //  **********************************************************************
  unsigned long startMillis = millis(); // Start of sample window
  unsigned int peakToPeak = 0;   // peak-to-peak level

  unsigned int signalMax = 0;
  unsigned int signalMin = 1024;

  // collect data for sampleWindow mS
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

  ledNewOn = ceil(volts * 37.5) + 50;

  // Cut-off at NUM_LEDS
  if (ledNewOn > NUM_LEDS_1) {
    ledNewOn = NUM_LEDS_1;
  }

  for (int i = 0; i < NUM_LEDS_1; ++i) {
    leds_1[i] = CRGB::Black;
    leds_2[i] = CRGB::Black;
    leds_3[i] = CRGB::Black;
  }

  if (ledNewOn > 163) {
    ledNewOn = 163;
  }

   for (int i = 0; i < ledNewOn; ++i) {
     leds_1[Front_Left_Bottom + i] = CHSV(i + gHue, 255, 192);
     leds_2[Front_Right_Bottom - i] = CHSV(i + gHue, 255, 192);
     leds_3[min(Rear_Right_Bottom, Rear_Right_Bottom - i + 14)] = CHSV(i + gHue, 255, 192);
     leds_1[Rear_Left_Bottom - i + 67] = CHSV(i + gHue, 255, 192);
   }

  // Front Rainbow Bar
  for (int i = 0; i < (Front_Right_Bottom - 163); ++i) {
    leds_2[i] = CHSV(gHue, 255, 192);
  }

  // Left Rainbow Bar
  for (int i = Front_Left_Top; i < Rear_Left_TopL; ++i) {
    leds_1[i] = CHSV(gHue, 255, 192);
  }

  // Right Rainbow Bar
  for (int i = 0; i < Rear_Right_TopR; ++i) {
    leds_3[i] = CHSV(gHue, 255, 192);
  }

    // Back Rainbow Bar
  for (int i = Rear_Right_TopL; i < 750; ++i) {
    leds_3[i] = CHSV(gHue, 255, 192);
  }

  ledOldOn = ledNewOn;
  gHue = gHue + 1;

  Serial.println(millis() - lastColorChange);

  // // Color shift on bass drop
  // if (millis() - lastColorChange > 100 && ledNewOn > 120) {
  //   lastColorChange = millis();
  //   gHue += 100;
  // }

  FastLED.show();
}