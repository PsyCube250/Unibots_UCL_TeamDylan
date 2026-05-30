#include <Adafruit_NeoPixel.h>

#define LED_PIN 48
#define NUM_LEDS 1

Adafruit_NeoPixel strip(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);

void setup() {
  Serial.begin(115200);
  while (!Serial);
  strip.begin();
  strip.show();
  Serial.println("BOOT COMPLETE");
}

void loop() {
  for (int i = 0; i < 255; i++) {
    for (int j = 0; j < 255; j++) {
      for (int k = 0; k < 255; k++) {
        strip.setPixelColor(0, strip.Color(i, j, k));
        strip.show();   // ✅ Actually push the colour to the LED
        delay(1);
      }
    }
  }
}