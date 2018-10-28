#include <Arduino.h>
#include <FastLED.h>

#define NUM_LEDS 68
#define DATA_PIN 6


CRGB leds[NUM_LEDS];

void setup() { 
   FastLED.addLeds<WS2812, DATA_PIN, GRB>(leds, NUM_LEDS);
   FastLED.setBrightness(84);
   Serial.begin(115200);
}

void loop() { 
//leds[0] = CRGB::Red; 
  
    if (Serial.available() > 0)
    {
      int incomingByte = Serial.read();
      
      if (incomingByte == 'w')
      {
        // Serial.println("WARNING");
        FastLED.setBrightness(100);
         for (int i=0; i<NUM_LEDS; ++i)
         {
            leds[i] = CRGB::Yellow;
         }
      }
      
      if (incomingByte == 'd')
      {
         FastLED.setBrightness(100);
//         Serial.println("DANGER");
         for (int i=0; i<NUM_LEDS; ++i)
         {
              leds[i] = CRGB::Red;
         }
      }

      if (incomingByte == 'g')
      {
        FastLED.setBrightness(40);
//         Serial.println("DANGER");
         for (int i=0; i<NUM_LEDS; ++i)
         {
              leds[i] = CRGB::Green;
         }
      }
      
    
    }

    FastLED.show(); 
    delay(30); 
}


