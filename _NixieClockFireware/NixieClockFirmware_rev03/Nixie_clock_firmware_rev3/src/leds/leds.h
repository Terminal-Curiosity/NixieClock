#pragma once

//#include <Arduino.h>
#include <Adafruit_NeoPixel.h>

// void showColor(uint32_t color, int LEDnum = 0) {
//   strip.setPixelColor(LEDnum, color);  // Set the first (only) LED
//   strip.show();
// }


// void rainbowFade() {
//   for (int j = 0; j < 256; j++) {
//     for (int i = 0; i < strip.numPixels(); i++) {
//       strip.setPixelColor(i, wheel((i + j) & 255));
//     }
//     strip.show();
//     delay(20);
//   }
// }


// // Helper function to generate rainbow colors
// uint32_t wheel(byte pos) {
//   if (pos < 85) 
//   return strip.Color(pos * 3, 255 - pos * 3, 0);
//   else if (pos < 170) {
//     pos -= 85;
//     return strip.Color(255 - pos * 3, 0, pos * 3);
//   } else {
//     pos -= 170;
//     return strip.Color(0, pos * 3, 255 - pos * 3);
//   }
// }


uint32_t ColorHSV(float h, float s, float v);
// 