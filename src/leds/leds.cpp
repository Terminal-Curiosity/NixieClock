#include "leds.h"

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


uint32_t ColorHSV(float h, float s, float v) {
  float c = v * s;
  float x = c * (1 - fabs(fmod(h / 60.0, 2) - 1));
  float m = v - c;

  float r1, g1, b1;
  if      (h < 60)  { r1 = c; g1 = x; b1 = 0; }
  else if (h < 120) { r1 = x; g1 = c; b1 = 0; }
  else if (h < 180) { r1 = 0; g1 = c; b1 = x; }
  else if (h < 240) { r1 = 0; g1 = x; b1 = c; }
  else if (h < 300) { r1 = x; g1 = 0; b1 = c; }
  else              { r1 = c; g1 = 0; b1 = x; }

  uint8_t r = (r1 + m) * 255;
  uint8_t g = (g1 + m) * 255;
  uint8_t b = (b1 + m) * 255;

  return ((uint32_t)r << 16) | ((uint32_t)g << 8) | b;
}