#include "ldr.h"
#include "config.h"

static uint8_t ldrValue=0;
int current_brightness_percent = MAX_BRIGHTNESS_PERCENT;

uint8_t readLDR()
{
    return ldrValue = analogRead(LDR_pin); 
}

uint8_t MapBrightness() {
  // Maps the raw ADC value (from LDR_BRIGHT_ADC_MIN to LDR_DARK_ADC_MAX)
  // to the desired brightness percentage (from MAX_BRIGHTNESS_PERCENT to MIN_BRIGHTNESS_PERCENT).
  // Remember to calibrate LDR_BRIGHT_ADC_MIN and LDR_DARK_ADC_MAX for your specific setup.

  current_brightness_percent = map(readLDR(), 
                                   LDR_BRIGHT_ADC_MIN,  // Input lower bound (ADC val for bright)
                                   LDR_DARK_ADC_MAX,    // Input upper bound (ADC val for dark)
                                   MAX_BRIGHTNESS_PERCENT, // Output lower bound (for bright)
                                   MIN_BRIGHTNESS_PERCENT  // Output upper bound (for dark)
                                  );

  // Ensures the brightness value stays within the defined min/max percentage.
  current_brightness_percent = constrain(current_brightness_percent, 
                                         MIN_BRIGHTNESS_PERCENT, 
                                         MAX_BRIGHTNESS_PERCENT
                                        );

  Serial.print("LDR Raw: ");
  Serial.print(ldrValue);
  Serial.print(", Brightness %: ");
  Serial.println(current_brightness_percent);

  return current_brightness_percent;
}