#pragma once

//-----------LDR Configuration -------------------
constexpr int LDR_BRIGHT_ADC_MIN = 100;     // Example: ADC reading for very bright light
constexpr int LDR_DARK_ADC_MAX = 3200;      // Example: ADC reading for very dark light+
constexpr int MIN_BRIGHTNESS_PERCENT = 10;  // Minimum duty cycle percentage
constexpr int MAX_BRIGHTNESS_PERCENT = 100; // Maximum duty cycle percentage


//----------assign GPIO numbers---------------------
constexpr int SDA1_pin = 4;
constexpr int SCL1_pin = 15;
constexpr int SDA2_pin = 21;
constexpr int SCL2_pin = 22;
constexpr int INT2 = 19;              //interrupt pin on second I2C bus (not used in this version)
constexpr int led_pin = 21;
constexpr int HV_enable_pin = 23;     //active high
constexpr int LDR_pin = 33;           //LDR connected to pin 33 (GPIO2)

