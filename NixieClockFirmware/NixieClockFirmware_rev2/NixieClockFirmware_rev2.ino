#include <Wire.h>
#include <Adafruit_NeoPixel.h>
#include <esp_timer.h> 

//select GPIO numbers
#define SDA_pin 4
#define SCL_pin 15
#define led_pin 32

//-----------LDR Configuration -------------------
const int LDR_PIN = 33;             // LDR connected to GPIO 2
const int LDR_BRIGHT_ADC_MIN = 100;  // Example: ADC reading for very bright light
const int LDR_DARK_ADC_MAX = 3200;  // Example: ADC reading for very dark light

// --- PWM Configuration ---
const long NIXIE_TUBE_REFRESH_TIME = 1000; // 500us refresh period = 2kHz nixie tube refresh rate to protect against cathode poisoning
const int MIN_BRIGHTNESS_PERCENT = 10; // Minimum duty cycle percentage
const int MAX_BRIGHTNESS_PERCENT = 100; // Maximum duty cycle percentage

// --- Nixie Tube & PCF8574 Configuration ---
const byte PCF8574_ADDRESS_1 = 0x24; // Address of your first PCF8574
const byte PCF8574_ADDRESS_2 = 0x20; // Address of your second PCF8574
const byte K155ID1_OFF_CODE = 0x0F; // BCD 1111 (an invalid input that turns off all cathodes on K155ID1)
const int digitCode[] = {8,1,10,11,2,12,4,0,9,3}; //tube pin to digit mapping

// --- Global Variables for PWM and Display (volatile for ISR access) ---
volatile int current_brightness_percent = MAX_BRIGHTNESS_PERCENT; 
volatile byte current_digit_bcd_tube1 = 0x00; // Example: Display digit '0' for tube 1
volatile byte current_digit_bcd_tube2 = 0x02; // Example: Display digit '2' for tube 2
volatile byte current_digit_bcd_tube3 = 0x03; // Example: Display digit '3' for tube 3
volatile byte current_digit_bcd_tube4 = 0x08; // Example: Display digit '4' for tube 4

// --- Timer Handles ---
esp_timer_handle_t pwm_on_timer_handle;  // Periodic timer (starts ON pulse)
esp_timer_handle_t pwm_off_timer_handle; // One-shot timer (ends ON pulse)
esp_timer_handle_t one_sec_tick_timer_handle; // One second timer to increment clock time

// --- Function Prototypes (IRAM_ATTR (instruction RAM atribute) for ISRs for reliability) ---
// IRAM_ATTR places the ISR functions in RAM instead of flash because its much faster and is not affected if flash memory is already in use elsewhere.
void IRAM_ATTR pwm_on_timer_callback(void* arg); 
void IRAM_ATTR pwm_off_timer_callback(void* arg);
void IRAM_ATTR one_sec_tick_timer_callback(void* arg);
void readLDRAndMapBrightness();
void writePcf8574(byte address, byte data);
void write1byte(byte, byte);
uint32_t ColorHSV(float h, float s, float v);

Adafruit_NeoPixel strip = Adafruit_NeoPixel(4, led_pin, NEO_GRB + NEO_KHZ800);
bool led_update = 0;


void setup() {
  Wire.begin(SDA_pin, SCL_pin);  //initialise I2C with pins specified
  Wire.setClock(100000); // set I2C speed (default is 100kHz)
  pinMode(led_pin,OUTPUT);
  analogSetAttenuation(ADC_11db); // sets attenuation of LDR input (otherwise it will be 0-1V)

  strip.begin();         // Initialize NeoPixel strip
  strip.setBrightness(50);  // brightness from 0–255
  strip.show();          // Turn off all LEDs at start
  
  Wire.beginTransmission(PCF8574_ADDRESS_1);
  Wire.write(0xFF);
  Wire.endTransmission();
  delay(5);
  Serial.begin(115200);


  // --- PWM Timer Setup ---  
//  // 1. Create the periodic timer for starting the ON pulse (pwm_on_timer_handle)
//  //create a data object of type "esp_timer_create_args_t". the object is named "pwm_on_timer_args". it contains a pointer to "pwm_on_timer_callback", which is the ISR function. the timer is named "pwm_on_timer".
//  esp_timer_create_args_t pwm_on_timer_args = {
//      .callback = &pwm_on_timer_callback, // Function called when timer fires
//      .name = "pwm_on_timer"             
//  };
//  esp_timer_create(&pwm_on_timer_args, &pwm_on_timer_handle);
//
//  // 2. Create the one-shot timer for ending the ON pulse (pwm_off_timer_handle)
//  esp_timer_create_args_t pwm_off_timer_args = {
//      .callback = &pwm_off_timer_callback, 
//      .name = "pwm_off_timer"            
//  };
//  esp_timer_create(&pwm_off_timer_args, &pwm_off_timer_handle);

  // 3. Create the periodic timer for counting 1 second pulses
  esp_timer_create_args_t one_sec_tick_timer_args = {
      .callback = &one_sec_tick_timer_callback, 
      .name = "one_sec_tick_timer"            
  };
  esp_timer_create(&one_sec_tick_timer_args, &one_sec_tick_timer_handle);

  // Start the main periodic timer; it will fire every PWM_PERIOD_US (e.g., 5ms)
  //esp_timer_start_periodic(pwm_on_timer_handle, NIXIE_TUBE_REFRESH_TIME); // turned off PWM timer as it turns out its generally better to run tubes at DC 
  esp_timer_start_periodic(one_sec_tick_timer_handle, 1000000);
  Serial.println("PWM timers initialized and started.");
 
}



void loop() {
  uint16_t hue = 0;

  readLDRAndMapBrightness();
  delay(10);

  if(led_update)
  {
  hue = 36 * current_digit_bcd_tube1;
  uint32_t color = ColorHSV(hue, 1.0, 1.0);

  uint32_t led_colour_val = 0x000000FF << current_digit_bcd_tube1;
  strip.setPixelColor(0, color);  // Set the first (only) LED
  strip.setPixelColor(1, color);
  strip.setPixelColor(2, color);
  strip.setPixelColor(3, color);
  
  strip.show();
  led_update = 0;
  }
  

//  for(int i = 0; i < 256; i++)
//   {
//   Wire.beginTransmission(PCF8574_ADDRESS_1);
//   Wire.write(i << 4);
//   Wire.endTransmission();
//   delay(100);
//   }

  
   //digitCycle(1000);
  // digitCycle(100);

  //rainbowFade();
  //delay(500);

  // //testing dimming circuit
  // //writeDigit(1);
  // write1byte(IO1_addr,1);
  // delay(9);

   //write1byte(IO1_addr,3);
  // delay(1);

  //delay(1000);
}

void readLDRAndMapBrightness() {
  int ldrValue = analogRead(LDR_PIN);

  // Maps the raw ADC value (from LDR_BRIGHT_ADC_MIN to LDR_DARK_ADC_MAX)
  // to the desired brightness percentage (from MAX_BRIGHTNESS_PERCENT to MIN_BRIGHTNESS_PERCENT).
  // Remember to calibrate LDR_BRIGHT_ADC_MIN and LDR_DARK_ADC_MAX for your specific setup.
  current_brightness_percent = map(ldrValue, 
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
}

// --- I2C Write Helper Function ---
// IMPORTANT WARNING: Calling Wire library functions directly from ISRs
// (like the timer callbacks below) is generally NOT recommended in robust
// embedded systems. It can cause issues if the library blocks or uses mutexes.
// For short, single-byte I2C writes on ESP32, it *often* works in hobby projects,
// especially with IRAM_ATTR. However, if you experience instability or crashes,
// you would need to implement a flag-based system where ISRs set flags, and
// actual I2C communication happens in the main loop() or a FreeRTOS task.
void writePcf8574(byte address, byte data) {
  Wire.beginTransmission(address); 
  Wire.write(data);                 
  Wire.endTransmission();           
}


//turns out its better to run the tubes at 2mADC and not bother with PWM. low current and repeated strikes will harm the tubes more over the long term. 
//change the code to never activate the PWM off timer

// --- PWM Timer Callback for ON Pulse (Runs Periodically) ---
// This function is executed by the periodic timer at the start of each PWM cycle.
//void IRAM_ATTR pwm_on_timer_callback(void* arg) {
//  // Calculate the duration (in microseconds) for which the tubes should be ON during this cycle.
//  long on_time_us = (long)((float)current_brightness_percent / 100.0 * NIXIE_TUBE_REFRESH_TIME);
//
//  // --- PCF8574 Data Construction and Writing (ADAPT THIS SECTION!) ---
//  // This is the most crucial part for your specific hardware.
//  // One PCF8574 (8 pins) can drive TWO K155ID1s (4 input pins each).
//  // Assuming PCF8574_ADDRESS_1 controls K155ID1 for tube 1 (pins P0-P3) and tube 2 (pins P4-P7).
//  // Similarly for PCF8574_ADDRESS_2 and tubes 3 & 4.
//  // Combine the BCD values for two tubes into a single byte for each PCF8574.
//  byte data_for_pcf1 = (digitCode[current_digit_bcd_tube2] << 4) | digitCode[current_digit_bcd_tube1];
//
//  writePcf8574(PCF8574_ADDRESS_1, data_for_pcf1);
//
//  byte data_for_pcf2 = (current_digit_bcd_tube4 << 4) | current_digit_bcd_tube3;
//  //writePcf8574(PCF8574_ADDRESS_2, data_for_pcf2);
//  // --- END ADAPTATION SECTION ---
//
//  // If brightness is 100%, the tubes stay ON for the entire PWM_PERIOD_US.
//  // In this case, we don't need to schedule the 'off' timer, as they will be
//  // turned on again by the next periodic call anyway.
//  if (current_brightness_percent < MAX_BRIGHTNESS_PERCENT) {
//    // If on_time_us is 0 (0% duty cycle), immediately turn off.
//    // This handles the edge case where brightness is set to effectively 0.
//    if (on_time_us == 0) {
//      byte data_for_pcf_off = (K155ID1_OFF_CODE << 4) | K155ID1_OFF_CODE; 
//      writePcf8574(PCF8574_ADDRESS_1, data_for_pcf_off);
//      //writePcf8574(PCF8574_ADDRESS_2, data_for_pcf_off);
//    } else {
//      if( on_time_us < 50)
//      {on_time_us = 50;}
//      // Start the one-shot timer to turn OFF the tubes after 'on_time_us' has passed.
//      esp_timer_start_once(pwm_off_timer_handle, on_time_us);
//    }
//  }
//}


//// --- PWM Timer Callback for OFF Pulse (Runs Once) ---
//// This function is executed by the one-shot timer after the 'ON' duration.
//void IRAM_ATTR pwm_off_timer_callback(void* arg) {
//  // Construct the data byte to send to the PCF8574s to turn the tubes OFF.
//  // This combines the K155ID1_OFF_CODE for both K155ID1s on a single PCF8574.
//  byte data_for_pcf_off = (K155ID1_OFF_CODE << 4) | K155ID1_OFF_CODE; 
//
//  // Send the "OFF" BCD code to both PCF8574 chips.
//  writePcf8574(PCF8574_ADDRESS_1, data_for_pcf_off);
//  //writePcf8574(PCF8574_ADDRESS_2, data_for_pcf_off);
//}

void IRAM_ATTR one_sec_tick_timer_callback(void* arg)
{
  current_digit_bcd_tube1 = current_digit_bcd_tube1 + 1;
  if(current_digit_bcd_tube1 == 0x0A)
  {
    current_digit_bcd_tube1 = 0;
  }

    byte data_for_pcf1 = (digitCode[current_digit_bcd_tube2] << 4) | digitCode[current_digit_bcd_tube1];

  writePcf8574(PCF8574_ADDRESS_1, data_for_pcf1);

  byte data_for_pcf2 = (current_digit_bcd_tube4 << 4) | current_digit_bcd_tube3;
  //writePcf8574(PCF8574_ADDRESS_2, data_for_pcf2);
  
  led_update = 1;
}

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



/*
void digitCycle(int delayms)
{
  for(int i=0; i<10; i++) //loop through all possible 4 bit states rapidly to show that we have reached end of the loop
  {
    int ldrValue = analogRead(LDR_PIN);
    //writeDigit(i);
    write1byte(PCF8574_ADDRESS_1,i);
    delay(delayms); //wait 0.5sec before next transition
  }
}

void printDigit (byte digit)
{
  byte sendVal = (digitCode[digit] & 0x0F) | ((digit & 0x0F) << 4);
  Wire.beginTransmission(PCF8574_ADDRESS_1);
  Wire.write(sendVal);
  byte error = Wire.endTransmission();
  if (error == 0) {
  //Serial.println("I2C write Success");
  } else {
  Serial.print("I2C Error: ");
  Serial.println(error);
  }
}

// void writeDigit(int digit)
// {
//     int address = digitCode[digit];
//     // digitalWrite(pin1,bitRead(address,0)); //read 0th bit of i, set it high on pin1
//     // digitalWrite(pin2,bitRead(address,1));
//     // digitalWrite(pin3,bitRead(address,2));
//     // digitalWrite(pin4,bitRead(address,3));
// }

void write1byte(byte address, byte digit)
{
  byte sendVal = (digitCode[digit] & 0x0F) | ((digit & 0x0F) << 4);
  Serial.println(sendVal,BIN);
  Wire.beginTransmission(address);
  //Wire.write(0x01);
 Wire.write(sendVal);
  byte error = Wire.endTransmission();
  if (error == 0) {
  Serial.println("I2C write Success");
  } else {
  Serial.print("I2C Error: ");
  Serial.println(error);
  }
}



void showColor(uint32_t color, int LEDnum = 0) {
  strip.setPixelColor(LEDnum, color);  // Set the first (only) LED
  strip.show();
}

void rainbowFade() {
  for (int j = 0; j < 256; j++) {
    for (int i = 0; i < strip.numPixels(); i++) {
      strip.setPixelColor(i, wheel((i + j) & 255));
    }
    strip.show();
    delay(20);
  }
}

// Helper function to generate rainbow colors
uint32_t wheel(byte pos) {
  if (pos < 85) 
  return strip.Color(pos * 3, 255 - pos * 3, 0);
  else if (pos < 170) {
    pos -= 85;
    return strip.Color(255 - pos * 3, 0, pos * 3);
  } else {
    pos -= 170;
    return strip.Color(0, pos * 3, 255 - pos * 3);
  }
}


*/
