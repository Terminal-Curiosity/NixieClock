#include <Wire.h>
#include <Adafruit_NeoPixel.h>
#include <esp_timer.h> 

volatile bool oneSecondFlag = false; // set by the one second timer ISR. used by loop

//assign GPIO numbers
const int SDA1_pin = 4;
const int SCL1_pin = 15;
const int SDA2_pin = 15;
const int SCL2_pin = 22;
const int INT2 = 19; //interrupt pin on second I2C bus (not used in this version)
const int led_pin = 21;
const int HV_enable_pin = 23; //active high
const int LDR_pin = 33; //LDR connected to GPIO 2

//-----------LDR Configuration -------------------
const int LDR_BRIGHT_ADC_MIN = 100;  // Example: ADC reading for very bright light
const int LDR_DARK_ADC_MAX = 3200;  // Example: ADC reading for very dark light

// --- PWM Configuration ---
const long NIXIE_TUBE_REFRESH_TIME = 1000; // 500us refresh period = 2kHz nixie tube refresh rate to protect against cathode poisoning
const int MIN_BRIGHTNESS_PERCENT = 10; // Minimum duty cycle percentage
const int MAX_BRIGHTNESS_PERCENT = 100; // Maximum duty cycle percentage

// --- Nixie Tube & PCF8574 Configuration ---
const byte PCF8574_ADDRESS_1 = 0x20; // Address of your first PCF8574
const byte PCF8574_ADDRESS_2 = 0x24; // Address of your second PCF8574
const byte K155ID1_OFF_CODE = 0x0F; // BCD 1111 (an invalid input that turns off all cathodes on K155ID1)
const int digitCodesOdds[] = {9,0,8,4,12,2,10,6,14,1}; //tube pin to digit mapping for tubes

// --- Global Variables for PWM and Display (volatile for ISR access) ---
volatile byte current_digit_bcd_tube1 = 0x00; // Example: Display digit '0' for tube 1
volatile byte current_digit_bcd_tube2 = 0x02; // Example: Display digit '2' for tube 2
volatile byte current_digit_bcd_tube3 = 0x03; // Example: Display digit '3' for tube 3
volatile byte current_digit_bcd_tube4 = 0x08; // Example: Display digit '4' for tube 4

// --- Timer Handles ---
esp_timer_handle_t one_sec_tick_timer_handle; // One second timer to increment clock time

// --- Function Prototypes (IRAM_ATTR (instruction RAM atribute) for ISRs for reliability) ---
// IRAM_ATTR places the ISR functions in RAM instead of flash because its much faster and is not affected if flash memory is already in use elsewhere.
void IRAM_ATTR pwm_on_timer_callback(void* arg); 
// void IRAM_ATTR pwm_off_timer_callback(void* arg);
void IRAM_ATTR one_sec_tick_timer_callback(void* arg);

void readLDRAndMapBrightness();
void writePcf8574(byte address, byte data);
void write1byte(byte, byte);
uint32_t ColorHSV(float h, float s, float v);
void doOneSecondTick();
void updateTubeDisplay(int value_to_display); 
void showCurrentTime();


Adafruit_NeoPixel strip = Adafruit_NeoPixel(4, led_pin, NEO_GRB + NEO_KHZ800);
bool led_update = 0;

uint32_t currentTimeSec; // current time in seconds since midnight


void setup() {
  Wire.begin(SDA1_pin, SCL1_pin);  //initialise I2C with pins specified
  Wire.setClock(100000); // set I2C speed (default is 100kHz)
  pinMode(led_pin,OUTPUT);
  digitalWrite(led_pin, LOW);
  analogSetAttenuation(ADC_11db); // sets attenuation of LDR input (otherwise it will be 0-1V)
  pinMode(HV_enable_pin,OUTPUT); //initialise the HV pin as output and set low (HV disabled)
  digitalWrite(HV_enable_pin, LOW);

  strip.begin();         // Initialize NeoPixel strip
  strip.setBrightness(50);  // brightness from 0–255
  strip.show();          // Turn off all LEDs at start
  
  Wire.beginTransmission(PCF8574_ADDRESS_1);
  Wire.write(0xFF);
  Wire.endTransmission();
  delay(5);
  Serial.begin(115200);

  currentTimeSec = 86200; //start at midnight


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


uint16_t hue = 0;
void loop() {

    digitalWrite(HV_enable_pin, HIGH); //enable HV supply
    //updateTubeDisplay(2026);
  if(oneSecondFlag)
  {
    oneSecondFlag = false; //reset the flag
    doOneSecondTick();     //increment the time by one second
    showCurrentTime();   //update the tube display to show the new time
    Serial.println("time updated");

  uint32_t color = ColorHSV(hue, 1, 1);

  uint32_t led_colour_val = 0x000000FF << current_digit_bcd_tube1;
  strip.setPixelColor(0, color);  // Set the first (only) LED
  strip.setPixelColor(1, color);
  strip.setPixelColor(2, color);
  strip.setPixelColor(3, color);

  

  
  strip.show();
  led_update = 0;
  hue = hue+36;
  Serial.println(hue);
  if(hue >= 360)
  {
    hue = 0;
  }
  }


  
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


// --- I2C Write Helper Function ---
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

void doOneSecondTick(void)
//update the time value by one second. 
//call functions to propagate the new time to the display tubes
{
  currentTimeSec++;
  if(currentTimeSec >= 86400) //wrap around after 24 hours
  {
    currentTimeSec = 0;
  } 
}

void showCurrentTime()
{
  byte hours = currentTimeSec / 3600;
  byte minutes = (currentTimeSec % 3600) / 60;
  byte seconds = currentTimeSec % 60;

  //int16_t displayTime = hours * 100 + minutes; //display HHMM only
  int16_t displayTime = minutes * 100 + seconds; //display MMSS only
  updateTubeDisplay(displayTime);
  //Serial.print("Time(HHMM): " + String(displayTime)+ "\n"); 
  Serial.print("Time(MMSS): " + String(displayTime)+ "\n"); 
  Serial.print("Time(SSSSS): " + String(currentTimeSec)+ "\n"); 
}


void updateTubeDisplay(int value_to_display = 0)
//update the tube display to show the value passed into the function.
//NOTE: tubes 1 and 3 have non-standard digit wiring, so the upper nibbles must be remapped before being used.
{

  //display upper pair
  uint8_t upper = digitCodesOdds[value_to_display / 1000];
  upper = ((upper & 0b1000) | ((upper & 0b0001) << 2) | ((upper & 0b0110) >> 1)) << 4; //realign bits as per wiring
  uint8_t lower = digitCodesOdds[(value_to_display % 1000) / 100];
  byte data_for_pcf2 = upper | lower;
  writePcf8574(PCF8574_ADDRESS_2, data_for_pcf2); //upper digits

  //display lower pair
  upper = digitCodesOdds[(value_to_display % 100) / 10];
  upper = ((upper & 0b1000) | ((upper & 0b0001) << 2) | ((upper & 0b0110) >> 1)) << 4; //realign bits as per wiring
  lower = digitCodesOdds[(value_to_display % 10)];
  byte data_for_pcf1 = upper | lower;
  writePcf8574(PCF8574_ADDRESS_1, data_for_pcf1); //lower digits
  
}

void IRAM_ATTR one_sec_tick_timer_callback(void* arg)
{
  oneSecondFlag = true; //after one sec, set the flag true
}

