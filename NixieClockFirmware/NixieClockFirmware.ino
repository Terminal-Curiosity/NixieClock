#include <Wire.h>

//define digital pins communicating to driver chip
const int pin1 = 19;
const int pin2 = 21;
const int pin3 = 22;
const int pin4 = 23;

const int IO1_addr = 0x20; //assign the i2c expander addresses
const int IO2_addr = 0x22;
const int IO3_addr = 0x24;

#define SDA_pin 17
#define SCL_pin 16 

const int digitCode[] = {8,4,3,2,10,1,11,12,9,0}; //working using non inverted outputs
//const int digitCode[] = {0b0111,0b1011,0b1100,0b1101,0b0101,0b1110,0b0100,0b0011,0b0110,0b1111};

void digitCycle(int);
void writeDigit(int);
void write1byte(byte, byte);

void setup() {
  // put your setup code here, to run once:
  Wire.begin(SDA_pin, SCL_pin); // initialise IO bit expander 1 I2C channel
  Serial.begin(115200);
  pinMode(SDA_pin, INPUT_PULLUP); //use internal pullups on the I2c pins instead of external hardware resistors
  pinMode(SCL_pin, INPUT_PULLUP);
  
  pinMode(pin1, OUTPUT); // set all digital pins to output
  pinMode(pin2, OUTPUT);
  pinMode(pin3, OUTPUT);
  pinMode(pin4, OUTPUT);
  
  digitalWrite(pin1,LOW); // initialise all digital pins low
  digitalWrite(pin2,LOW);
  digitalWrite(pin3,LOW);
  digitalWrite(pin4,LOW);

  Wire.beginTransmission(IO3_addr); // turn off all GPIO pins on port expanders by writing all "1"
  Wire.write(0xFF);
  Wire.endTransmission();
  Wire.beginTransmission(IO2_addr);
  Wire.write(0xFF);
  Wire.endTransmission();
  Wire.beginTransmission(IO1_addr);
  Wire.write(0xFF);
  Wire.endTransmission();
  
}

void loop() {

  while(1)
    { 
      digitCycle(1000);
      digitCycle(100);
      
      
      
    }
  
}

void digitCycle(int delayms)
{
  for(int i=0; i<10; i++) //loop through all possible 4 bit states rapidly to show that we have reached end of the loop
  {
    writeDigit(i);
    write1byte(IO3_addr,i);
    delay(delayms); //wait 0.5sec before next transition
  }
}

void writeDigit(int digit)
{
    int address = digitCode[digit];
    digitalWrite(pin1,bitRead(address,0)); //read 0th bit of i, set it high on pin1
    digitalWrite(pin2,bitRead(address,1));
    digitalWrite(pin3,bitRead(address,2));
    digitalWrite(pin4,bitRead(address,3));
}

void write1byte(byte address, byte digit)
{
    byte sendVal = digitCode[digit] + (digit<<4);  
    Serial.println(sendVal,BIN);
    Wire.beginTransmission(address);
    //Wire.write(0x01);
    Wire.write(sendVal);
    Wire.endTransmission();
}
