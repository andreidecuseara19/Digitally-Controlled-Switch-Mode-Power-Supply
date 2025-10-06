#include "Arduino.h"
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <SPI.h>
#include <Adafruit_MCP4725.h>
Adafruit_MCP4725 dac;
 
#define DAC_RESOLUTION    (9)   // Set this value to 9, 8, 7, 6 or 5 to adjust the resolution

//Defining the Liquid Crystal Display (address and size)
LiquidCrystal_I2C lcd(0x23,  16, 2);

//Defining the Voltage/Current measurement board address
byte ADDRESS_STEVAL = 0X40;
byte REGISTER_XY = 0X00;
byte READ_LENGTH = 2;

byte I2C_ADDR = ADDRESS_STEVAL;
byte I2C_REG = REGISTER_XY;
byte I2C_LENGTH = READ_LENGTH;

//Defining the Rotary Encoder KY-040 module
const int PinSW = 17;   // Rotary Encoder Switch
const int PinDT = 18;    // DATA signal
const int PinCLK = 19;    // CLOCK signal

volatile int displaycounter = 4095; // Store current counter value (volatile for ISR)
volatile uint8_t prevState = 0;  // Previous state of both pins
volatile uint8_t currentState = 0; // Current state of both pins

double VoltageRead = 0;
double CurrentRead = 0;

void setup() 
{
  Serial.begin(9600); //Begin serial monitor

  lcd.init(); //Initialize the lcd
  lcd.backlight(); //Turn on the backlight 
  lcd.setCursor(3, 0); //Position the cursor
  lcd.print("Startup..."); //Print characters
  delay(100);

  Serial.println("MCP4725 Test");
  if (dac.begin(0x60))        //could be 0x60 or 0x61
  {
    Serial.println("MCP4725 Initialized Successfully.");
  }
  else
  {
    Serial.println("Failed to Initialize MCP4725.");
  }
  //setup_timer(); // Function call for PWM timer setup

  Wire.begin(); //Begin I2C communication
  Wire.setClock(400000); //Set I2C 'full-speed'

  // Set pin modes
  pinMode(PinSW, INPUT_PULLUP);
  pinMode(PinDT, INPUT_PULLUP);
  pinMode(PinCLK, INPUT_PULLUP);
  
  // Read initial state
  prevState = (digitalRead(PinCLK) << 1) | digitalRead(PinDT);
  
  // Attach interrupts to both CLK and DT pins
  attachInterrupt(digitalPinToInterrupt(PinCLK), encoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PinDT), encoderISR, CHANGE);

  //Declaring the initial states of the STEVAL-DIGAFEV board and writing to it
  uint16_t STEVAL_RESET = 0x8037; //Reset bit on
  uint16_t STEVAL_INIT_T = 0x3F; //Temperature bit on
  uint16_t STEVAL_INIT = 0x37; //Temperature bit off
  uint16_t RSHUNT_5m = 0x1F4; //Shunt Resistance value of 5mOhm
  I2C_Write_int16(ADDRESS_STEVAL,0x00,STEVAL_INIT_T); //Writing the initial state
  I2C_Write_int16(ADDRESS_STEVAL,0x08,RSHUNT_5m); //Writing the value of shunt resistor, 50000 * 10uOhm

  Serial.print("Configuration register:\n");
  I2C_Read(I2C_ADDR,0x00,2);
  
  Serial.print("Shunt Resistor Value register:\n");
  I2C_Read(I2C_ADDR,0x08,2);
}

void loop() 
{
  // Check if Rotary Encoder switch was pressed
  if (digitalRead(PinSW) == LOW) {
    displaycounter = 800;  // Reset counter to zero
    Serial.print("\nCounter: ");
    Serial.print(displaycounter);
    Serial.print("\n");
    delay(200); // Simple debounce

    //digitalPotWrite(displaycounter);
  }
  //**********************************************

  //Reading from an I2C device at a specified address 
  Serial.print("Load Voltage Value:\n");
  VoltageRead = uintToDouble(I2C_Read(I2C_ADDR,0x02,2),0.002);
  Serial.println(VoltageRead);

  lcd.setCursor(1, 0); //Position the cursor
  lcd.print("Output V: "); //Print characters
  lcd.setCursor(11, 0); //Position the cursor
  lcd.print(VoltageRead);

  Serial.print("Current Value:\n");
  CurrentRead = (uintToDouble(I2C_Read(I2C_ADDR,0x04,2),0.002))/4;
  Serial.println(CurrentRead);

  lcd.setCursor(0, 1); //Position the cursor
  lcd.print("Current A: "); //Print characters
  lcd.setCursor(11, 1); //Position the cursor
  lcd.print(CurrentRead);

  //Serial.print("Temperature Value:\n");
  //I2C_Read(I2C_ADDR,0x05,2);
  //***********************************************

  if( (displaycounter >= 0) && (displaycounter <= 4095) ){
  Serial.print("Dac value set to:\n");
  Serial.println(displaycounter);
  dac.setVoltage(displaycounter, false);    //Set voltage proportionally
  }
  else
  Serial.println("Invalid DAC Value");

  delay(100);
}

double uintToDouble(uint16_t adcValue, double lsbVoltage) {
    return adcValue * lsbVoltage; // Direct multiplication
}

//Reading specified number of bytes (2 max) from I2C device
uint16_t I2C_Read(byte ADDR,byte REG,byte LENGTH)
{
  Wire.beginTransmission(ADDR);
  Wire.write(REG);  // Set register for read
  Wire.endTransmission(false); // Set false flag, to not release the line
  Wire.requestFrom(ADDR,LENGTH); // request bytes from register XY
  byte buff[LENGTH]; //Declaring the buffer for reading
  Wire.readBytes(buff, LENGTH); //Reading into the buffer

  uint16_t output;
  switch (LENGTH){
    case 1:
    output = buff[0];
    Serial.println(output, HEX);
    break;

    case 2:
    output = (buff[0] << 8) | buff[1];
    Serial.println(output, HEX);
    break;

    default:
    output = 0;
    Serial.println(output, HEX);
    break;
  }
  return output;
}

//Write 8bit value to I2C device
void I2C_Write_int8(byte ADDR,byte REG,byte DATA)
{
  Wire.beginTransmission(ADDR);
  Wire.write(REG);  // Set register for read
  Wire.write(DATA);
  Wire.endTransmission();
}

//Write 16bit value to I2C device
void I2C_Write_int16(byte ADDR,byte REG,uint16_t DATA)
{
  Wire.beginTransmission(ADDR);
  Wire.write(REG);  // Set register for read
  Wire.write(highByte(DATA));
  Wire.write(lowByte(DATA));
  Wire.endTransmission();
}

// Interrupt Service Routine for encoder
void encoderISR() {
  // Read current state
  currentState = (digitalRead(PinCLK) << 1) | digitalRead(PinDT);
  
  // Check for valid state transition (Gray code sequence)
  if ((prevState == 0b00 && currentState == 0b01) ||
      (prevState == 0b01 && currentState == 0b11) ||
      (prevState == 0b11 && currentState == 0b10) ||
      (prevState == 0b10 && currentState == 0b00)) {
    if(displaycounter == 4095 || displaycounter > 4095) displaycounter = 4095;
    else  displaycounter += 10;

    Serial.print("\nCounter: ");
    Serial.print(displaycounter);
    Serial.print("\n");
  }
  else if ((prevState == 0b00 && currentState == 0b10) ||
           (prevState == 0b10 && currentState == 0b11) ||
           (prevState == 0b11 && currentState == 0b01) ||
           (prevState == 0b01 && currentState == 0b00)) {
    if(displaycounter == 800 || displaycounter < 800) displaycounter = 800;
    else  displaycounter -= 10;

    Serial.print("\nCounter: ");
    Serial.print(displaycounter);
    Serial.print("\n");
  }
  prevState = currentState;
}