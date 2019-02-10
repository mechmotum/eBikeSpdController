
/*               eBike Speed Controller Implementation Code             */               

/// DESCRIPTION
///  This code implements a PID controller designed in MATLAB digitally on an Arduino nano 
///   to control the speed of an instrumented ebike. 

/// ACKNOWLEDGEMENTS
///  PID_v1 library can be downloaded from here: https://github.com/br3ttb/Arduino-PID-Library
///   PinChangeInterrupt Library can be downloaded from here: https://github.com/GreyGnome/PinChangeInt
///    Tutorial for PinChangeInterrupt Library can be found here https://www.brainy-bits.com/make-any-arduino-pin-an-interrupt-pin/
///     This code is based on Nicholas Chan's code for implementing a PID controller on a camera gimbal which can be found here: https://github.com/DavisDroneClub/gimbal 

// Sets code mode to run system diagnostics. Set to true to enable diagnostics and false to disable
boolean diag = false;
boolean serial = true;

/* 
* -------- Code Libraries --------
*/
#include <PID_v1.h>              // Library for PID features
#include <LiquidCrystal.h>       // Library for LCD display 
#include <SD.h>                  // Library for logging data to the SD card
#include <EEPROM.h>              // Library for using EEPROM
#include <SPI.h>                 // Library for SPI communications 
#include <PinChangeInterrupt.h>  // The following libraries enable hardware interrupts to be used on any pin of the nano
#include <PinChangeInterruptBoards.h>
#include <PinChangeInterruptPins.h>
#include <PinChangeInterruptSettings.h>


/* 
* -------- Function Prototypes ----------
*/
double getSpeed();                                                                  // Function for converting raw measured voltage into speed
void plusISR();                                                                     // Interrupt Service Routine called when plus speed increment button is pressed
void minusISR();                                                                    // Interrupt Service Routine called when minus speed increment button is pressed
void logData(double Setpoint, double Input, double Output, unsigned long currTime); // Function that logs pertinent information to SD card
void serialData(boolean cruiseControlState, double Setpoint, float Tsig, double Input, double Output, unsigned long currTime); // Function that prints pertinent information to serial monitor for debugging

/* 
* -------- Variable Declarations ---------
*/ 
// Hardware Pins
int genPin = A7;    // DC generator input pin
int Tpin = A6;      // Analog input pin for throttle trigger hall effect sensor input 
int plusPin = 17;    // Hardware interrupt pin INT0 for plus button
int minusPin = 19;   // Hardware interrupt pin INT1 for minus button
int outputPin = A4;  // Pin at which the output signal to the existing motor controller is sent
int csPin = 4;      // Chip select pin for SPI communication with the SD Card Module

// Variables Used For Button Debouncing
volatile unsigned long lastInterruptTimePlus = 0;
volatile unsigned long lastInterruptTimeMinus = 0;

// LCD hardware pins 
int rs = 9; 
int en = 8;
int d4 = 7;
int d5 = 6;
int d6 = 5;
int d7 = 2;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7); // Sets up the LCD display 

float Tsig; // Signal incoming from the throttle trigger hall effect sensor
int MC_Val;
unsigned long currTime; // Used for logging time in diagnostics 
File diagFile; // Creates a new file object 

// Constants for converting DC generator voltage to speed 
double
sf = 2*3.1415/60,     /* from dissertation */ \
m = 456.3862,         /* from dissertation */ \
b = -1.2846,          /* from dissertation */ \
rR = 0.341,           /* from modeling the human controlled bicycle [m] */ \
rD = 0.028985,        /* from dissertation [m] */ \
rC = 0.333375         /* from dissertation [m] */ \ 
;

// PID Setup 
double kp = 7120.0, ki = 272.0, kd = 0;
volatile double Setpoint = 0; // declared as volatile so that its value may be shared between the ISR and the main program
double Input, Output;
PID motorPID(&Input, &Output, &Setpoint, kp, ki, kd, P_ON_M, DIRECT);

boolean cruiseControlState = false; // Keeps track of whether or not cruise control is engaged or not. Default is disengaged.

/* 
* ------------ Setup Function ------------ 
*/ 
void setup() {
    
    if(serial) Serial.begin(9600); 
    
    // Initializing LCD 
    lcd.begin(16,2);
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Initializing...");
    delay(3000);
    lcd.clear();
    
    // Declaring Hardware Interrupts for plus and minus speed increment buttons
    attachPCINT (digitalPinToPCINT(plusPin), plusISR, RISING); 
    attachPCINT (digitalPinToPCINT(minusPin), minusISR, RISING);
    
    // Declaring pin modes for hardware I/O
    pinMode(genPin, INPUT);
    pinMode(Tpin, INPUT);
    pinMode(plusPin, INPUT);
    pinMode(minusPin, INPUT);
    pinMode(outputPin, OUTPUT);
    pinMode(A5, INPUT); 
    pinMode(csPin, OUTPUT);
    
    // Setting Button Pins to High 
    digitalWrite(plusPin, HIGH);
    digitalWrite(minusPin, HIGH);
    
    // PID library settings
    motorPID.SetOutputLimits(0,198);  // Cuts the PID output at ~3.9V which is ~the max output of the hall effect sensor given the throttle range of motion 
    motorPID.SetMode(AUTOMATIC); // turns PID on
    
    // Diagnostics
    if (diag) {
      // SD Card Module Pins
      //pinMode(csPin, OUTPUT);
      
      // SD Card Initialization
      if (SD.begin()) {
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print("SD Card");
        lcd.setCursor(0,1);
        lcd.print("Rdy");
        delay(3000);
      }
      else {
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print("SD card");
        lcd.setCursor(0,1);
        lcd.print("init. failed"); 
        delay(2000);
        return;
      } 
    }
    
    // Printing ready message
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Ready");
    delay(3000); 
    lcd.clear();

} // END SETUP FUNCTION


/* 
* ------------ Loop Function -------------- 
*/
void loop() {

 currTime = millis();
 
 // Printing message to let user know cruise control is off
 lcd.setCursor(0,0);
 lcd.print("Cruise Control");
 lcd.setCursor(0,1);
 lcd.print("Off");
 
 // Passing throttle signal through the Arduino 
 Tsig = analogRead(Tpin); 
 analogWrite(outputPin, Tsig/4.0); 

 if(serial) serialData(cruiseControlState, Setpoint, Tsig, Input, Output, currTime); // displays pertinent info to serial monitor 
 
 if(digitalRead(plusPin) == LOW && digitalRead(minusPin) == LOW) {  // If the user has activated the cruise control 

   cruiseControlState = true;
   
   // Letting the user know they engaged the cruise control
   lcd.clear();
   lcd.print("Cruise Control");
   lcd.setCursor(0,1);
   lcd.print("Engaged");
   delay(2000);
   
   float currSpeed = getSpeed(); 
   currSpeed = 0.1*round(currSpeed*10.0); // rounds the current speed to the nearest 0.1m/s
   Setpoint = currSpeed; 
   
   // Setting up the display formating
   lcd.clear();
   
   // Top row for setpoint
   lcd.setCursor(0,0);
   lcd.print("SetPt:");
   lcd.setCursor(7,0);
   lcd.print(Setpoint);
   lcd.setCursor(12,0);
   lcd.print("m/s");
  
   // Bottom row for current speed
   lcd.setCursor(0,1);
   lcd.print("CurrSpd:");
   lcd.setCursor(9,1);
   lcd.print(Input);
   lcd.setCursor(13,1);
   lcd.print("m/s");
   
   Input = getSpeed(); // measures the new current speed as the input to the PID algorithim
   motorPID.Compute(); // computes the output
   analogWrite(outputPin, 0); // turns the output from the nano off
   analogWrite(outputPin, Output); // Some manipulation of "Output" may be needed here
   
   while(analogRead(Tpin) <= 410) { // If the throttle is slightly moved past it's neutral position (~2V), exit the cruise control
    currTime = millis();
    Input = getSpeed();
    motorPID.Compute();
    analogWrite(outputPin, Output); // Some manipulation of "Output" may be needed here 
    
    // Log performance data to SD card and/or serial monitor if diagnostics enabled
    if(diag) logData(Setpoint, Input, Output, currTime);
    if(serial) serialData(cruiseControlState, Setpoint, Tsig, Input, Output, currTime);
    
    //Updating the current speed on the LCD
    lcd.setCursor(9,1);
    lcd.print(Input); 

    //Updating the setpoint on the LCD 
    lcd.setCursor(7,0);
    lcd.print(Setpoint);
   } 

   cruiseControlState = false;
   
   delay(500);
   // Letting the user know the cruise control is disengaging
   lcd.clear();
   delay(750);
   lcd.setCursor(0,0);
   lcd.print("Cruise Control");
   lcd.setCursor(0,1);
   lcd.print("Disengaged");
   
   analogWrite(outputPin, 0); // turns off the output to  "flip the switch"
   delay(2000);
   lcd.clear();
   
 } // END CRUISE CONTROL FEATURES



} // END LOOP FUNCTION


/* 
* -------------- Utility Functions ----------- 
*/

// The following two functions are the Interrupt Service Routine (ISR) called when either the plus or minus buttons are pressed. 
// Each function contains an if statement that evaluates the amount of time that has passed since the last time the ISR was called, effectively debouncing the button
void plusISR() {
  if (millis() - lastInterruptTimePlus > 500) {
   Setpoint+=0.1;
   lastInterruptTimePlus = millis();
  }
}
void minusISR() {
  if (millis() - lastInterruptTimeMinus > 500) {
   Setpoint-=0.1;
   lastInterruptTimeMinus = millis();
  }
}

// function for converting generator voltage reading into actual velocity 
// See: http://moorepants.github.io/dissertation/davisbicycle.html#calibration 
double getSpeed() {
  int genVoltage = 2*analogRead(genPin); 
  genVoltage = map(genVoltage,0,1023,0,5); // Converts digital output of analogRead to voltage
  double speed = (sf*(m*genVoltage+b)*rR*rD)/rC;
  return speed;
} 

// function for logging performance information to an SD card for data logging
void logData(double Setpoint, double Input, double Output, unsigned long currTime) {
  diagFile = SD.open("test_1.txt", FILE_WRITE);
  if(diagFile) {  // Skips executing the function if the file did not open correctly
    diagFile.print(Setpoint);
    diagFile.print(",");
    diagFile.print(Input);
    diagFile.print(",");
    diagFile.print(Output);
    diagFile.print(",");
    diagFile.println(currTime/1000.0);
    diagFile.close();
  }
} 

// function for writing pertinent information to serial monitor for debugging
void serialData(boolean cruiseControlState, double Setpoint, float Tsig, double Input, double Output, unsigned long currTime) { 
  if (cruiseControlState == true) {
    Serial.print("|CC_on, ");
  } else {
    Serial.print("|CC_off, ");
  }
  Serial.print(Setpoint);
  Serial.print(",| ");
  Serial.print(Tsig * (5.0/1023.0));  // converts signal from throttle to a voltage
  Serial.print(",");
  Serial.print(Input);
  Serial.print(",");

  // Getting DC Generator Voltage
  int genVoltage = 2*analogRead(genPin); 
  genVoltage = map(genVoltage,0,1023,0,5); // Converts digital output of analogRead to voltage
  Serial.print(",");
  Serial.print(genVoltage);
  
  Serial.print(Output *(5.0/255.0));  // Output in voltage
  Serial.print(",");
  Serial.println(currTime/1000.0); 
}
