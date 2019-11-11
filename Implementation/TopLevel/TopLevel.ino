/*               eBike Speed Controller Implementation Code             */               

/// DESCRIPTION 
///  This code implements a PID controller designed in MATLAB digitally on an Arduino nano 
///   to control the speed of an instrumented ebike. 

/// ACKNOWLEDGEMENTS
///  PID_v1 library can be downloaded from here: https://github.com/br3ttb/Arduino-PID-Library
///   PinChangeInterrupt Library can be downloaded from here: https://github.com/GreyGnome/PinChangeInt
///    Tutorial for PinChangeInterrupt Library can be found here https://www.brainy-bits.com/make-any-arduino-pin-an-interrupt-pin/
///     Some of this code is based on Nicholas Chan's code for implementing a PID controller on a camera gimbal which can be found here: https://github.com/DavisDroneClub/gimbal 

// Sets code mode to run system diagnostics. This enables or disables logging information to the micro SD card. Additionally, serial monitoring can be toggled on and off here.
boolean diag = false; // toggles SD card logging
boolean serial = false; // toggles serial monitoring

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
* -------- Variable Declarations ---------
*/ 
// Hardware Pins
int genPin = A7;    // DC generator input pin
int Tpin = A6;      // Analog input pin for throttle trigger hall effect sensor input 
int plusPin = 17;    // Hardware interrupt pin INT0 for plus button
int minusPin = 19;   // Hardware interrupt pin INT1 for minus button
int outputPin = 3;  // Pin at which the output signal to the existing motor controller is sent
int csPin = 4;      // Chip select pin for SPI communication with the SD Card Module

// Variables Used For Button Debouncing
volatile unsigned long lastInterruptTimePlus = 0;
volatile unsigned long lastInterruptTimeMinus = 0; 

// Variables Used For Throttle Signal Averaging When Cruise Control is Off 
const int numReadings = 10;
int readings[numReadings];      // the readings from the analog input
int readIndex = 0;              // the index of the current reading
int total = 0;                  // the running total
int average = 0;

// LCD hardware pins 
int rs = 9; 
int en = 8;
int d4 = 7;
int d5 = 6;
int d6 = 5;
int d7 = 2;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7); // Sets up the LCD display 

// Miscallaneous Variables
int Tsig; // Signal incoming from the throttle trigger hall effect sensor
unsigned long currTime; // Used for logging time in diagnostics 
File diagFile; // Creates a new file object  
int fileCounter; // Variable used for naming .txt file written to SD card  

// Variables used in loop timing for diagnostic purposes
int loopStartState; 
int ifState1; 
int ifState2;
int ifState3;
int ifState4; 
int cruiseControlState = 0; // Keeps track of whether or not cruise control is engaged or not. Default is disengaged.
double unfiltrdGenVoltage; // declared here as a global variable to log to SD card 
double filtrdGenVoltage; // declared here as a global variable to log to SD card 

// Variables for low pass filtering 
double pastFiltrdSpd = 0, d_pastFiltrdSpd = 0, pastSpd = 0, pastTime = 0; 
double cutoffFreq = .001; // cutoff frequency in Hz
struct filtrdVals {     // structure definition used to return two values from the low pass filter function call
  double filtrdRdng; 
  double d_filtrdRdng; 
  double unfiltrdRdng;
}; 

// Constants for converting DC generator voltage to speed in getSpeed() function
double
sf = 2*3.1415/60,     /* from dissertation */ \
m = 456.3862,         /* from dissertation */ \
b = -1.2846,          /* from dissertation */ \
rR = 0.341,           /* from modeling the human controlled bicycle [m] */ \
rD = 0.028985,        /* from dissertation [m] */ \
rC = 0.333375         /* from dissertation [m] */ \ 
;

// PID Library Setup 
double kp = 1.03, ki = 0.145, kd = 0.05; // Constants Acquired From Controller Design Stage
volatile double Setpoint = 0; // declared as volatile so that its value may be shared between the button Interrupt Service Routine Functions and the main program
double Input, Output;
PID motorPID(&Input, &Output, &Setpoint, kp, ki, kd, P_ON_M, DIRECT); // Creates PID object. See PID library documentation
int OutputWrite; // variable for writing output to motor controller as a PWM duty cycle 

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
    pinMode(genPin, INPUT);            // Input from wheel speed sensor
    pinMode(Tpin, INPUT);              // Input from throttle 
    pinMode(plusPin, INPUT);           // Input from setpoint increment button
    pinMode(minusPin, INPUT);          // Input from setpoint decrement button
    pinMode(outputPin, OUTPUT);        // Output to the motor controller
    pinMode(A5, INPUT); 
    pinMode(csPin, OUTPUT);            // Output for chip select on micro SD card module
    pinMode(A2, OUTPUT);               // Output for throttle relay coil
    
    // Setting Button Pins to High 
    digitalWrite(plusPin, HIGH);
    digitalWrite(minusPin, HIGH);
    
    // PID library settings
    motorPID.SetOutputLimits(0,4.59);  // Cuts the PID output (V) at about the max output of the hall effect sensor at max throttle (measured at 940 bits ~4.59V) 
    motorPID.SetMode(AUTOMATIC); // turns PID on
    
    // Diagnostics
    if (diag) {
      
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
       // Updating fileCounter variable 
      fileCounter = EEPROM.read(0); 
      fileCounter++;
      EEPROM.write(0,fileCounter);
    }
    
    // Printing ready message
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Ready");
    delay(3000); 
    lcd.clear(); 
    
   // Prepping for signal averaging of the throttle signal when cruise control is off 
   for (int thisReading = 0; thisReading < numReadings; thisReading++) {  // initializing readings array with zeros
    readings[thisReading] = 0;
   } 
   
   lcd.clear();

} // END SETUP FUNCTION


/* 
* ------------ Loop Function -------------- 
*/
void loop() { 
 
 // PASSING THROTTLE THROUGH THE NANO
 if(cruiseControlState == 0) {
   
   digitalWrite(A2, HIGH); // Energizes the coil on the relay and closes the switch to eliminate the Arduino from the throttle circuit

   // Logging to the serial monitor 
   if(serial) serialData(cruiseControlState, Setpoint, Tsig, average, Input, Output, currTime); // displays pertinent info to serial monitor 
   
   Input = getSpeed(); // Acquiring input here for logging purposes
   
   // Printing current speed on LCD
   lcd.setCursor(0,0);
   lcd.print("Current Spd");
   lcd.setCursor(0,1);
   lcd.print(Input);
   
   // Logging Data For Diagnostics
   if(diag) {
    ifState1 = 0; currTime = millis();
    logData(cruiseControlState, Setpoint, Tsig, Input, Output, currTime, fileCounter, loopStartState, ifState1, ifState2, ifState3, ifState4); 
    }
  }

 // CRUISE CONTROL INITIALIZATION
 if(digitalRead(plusPin) == LOW && digitalRead(minusPin) == LOW) {  // If the user has activated the cruise control 
    
   cruiseControlState = 1; Tsig = 0;  // Setting Tsig to zero here so that it does not trigger the condition for cruise control disengagement presented below     
   
   // Setting the current speed as the initial setpoint speed
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

   // THE HANDOFF // Instead of recieving a throttle signal from the throttle itself, the motor controller will now recieve an emulated throttle signal coming from the Arduino
   digitalWrite(A2, LOW); // Turns off power to the coil on the throttle relay, enabling the Arduino to now take over passing its emulated throttle signa to the motor controller
   delay(100); // Some delay to ensure a clean handoff between the throttle and the Arduino
   OutputWrite = (int)Output * (255/5); // Converting Output [V] to PWM duty cycle
   analogWrite(outputPin, OutputWrite); // Writing output to motor controller 
 }

 // RUNNING THE PID ALGORITHM
 if(cruiseControlState == 1) {

    // Computing PID Output
    Input = getSpeed();
    motorPID.Compute();
    OutputWrite = (int)Output * (255/5); // Converting Output [V] to PWM duty cycle
    analogWrite(outputPin, OutputWrite); // Writing output to motor controller
    
    // Log performance data to serial monitor if diagnostics enabled
    if(serial) serialData(cruiseControlState, Setpoint, Tsig, average, Input, Output, currTime);
    
    //Updating the current speed on the LCD
    lcd.setCursor(9,1);
    lcd.print(Input); 

    //Updating the setpoint on the LCD 
    lcd.setCursor(7,0);
    lcd.print(Setpoint);
    
     // Logging Data For Diagnostics
    if(diag) {
     ifState3 = 0; currTime = millis();
     logData(cruiseControlState, Setpoint, Tsig, Input, Output, currTime, fileCounter, loopStartState, ifState1, ifState2, ifState3, ifState4); 
     }
    
   }

 // CHECKING FOR CONDITIONS FOR CRUISE CONTROL DISENGAGEMENT
 if(analogRead(Tpin) >= 700 && cruiseControlState == 1) {     // Threshoat 700 bits (~3.5V) 
   
   cruiseControlState = 0;
   Setpoint = 0; Output = 0; // Setting setpoint and output back to zero so they can be reinitialized next time cruise control is engaged
   
   // Letting the user know the cruise control is disengaging
   lcd.clear();
   lcd.setCursor(0,0);
   lcd.print("Cruise Control");
   lcd.setCursor(0,1);
   lcd.print("Disengaged");
   
   analogWrite(outputPin, 0); // turns off the output to  "flip the switch"
   delay(2000);
   lcd.clear(); 
   
   }  

} // END LOOP FUNCTION


/* 
* -------------- Utility Functions ----------- 
*/

// The following two functions are the Interrupt Service Routines (ISR) called when either the plus or minus buttons are pressed. 
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

// function for 2nd order Butterworth low pass filter 
// This function is based on the one implemented here: 
// https://gitlab.com/mechmotum/row_filter/blob/master/row_filter/complementary.py
struct filtrdVals lowPassFilter(double cutoffFreq, double dt, double currentRdng, double pastRdng, double pastFiltrdRdng, double d_pastFiltrdRdng) {
  
  struct filtrdVals vals;
  
  // Comput coefficients of the state equation 
  double a = pow((2 * M_PI * cutoffFreq), 2);
  double b = pow(2,0.5) * 2 * M_PI * cutoffFreq;
  
  // Integrate the filter state equation using the midpoint Euler method with time step h 
  double h = dt; 
  double denom = 4 + 2*h*b + pow(h,2)*a;
  
  double A = (4 + 2*h*b - pow(h,2)*a)/denom;
  double B = 4*h/denom; 
  double C = -4*h*a/denom; 
  double D = (4 - 2*h*b - pow(h,2)*a)/denom; 
  double E = 2*pow(h,2)*a/denom; 
  double F = 4*h*a/denom; 
  
  vals.unfiltrdRdng = currentRdng;
  vals.filtrdRdng = A*pastFiltrdRdng + B*d_pastFiltrdRdng + E*(currentRdng + pastRdng)/2; 
  vals.d_filtrdRdng = C*pastFiltrdRdng + D*d_pastFiltrdRdng + F*(currentRdng + pastRdng)/2;
  
  return vals;
}

// function for converting generator voltage reading into velocity 
// See: http://moorepants.github.io/dissertation/davisbicycle.html#calibration 
double getSpeed() {
  
  unfiltrdGenVoltage = (double)analogRead(genPin); 
  
  // Filtering analogRead 
  double dt = millis() - pastTime; // pastTime is the time at which this function was last called
  struct filtrdVals vals = lowPassFilter(cutoffFreq, dt, unfiltrdGenVoltage, pastSpd, pastFiltrdSpd, d_pastFiltrdSpd);
  
  // Updating global variables with new "past" values 
  pastSpd = vals.unfiltrdRdng;              // This is the current unfiltered speed value
  pastFiltrdSpd = vals.filtrdRdng;          // This is the current filtered speed value
  d_pastFiltrdSpd = vals.d_filtrdRdng;      // This is the current derivative of the filtered speed value
  pastTime = millis();                      // This is the current time
  
  // Transforming genVoltage to a voltage value
  filtrdGenVoltage = vals.filtrdRdng;     // updating global variable for logging purposes
  double genVoltage = 2.0*vals.filtrdRdng; // multiply by 2 to account for the voltage divider
  genVoltage = genVoltage*(5.0/1023.0); // Converts digital output of analogRead to voltage
  
  // Converting voltage to speed
  double speed = (sf*(m*genVoltage+b)*rR*rD)/rC; // Calibration equation from http://moorepants.github.io/dissertation/davisbicycle.html#calibration 
  
  return speed; // Returns the filtered speed
} 


// function for logging performance information to an SD card for data logging. This function is totally customizable to display whatever is deemed necessary
void logData(int cruiseControlState, double Setpoint, int Tsig, double Input, double Output, unsigned long currTime, int fileCounter, int loopStartState, int ifState1, int ifState2, int ifState3, int ifState4) {
  String stringOne = String(fileCounter);
  String fileName = String("TEST_" + stringOne + ".txt"); 
  diagFile = SD.open(fileName, FILE_WRITE);
  if(diagFile) {  // Skips executing the function if the file did not open correctly
    
    diagFile.print(cruiseControlState);
    diagFile.print(",");
    diagFile.print(Setpoint);
    diagFile.print(",");
    //diagFile.print(Tsig * (5.0/1023.0));  // converts signal from throttle to a voltage
    diagFile.print(Tsig);  // Raw Tsig value from analogRead
    diagFile.print(",");
    diagFile.print(Input);
    diagFile.print(",");

    // Getting DC Generator Voltage
    diagFile.print(filtrdGenVoltage);
    diagFile.print(",");
  
    diagFile.print(Output);  // Raw output value before analogWrite
    diagFile.print(",");
    diagFile.print(currTime/1000.0);  
    
    // Printing Loop Timing Flags 
    diagFile.print(",");
    diagFile.print(loopStartState);
    diagFile.print(",");
    diagFile.print(ifState1);
    diagFile.print(",");
    diagFile.print(ifState2);
    diagFile.print(",");
    diagFile.print(ifState3); 
    diagFile.print(",");
    diagFile.print(ifState4);
    
    diagFile.print(",");
    diagFile.println(unfiltrdGenVoltage);
    
    diagFile.close();
    }
} 

// function for writing pertinent information to serial monitor for debugging. This function is totally customizable to display whatever is deemed necessary
void serialData(boolean cruiseControlState, double Setpoint, int Tsig, int average, double Input, double Output, unsigned long currTime) { 
  if (cruiseControlState == true) {
    Serial.print("|CC_on, ");
  } else {
    Serial.print("|CC_off, ");
  }
  Serial.print(Setpoint);
  Serial.print(",| ");
  //Serial.print(Tsig * (5.0/1023.0));  // converts signal from throttle to a voltage
  Serial.print(Tsig);  // Raw Tsig value from analogRead
  Serial.print(",");
  Serial.print(analogRead(A6));
  Serial.print(",");
  Serial.print(Tsig/4);
  Serial.print(","); 
  Serial.print((int)average);
  Serial.print(",");
  Serial.print(Input);
  Serial.print(",");

  // Getting DC Generator Voltage
  double genVoltage = 2.0*(double)analogRead(genPin);   // Multiply by two to account for voltage divider
  genVoltage = genVoltage*(5.0/1023.0); // Converts digital output of analogRead to voltage
  //Serial.print(genVoltage);
  //Serial.print(",");
  
  //Serial.print(Output * (5.0/255.0));  // Output in voltage
  Serial.print(Output);  // Raw output value before analogWrite
  Serial.print(","); 
  Serial.print((int)Output*(255/5));
  Serial.print(",");
  Serial.println(currTime/1000.0); 
}
