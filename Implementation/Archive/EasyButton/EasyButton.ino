#include <PinChangeInterrupt.h>
#include <PinChangeInterruptBoards.h>
#include <PinChangeInterruptPins.h>
#include <PinChangeInterruptSettings.h>



    /////////////////////////////////////////////////////////////////
   //             Easy Arduino Button             v1.00           //
  //       Get the latest version of the code here:              //
 //     http://educ8s.tv/arduino-easy-button-tutorial           //
/////////////////////////////////////////////////////////////////

#define ACTIVATED LOW 
const int buttonPin = 18; 
const int buttonPin2 = 19;    
 
int buttonState = 0; 
int buttonState2 = 0;   

volatile unsigned long lastInterruptTime1 = 0;
volatile unsigned long lastInterruptTime2 = 0;

void setup() {
Serial.begin(9600);
  pinMode(buttonPin, INPUT);
  digitalWrite(buttonPin,HIGH); 
  pinMode(buttonPin2, INPUT);
  digitalWrite(buttonPin2, HIGH); 

   // Declaring Hardware Interrupts for plus and minus speed increment buttons
    attachPCINT (digitalPinToPCINT(buttonPin), plusISR, RISING); 
    attachPCINT (digitalPinToPCINT(buttonPin2), minusISR, RISING);
 //pinMode(5, INPUT);
}

void loop() {
  
  buttonState = digitalRead(buttonPin);
  buttonState2 = digitalRead(buttonPin2);
   if (buttonState == ACTIVATED && buttonState2 == ACTIVATED) {
   Serial.println("ACtivateDD");
  }
  else if ( buttonState == ACTIVATED || buttonState2 == ACTIVATED) {
    //Serial.println("PRESSED");
  }
  else {
   //Serial.println("closed");
  } 
 //digitalWrite(6, HIGH);
 //analogWrite(6,255);
 //float val = (5.0/1024.0) * analogRead(A5);
 //Serial.println(val);  
}

// The following two functions are the Interrupt Service Routine (ISR) called when either the plus or minus buttons are pressed
void plusISR() {
  if (millis() - lastInterruptTime1 > 500) {
  Serial.println("plus");
  lastInterruptTime1 = millis();
  }
}
void minusISR() {
  if (millis() - lastInterruptTime2 > 500) {
  Serial.println("minus");
  lastInterruptTime2 = millis();
  }
} 

boolean debounceButton(boolean state)
{
  boolean stateNow = digitalRead(buttonPin);
  if(state!=stateNow)
  {
    delay(10);
    stateNow = digitalRead(buttonPin);
  }
  return stateNow;
  
}
