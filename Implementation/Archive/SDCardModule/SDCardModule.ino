/*
 *  Arduino SD Card Tutorial Example
 *  
 *  by Dejan Nedelkovski, www.HowToMechatronics.com
 */
#include <SD.h>
#include <SPI.h>
#include <LiquidCrystal.h>
File myFile;
int pinCS = 13; // Pin 10 on Arduino Uno

// LCD hardware pins 
int rs = 7; 
int en = 12;
int d4 = 11;
int d5 = 10;
int d6 = 9;
int d7 = 8;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7); // Sets up the LCD display 

void setup() {
    
  Serial.begin(9600);
  pinMode(pinCS, OUTPUT);

lcd.setCursor(0,0);
lcd.print("Starting");
delay(3000);
  
  // SD Card Initialization
  if (SD.begin())
  {
    Serial.println("SD card is ready to use.");
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Set");
    delay(3000);
  } else
  {
    Serial.println("SD card initialization failed");
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Dont go");
    delay(3000);
    return;
  }

lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Go");
    delay(3000);
  
  // Create/Open file 
  myFile = SD.open("test.txt", FILE_WRITE);
  
  // if the file opened okay, write to it:
  if (myFile) {
    Serial.println("Writing to file...");
    // Write to file
    myFile.println("Testing text 1, 2 ,3...");
    myFile.close(); // close the file
    Serial.println("Done.");
  }
  // if the file didn't open, print an error:
  else {
    Serial.println("error opening test.txt");
  }
  // Reading the file
  myFile = SD.open("test.txt");
  if (myFile) {
    Serial.println("Read:");
    // Reading the whole file
    while (myFile.available()) {
      Serial.write(myFile.read());
   }
    myFile.close();
  }
  else {
    Serial.println("error opening test.txt");
  }
  
}
void loop() {
  // empty
}
