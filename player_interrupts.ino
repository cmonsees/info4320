/*************************************************** 
  This is an example for the Adafruit VS1053 Codec Breakout

  Designed specifically to work with the Adafruit VS1053 Codec Breakout 
  ----> https://www.adafruit.com/products/1381

  Adafruit invests time and resources providing this open source code, 
  please support Adafruit and open-source hardware by purchasing 
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.  
  BSD license, all text above must be included in any redistribution
 ****************************************************/

// include SPI, MP3 and SD libraries
#include <SPI.h>
#include <Adafruit_VS1053.h>
#include <SD.h>

// These are the pins used for the breakout example
#define BREAKOUT_RESET  9      // VS1053 reset pin (output)
#define BREAKOUT_CS     10     // VS1053 chip select pin (output)
#define BREAKOUT_DCS    8      // VS1053 Data/command select pin (output)
// These are the pins used for the music maker shield
#define SHIELD_RESET  -1      // VS1053 reset pin (unused!)
#define SHIELD_CS     7      // VS1053 chip select pin (output)
#define SHIELD_DCS    6      // VS1053 Data/command select pin (output)

// These are common pins between breakout and shield
#define CARDCS 4     // Card chip select pin
// DREQ should be an Int pin, see http://arduino.cc/en/Reference/attachInterrupt
#define DREQ 3       // VS1053 Data request, ideally an Interrupt pin

Adafruit_VS1053_FilePlayer musicPlayer = 
  // create breakout-example object!
  //Adafruit_VS1053_FilePlayer(BREAKOUT_RESET, BREAKOUT_CS, BREAKOUT_DCS, DREQ, CARDCS);
  // create shield-example object!
  Adafruit_VS1053_FilePlayer(SHIELD_RESET, SHIELD_CS, SHIELD_DCS, DREQ, CARDCS);


////
const int C_PIN = A0;
const int D_PIN = A1;
const int E_PIN = A2;
const int F_PIN = A3;
const int G_PIN = A4;
const int A_PIN = A5;

const int Red_PIN = 10;
const int Orange_PIN = 9;
const int Yellow_PIN = 8;
const int YG_PIN = 5;
const int Green_PIN = 2;

int incomingByte;

// Measure the voltage at 5V and resistance of your 3.3k resistor, and enter
// their value's below:
const float VCC = 4.98; // Measured voltage of Ardunio 5V line
const float R_DIV = 3230.0; // Measured resistance of 3.3k 

void setup() {
  Serial.begin(9600);
  pinMode(C_PIN, INPUT);
  pinMode(D_PIN, INPUT);
  pinMode(E_PIN, INPUT);
  pinMode(F_PIN, INPUT);
  pinMode(G_PIN, INPUT);
  pinMode(A_PIN, INPUT);

  pinMode(Red_PIN, OUTPUT);
  pinMode(Orange_PIN, OUTPUT);
  pinMode(Yellow_PIN, OUTPUT);
  pinMode(YG_PIN, OUTPUT);
  pinMode(Green_PIN, OUTPUT);
  
  
  Serial.println("Adafruit VS1053 Library Test");

  // initialise the music player
  if (! musicPlayer.begin()) { // initialise the music player
     Serial.println(F("Couldn't find VS1053, do you have the right pins defined?"));
     while (1);
  }
  Serial.println(F("VS1053 found"));
 
  if (!SD.begin(CARDCS)) {
    Serial.println(F("SD failed, or not present"));
    while (1);  // don't do anything more
  }
  Serial.println("SD OK!");
  
  // Set volume for left, right channels. lower numbers == louder volume!
  musicPlayer.setVolume(20,20);

  /***** Two interrupt options! *******/ 
  // This option uses timer0, this means timer1 & t2 are not required
  // (so you can use 'em for Servos, etc) BUT millis() can lose time
  // since we're hitchhiking on top of the millis() tracker
  //musicPlayer.useInterrupt(VS1053_FILEPLAYER_TIMER0_INT);
  
  // This option uses a pin interrupt. No timers required! But DREQ
  // must be on an interrupt pin. For Uno/Duemilanove/Diecimilla
  // that's Digital #2 or #3
  // See http://arduino.cc/en/Reference/attachInterrupt for other pins
  // *** This method is preferred
  if (! musicPlayer.useInterrupt(VS1053_FILEPLAYER_PIN_INT))
    Serial.println(F("DREQ pin is not an interrupt pin"));
}

void loop() {
  int C_read = analogRead(C_PIN);
  int C_force = calcForce(C_read);
  int D_read = analogRead(D_PIN);
  int D_force = calcForce(D_read);
  int E_read = analogRead(E_PIN);
  int E_force = calcForce(E_read);
  int F_read = analogRead(F_PIN);
  int F_force = calcForce(F_read);
  int G_read = analogRead(G_PIN);
  int G_force = calcForce(G_read);
  int A_read = analogRead(A_PIN);
  int A_force = calcForce(A_read);
    
  if(C_force > 0 && C_force < 50) { 
  Serial.print("c"); digitalWrite(Red_PIN, HIGH); musicPlayer.playFullFile("C4.mp3"); digitalWrite(Red_PIN, LOW);}
  if(C_force > 50) { 
  Serial.println("C"); digitalWrite(10, HIGH); musicPlayer.playFullFile("C5.mp3"); digitalWrite(10, LOW);}
  
  if(D_force > 0 && D_force < 50) { 
  Serial.println("d"); digitalWrite(Orange_PIN, HIGH); musicPlayer.playFullFile("D4.mp3"); digitalWrite(Orange_PIN, LOW); }
  if(D_force > 50) {  
  Serial.println("D"); digitalWrite(Orange_PIN, HIGH); musicPlayer.playFullFile("D5.mp3"); digitalWrite(Orange_PIN, LOW);}
  
  if(E_force > 0 && E_force < 50) {  
  Serial.println("e"); digitalWrite(Yellow_PIN, HIGH); musicPlayer.playFullFile("E4.mp3"); digitalWrite(Yellow_PIN, LOW);}
  if(E_force > 50) {   
  Serial.println("E"); digitalWrite(Yellow_PIN, HIGH);musicPlayer.playFullFile("E5.mp3"); digitalWrite(Yellow_PIN, LOW);}
  
  if(F_force > 0 && F_force < 50) {   
  Serial.println("f"); digitalWrite(YG_PIN, HIGH); musicPlayer.playFullFile("F4.mp3"); digitalWrite(YG_PIN, LOW);}
  if(F_force > 50) {   
  Serial.println("F"); digitalWrite(YG_PIN, HIGH); musicPlayer.playFullFile("F5.mp3"); digitalWrite(YG_PIN, LOW);}
  
  if(G_force > 0 && G_force < 50) {   
  Serial.println("g"); digitalWrite(Green_PIN, HIGH); musicPlayer.playFullFile("G4.mp3"); digitalWrite(Green_PIN, LOW);}
  if(G_force > 50) {   
  Serial.println("G"); digitalWrite(Green_PIN, HIGH); musicPlayer.playFullFile("G5.mp3"); digitalWrite(Green_PIN, LOW);} 
  
  if(A_force > 0 && A_force < 50) { Serial.write("$"); musicPlayer.playFullFile("A4.mp3");}
  if(A_force > 50) { Serial.println("$"); musicPlayer.playFullFile("A5.mp3");} 
  
  if (Serial.available() > 0) {
    // read the oldest byte in the serial buffer:
    incomingByte = Serial.read();
    if(incomingByte == 'b') { musicPlayer.playFullFile("B4.mp3");  Serial.println("Playing B4");}
    if(incomingByte == 'B') { musicPlayer.playFullFile("B5.mp3");  Serial.println("Playing B5");}
  } 
}



/// File listing helper
void printDirectory(File dir, int numTabs) {
   while(true) {
     
     File entry =  dir.openNextFile();
     if (! entry) {
       // no more files
       //Serial.println("**nomorefiles**");
       break;
     }
     for (uint8_t i=0; i<numTabs; i++) {
       Serial.print('\t');
     }
     Serial.print(entry.name());
     if (entry.isDirectory()) {
       Serial.println("/");
       printDirectory(entry, numTabs+1);
     } else {
       // files have sizes, directories do not
       Serial.print("\t\t");
       Serial.println(entry.size(), DEC);
     }
     entry.close();
   }
}

int calcForce(int input) {
  const float VCC = 5.00; // Measured voltage of Ardunio 5V line
  const float R_DIV = 3330.0; // Measured resistance of 3.3k resistor
  float force;

  float fsrV = input * VCC / 1023.0; // Calculate voltage
  float fsrR = R_DIV * (VCC / fsrV - 1.0); // Calculate FSR resistance
  float fsrG = 1.0 / fsrR; // Calculate conductance
  if (fsrR <= 600) {
    force = (fsrG - 0.00075) / 0.00000032639;
  } else { force =  fsrG / 0.000000642857; }

  int returnSpeed = map(force, 1, 1000, 0, 128); // Map force to motor Speed
  return returnSpeed;
}
