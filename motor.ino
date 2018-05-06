// Motor control
const int motorEnablePin = D0;
const int leftSidePin = D1;
const int rightSidePin = D2;

// current transition count
int pulse_count;

const int CPin = A0; // Pin connected to FSR/resistor divider
const int DPin = A1;
const int EPin = A2;
const int FPin = A3;

const int BuzzPin = WKP;

const int C4Freq = 262;
const int D4Freq = 294;
const int E4Freq = 330;
const int F4Freq = 349;

// Measure the voltage at 5V and resistance of your 3.3k resistor, and enter
// their value's below:
const float VCC = 3.3; // Measured voltage of Ardunio 5V line
const float R_DIV = 3330.0; // Measured resistance of 3.3k resistor

void setup(){
  Serial.begin(9600);
  
  // configure the motor pins
  pinMode(motorEnablePin,OUTPUT);
  pinMode(leftSidePin,OUTPUT);
  pinMode(rightSidePin,OUTPUT);

  pinMode(BuzzPin, OUTPUT);
  
  // Motor is off going forward
  analogWrite(motorEnablePin,0);
  digitalWrite(leftSidePin, HIGH);
  digitalWrite(rightSidePin, LOW);
}

void loop(){
  int fsrADC = analogRead(CPin);
  // If the FSR has no pressure, the resistance will be
  // near infinite. So the voltage should be near 0.
  
  if (fsrADC != 0) // If the analog reading is non-zero
  {
    // Use ADC reading to calculate voltage:
    float fsrV = fsrADC * VCC / 1023.0;
    // Use voltage and static resistor value to 
    // calculate FSR resistance:
    float fsrR = R_DIV * (VCC / fsrV - 1.0);
    Serial.println("Resistance: " + String(fsrR) + " ohms");
    // Guesstimate force based on slopes in figure 3 of
    // FSR datasheet:
    float force;
    float fsrG = 1.0 / fsrR; // Calculate conductance
    // Break parabolic curve down into two linear slopes:
    if (fsrR <= 600){
      force = (fsrG - 0.00075) / 0.00000032639;}
    else{
      force =  fsrG / 0.000000642857;}
    
    int motorStrength = map(force, 1, 1000, 0, 255);
    analogWrite(motorEnablePin, motorStrength);

    Serial.println(motorStrength);
    Serial.println("Force: " + String(force) + " g");
    Serial.println();
  
    delay(500);
  }
  int C_Read = analogRead(CPin);
  int D_Read = analogRead(DPin);
  int E_Read = analogRead(EPin);
  int F_Read = analogRead(FPin);

  if(C_Read >50){ tone(BuzzPin, C4Freq, 500);}
  if(D_Read >50){ tone(BuzzPin, D4Freq, 500);}
  if(E_Read >50){ tone(BuzzPin, E4Freq, 500);}
  if(F_Read >50){ tone(BuzzPin, F4Freq, 500);}
}
