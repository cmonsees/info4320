// Motor control
const int motorEnable_Pin = D0;
const int leftSide_Pin = D1;
const int rightSide_Pin = D2;

const int motorEnable_Pin2 = D3;
const int leftSide_Pin2 = D4;
const int rightSide_Pin2 = D5;

const int motorEnable_Pin3 = A6;
const int leftSide_Pin3 = A5;
const int rightSide_Pin3 = A4;

// current transition count
int pulse_count;

const int C_Pin = A0; // Pin connected to FSR/resistor divider
const int D_Pin = A1;
const int E_Pin = A2;
const int F_Pin = A3;

const int buzz_pin = WKP;

const int C4Freq = 262;
const int D4Freq = 294;
const int E4Freq = 330;
const int F4Freq = 349;

// Measure the voltage at 5V and resistance of your 3.3k resistor, and enter
// their value's below:
const float VCC = 3.3; // Measured voltage of Ardunio 5V line
const float R_DIV = 3330.0; // Measured resistance of 3.3k resistor

void setup() {
  Serial.begin(9600);

  // configure the motor pins
  pinMode(motorEnable_Pin,OUTPUT);
  pinMode(leftSide_Pin,OUTPUT);
  pinMode(rightSide_Pin,OUTPUT);

  pinMode(motorEnable_Pin2,OUTPUT);
  pinMode(leftSide_Pin2,OUTPUT);
  pinMode(rightSide_Pin2,OUTPUT);

  pinMode(motorEnable_Pin3,OUTPUT);
  pinMode(leftSide_Pin3,OUTPUT);
  pinMode(rightSide_Pin3,OUTPUT);

  pinMode(buzz_pin, OUTPUT);

  // Motor is off going forward
  analogWrite(motorEnable_Pin,0);
  digitalWrite(leftSide_Pin, HIGH);
  digitalWrite(rightSide_Pin, LOW);

  // Motor2 is off going forward
  analogWrite(motorEnable_Pin2,0);
  digitalWrite(leftSide_Pin2, LOW);
  digitalWrite(rightSide_Pin2, HIGH);

  // Motor3 is off going forward
  analogWrite(motorEnable_Pin3,0);
  digitalWrite(leftSide_Pin3, LOW);
  digitalWrite(rightSide_Pin3, HIGH);
}

int calcForce(int input) {
  // Use ADC reading to calculate voltage:
  float fsrV = input * VCC / 1023.0;
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
  else {
    force =  fsrG / 0.000000642857;
  }
  int returnStrength = map(force, 1, 1000, 0, 255);

  return returnStrength;
}

void loop(){
  int C_Read = analogRead(C_Pin);
  int D_Read = analogRead(D_Pin);
  int E_Read = analogRead(E_Pin);
  int F_Read = analogRead(F_Pin);
  // If the FSR has no pressure, the resistance will be
  // near infinite. So the voltage should be near 0.

  if (C_Read != 0) { // If the analog reading is non-zero
    int motorStrength = calcForce(C_Read);
    // Motor 1: left LOW, right HIGH = SPOOL
    digitalWrite(leftSide_Pin, HIGH);
    digitalWrite(rightSide_Pin, LOW);
    analogWrite(motorEnable_Pin, motorStrength);

    // Serial.println(motorStrength);
    // // Serial.println("Force: " + String(force) + " g");
    // Serial.println();

  }

  if (D_Read != 0) {// If the analog reading is non-zero
    int motorStrength2 = calcForce(D_Read);
    // Motor 2: left LOW, right HIGH = SPOOL
    digitalWrite(leftSide_Pin2, LOW);
    digitalWrite(rightSide_Pin2, HIGH);
    analogWrite(motorEnable_Pin2, motorStrength2);

    // Serial.println(motorStrength2);
    // Serial.println("Force: " + String(force2) + " g");
    // Serial.println();
  }

  if (E_Read != 0) {// If the analog reading is non-zero
    int motorStrength3 = calcForce(E_Read);
    // Motor 2: left LOW, right HIGH = SPOOL
    digitalWrite(leftSide_Pin3, LOW);
    digitalWrite(rightSide_Pin3, HIGH);
    analogWrite(motorEnable_Pin3, motorStrength3);

    // Serial.println(motorStrength2);
    // Serial.println("Force: " + String(force2) + " g");
    // Serial.println();
  }

  if(C_Read >50){ tone(buzz_pin, C4Freq, 500);}
  if(D_Read >50){ tone(buzz_pin, D4Freq, 500);}
  if(E_Read >50){ tone(buzz_pin, E4Freq, 500);}
  if(F_Read >50){ tone(buzz_pin, F4Freq, 500);}
}
