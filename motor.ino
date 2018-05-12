// e.g. motorPin[0] is motorEnable for motor 0
const int motorPin[3] = {D0, D3, A6};
const int leftPin[3] = {D2, D5, A4};
const int rightPin[3] = {D1, D4, A5};

const int Blue_PIN = D6;
const int Pink_PIN = D7;
int incomingByte; // a variable to read incoming serial data into
 
const int B_PIN = A3;
const float VCC = 4.98; // Measured voltage of Ardunio 5V line
const float R_DIV = 3230.0; // Measured resistance of 3.3k

int calcForce(int input) {
  const float VCC = 3.3; // Measured voltage of Ardunio 5V line
  const float R_DIV = 3330.0; // Measured resistance of 3.3k resistor
  float force;

  float fsrV = input * VCC / 1023.0; // Calculate voltage
  float fsrR = R_DIV * (VCC / fsrV - 1.0); // Calculate FSR resistance
  float fsrG = 1.0 / fsrR; // Calculate conductance
  if (fsrR <= 600) {
    force = (fsrG - 0.00075) / 0.00000032639;
  } else { force =  fsrG / 0.000000642857; }

  int returnSpeed = map(force, 1, 1000, 0, 128); // Map force to motor Speed
  if (abs(returnSpeed) > 128) { returnSpeed = 128; }

  return returnSpeed;
}

void spool(int motor, int speed, int direction) {
  if ((direction == 1) || (direction == -1)) {
    if (direction == 1) { // 1 = spool
      digitalWrite(leftPin[motor], HIGH);
      digitalWrite(rightPin[motor], LOW);
      analogWrite(motorPin[motor], speed);
    }
    if (direction == -1) { // -1 = unspool
      digitalWrite(leftPin[motor], LOW);
      digitalWrite(rightPin[motor], HIGH);
      analogWrite(motorPin[motor], speed);
    }
  }
}

void spoolEase(int motor, int direction) {
  if (direction == 1) { // 1 = spool
    digitalWrite(leftPin[motor], HIGH);
    digitalWrite(rightPin[motor], LOW);
    for (int e = 1; e < 128; e++) {
      analogWrite(motorPin[motor], e);
      e = e*2;
      delay(1);
    }
    for (int o = 1; o < 128; o++) {
      analogWrite(motorPin[motor], o);
      o = o/2;
      delay(1);
    }
  }
  if (direction == -1) { // -1 = unspool
    digitalWrite(leftPin[motor], LOW);
    digitalWrite(rightPin[motor], HIGH);
    for (int e = 1; e < 128; e*2) {
      analogWrite(motorPin[motor], e);
    }
    for (int o = 1; o < 128; o/2) {
      analogWrite(motorPin[motor], o);
    }
  }
  delay(100);
}

int spoolToggle = 1; // global spool direction

void setup() {
  Serial.begin(9600);
  Serial1.begin(9600);
  // configure the motor pins
  for (int i = 0; i < 3; i++) {
    pinMode(motorPin[i], OUTPUT);
    pinMode(leftPin[i], OUTPUT);
    pinMode(rightPin[i], OUTPUT);

    // set motors to default spool state
    spool(motorPin[i], 0, 1); // 1 = spool; -1 = unspool
  }

  pinMode(B_PIN, INPUT);
  pinMode(Blue_PIN, OUTPUT);
  pinMode(Pink_PIN, OUTPUT);
}

void loop(){

  if(Serial1.available() >0) { //Lights for A
    incomingByte = Serial1.read();
    if(incomingByte == 'B') {digitalWrite(Blue_PIN, HIGH); delay(1000); digitalWrite(Blue_PIN, LOW);}
  }
  
  int B_read = analogRead(B_PIN);
  int B_force = calcForce(B_read);
  
  if(B_force > 50 && B_force < 100) {
  digitalWrite(Pink_PIN, HIGH); Serial.print('4'); Serial1.print('4'); delay(1000); digitalWrite(Pink_PIN, LOW);}
  if(B_force > 100) {
  digitalWrite(Pink_PIN, HIGH);Serial1.print('5'); delay(1000); digitalWrite(Pink_PIN, LOW);}
  
//  int C_Read = analogRead(C_Pin);
//  int D_Read = analogRead(D_Pin);
//  int E_Read = analogRead(E_Pin);
//  int F_Read = analogRead(F_Pin);
//
//  if (C_Read > 50) { // 50 provides padding for signal noise
//    int motorSpeed = calcForce(C_Read);
//    Serial.print(String(motorSpeed)+"\n");
//    // delay(5);
//    spool(0, motorSpeed, spoolToggle);
//    // Serial.print("Read C");
//    // spoolEase(0, spoolToggle);
//  }
//
//  if (D_Read > 50) { // 50 provides padding for signal noise
//    int motorSpeed2 = calcForce(D_Read);
//    spool(1, motorSpeed2, spoolToggle);
//  }
//
//  if (E_Read > 50) { // 50 provides padding for signal noise
//    int motorSpeed3 = calcForce(E_Read);
//    spool(2, motorSpeed3, spoolToggle);
//  }
//  if (F_Read > 500) {
//    spoolToggle = -spoolToggle;
//    delay(300);
//  }

}
