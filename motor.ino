// e.g. motorPin[0] is motorEnable for motor 0
const int motorPin[3] = {D0, D3, A6};
const int leftPin[3] = {D2, D5, A4};
const int rightPin[3] = {D1, D4, A5};

const int C_Pin = A0; // Pin connected to FSR/resistor divider
const int D_Pin = A1;
const int E_Pin = A2;
const int F_Pin = A3;

// Passive buzzer setup
// const int buzz_pin = WKP;
// const int C4Freq = 262;
// const int D4Freq = 294;
// const int E4Freq = 330;
// const int F4Freq = 349;

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

  // configure the motor pins
  for (int i = 0; i < 3; i++) {
    pinMode(motorPin[i], OUTPUT);
    pinMode(leftPin[i], OUTPUT);
    pinMode(rightPin[i], OUTPUT);

    // set motors to default spool state
    spool(motorPin[i], 0, 1); // 1 = spool; -1 = unspool
  }
}

void loop(){
  int C_Read = analogRead(C_Pin);
  int D_Read = analogRead(D_Pin);
  int E_Read = analogRead(E_Pin);
  int F_Read = analogRead(F_Pin);

  if (C_Read > 50) { // 50 provides padding for signal noise
    int motorSpeed = calcForce(C_Read);
    Serial.print(String(motorSpeed)+"\n");
    // delay(5);
    spool(0, motorSpeed, spoolToggle);
    // Serial.print("Read C");
    // spoolEase(0, spoolToggle);
  }

  if (D_Read > 50) { // 50 provides padding for signal noise
    int motorSpeed2 = calcForce(D_Read);
    spool(1, motorSpeed2, spoolToggle);
  }

  if (E_Read > 50) { // 50 provides padding for signal noise
    int motorSpeed3 = calcForce(E_Read);
    spool(2, motorSpeed3, spoolToggle);
  }
  if (F_Read > 500) {
    spoolToggle = -spoolToggle;
    delay(300);
  }

  // if(C_Read >50){ tone(buzz_pin, C4Freq, 500);}
  // if(D_Read >50){ tone(buzz_pin, D4Freq, 500);}
  // if(E_Read >50){ tone(buzz_pin, E4Freq, 500);}
  // if(F_Read >50){ tone(buzz_pin, F4Freq, 500);}
}
