// e.g. motorPin[0] is motorEnable for motor 0
const int motorPin[3] = {D0, D3, A6};
const int leftPin[3] = {D2, D5, A4};
const int rightPin[3] = {D1, D4, A5};

int bendInfo[3][3] = {
  {0, 0, 0}, // time bent
  {0, 0, 0}, // cumulative bend speed
  {0, 0, 0}  // calculated bend
};

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
  return returnSpeed;
}

void spool(int motor, int speed, int direction) {
  if ((direction == 1) || (direction == -1)) {
    if (direction == 1) { // 1 = spool
      digitalWrite(leftPin[motor], HIGH);
      digitalWrite(rightPin[motor], LOW);
      analogWrite(motorPin[motor], speed);

      bendInfo[1][motor] += (speed/10);
    }
    if (direction == -1) { // -1 = unspool
      digitalWrite(leftPin[motor], LOW);
      digitalWrite(rightPin[motor], HIGH);
      analogWrite(motorPin[motor], speed);

      bendInfo[1][motor] -= (speed/10);
    }
    bendInfo[0][motor]++;
    bendInfo[2][motor] = (bendInfo[1][motor] / bendInfo[0][motor]);
  }
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

  if (C_Read > 50) { // If the analog reading is non-zero
    int motorSpeed = calcForce(C_Read);
    // 1 = spool; -1 = unspool
    spool(0, motorSpeed, spoolToggle);
    Serial.println(bendInfo[2][0]);
  }

  if (D_Read > 50) {// If the analog reading is non-zero
    int motorSpeed2 = calcForce(D_Read);
    // 1 = spool; -1 = unspool
    spool(1, motorSpeed2, spoolToggle);
    Serial.println(bendInfo[2][1]);
  }

  if (E_Read > 50) {// If the analog reading is non-zero
    int motorSpeed3 = calcForce(E_Read);
    // 1 = spool; -1 = unspool
    spool(2, motorSpeed3, spoolToggle);
    Serial.println(bendInfo[2][2]);
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
