// e.g. motorPin[0] is motorEnable for motor 0
const int motorPin[3] = {D0, D3, A6};
const int leftPin[3] = {D2, D5, A4};
const int rightPin[3] = {D1, D4, A5};
int motorCounter[3] = {0, 0, 0};

const int petalSPin = A0;
const int petalDPin = A1;
const int STEPS_PER_TURN = 200;
const int delay_between_step_microsec = 5000;

const int Blue_PIN = D6;
const int Pink_PIN = D7;
int incomingByte; // a variable to read incoming serial data into

const int B_PIN = A3;
const float VCC = 4.98; // Measured voltage of Ardunio 5V line
const float R_DIV = 3230.0; // Measured resistance of 3.3k

int spoolToggle = 1; // global spool direction

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

  int returnSpeed = map(force, 1, 250, 0, 128); // Map force to motor Speed
  if (abs(returnSpeed) > 128) { returnSpeed = 128; }

  return returnSpeed;
}

void step(bool forward) {
  if (forward == true) {
    digitalWrite(petalDPin, HIGH);
  } else {
    digitalWrite(petalDPin, LOW);
  }
  // creating a step
  digitalWrite(petalSPin, HIGH);
  // minimum delay is 1.9us
  digitalWrite(petalSPin, LOW);
}

void petalSteps(int number_of_steps) {
  Serial.write("Steps called\n");
  bool move_forward = true;
  // Establishing the direction
  if (number_of_steps >= 0) {
    move_forward = true;
  } else {
    move_forward = false;
    number_of_steps = -number_of_steps;
  }
  for (int i = 0; i < number_of_steps; i++) {
    step(move_forward);
    // Delay for proper speed
    delayMicroseconds(delay_between_step_microsec);
  }
}

void spool(int motor, int speed, int direction) {
  if ((direction == 1) || (direction == -1)) {
    digitalWrite(motorPin[motor], LOW);
    // if (motorCounter[motor] >= 3) {
    //   spoolToggle = -spoolToggle;
    //   motorCounter[motor]--;
    //   // spool(motor, speed, spoolToggle);
    // }
    // if (motorCounter[motor] < 0) {
    //   spoolToggle = spoolToggle;
    //   motorCounter[motor]++;
    //   // spool(motor, speed, spoolToggle);
    // }
    if (direction == 1) { // 1 = spool
      digitalWrite(leftPin[motor], HIGH);
      digitalWrite(rightPin[motor], LOW);
      motorCounter[motor]++;
      analogWrite(motorPin[motor], speed);
    }
    if (direction == -1) { // -1 = unspool
      digitalWrite(leftPin[motor], LOW);
      digitalWrite(rightPin[motor], HIGH);
      motorCounter[motor]--;
      analogWrite(motorPin[motor], speed);
    }
    delay(150);
    digitalWrite(motorPin[motor], LOW);
  }
}

void setup() {
  Serial.begin(9600);
  Serial1.begin(9600);
  // configure the motor pins
  for (int i = 0; i < 3; i++) {
    pinMode(motorPin[i], OUTPUT);
    pinMode(leftPin[i], OUTPUT);
    pinMode(rightPin[i], OUTPUT);

    // set motors to default spool state
    digitalWrite(motorPin[i], LOW); // 1 = spool; -1 = unspool
  }

  pinMode(B_PIN, INPUT);
  pinMode(Blue_PIN, OUTPUT);
  pinMode(Pink_PIN, OUTPUT);

  pinMode(petalSPin, OUTPUT);
  pinMode(petalDPin, OUTPUT);
}

void loop(){
  if(Serial1.available() >0) { //Lights for A
    incomingByte = Serial1.read();
    if ((incomingByte == 'c') || (incomingByte == 'C')) {
      Serial.write("C received\n");
      spool(0, 100, -spoolToggle);
    }
    if ((incomingByte == 'd') || (incomingByte == 'D')) {
      Serial.write("D received\n");
      spool(0, 100, -spoolToggle);
      spool(1, 100, -spoolToggle);
    }
    if ((incomingByte == 'e') || (incomingByte == 'E')) {
      Serial.write("E received\n");
      spool(1, 100, -spoolToggle);
    }
    if ((incomingByte == 'F') || (incomingByte == 'f')) {
      Serial.write("Direction toggled\n");
      spoolToggle = -spoolToggle;
      //spool(1, 25, -spoolToggle);
      //spool(2, 25, -spoolToggle);
    }
    if ((incomingByte == 'G') || (incomingByte == 'g')) {
      Serial.write("G received\n");
      spool(2, 100, -spoolToggle);
    }
    if (incomingByte == '$') {
      Serial.write("A received\n");
      // spool(2, 25, spoolToggle);
      // spool(0, 25, -spoolToggle);
      petalSteps(200);
      digitalWrite(Blue_PIN, HIGH);
      digitalWrite(Blue_PIN, LOW);
    }

  }

  int B_read = analogRead(B_PIN);
  int B_force = calcForce(B_read);
  if(B_force > 50) {
    Serial.write("B received\n");
    // spool(0, 25, spoolToggle);
    petalSteps(-200);
    digitalWrite(Pink_PIN, HIGH);
    Serial.print('b');
    Serial1.print('b');
    digitalWrite(Pink_PIN, LOW);
  }
}
