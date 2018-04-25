//#include <Adafruit_VS1053.h>
// https://learn.adafruit.com/adafruit-music-maker-shield-vs1053-mp3-wav-wave-ogg-vorbis-player/downloads

// setup for stepper
#include <Stepper.h>
int STEPS_PER_MOTOR_REVOLUTION = 32;
int STEPS_PER_OUTPUT_REVOLUTION= 2048;
int steps_1;
Stepper stepper_1(STEPS_PER_MOTOR_REVOLUTION, D0, D1, D2, D3);

// set LED PWM pins
const int led_1 = A0;
const int led_2 = A1;
const int led_3 = A6;

// set up pressure sensor
const int pressure_in = A2;
const float VCC = 3.32;
const float R_DIV = 5000.0;
const int pressure_light = 50; // threshold for accepting light tap
const int pressure_heavy = 2000; // threshold for accepting heavy tap

// fades LEDs in and out
void fadeLED(int led) {
  for (int i=0; i<256; i+=15) {
    analogWrite(led,i);
    delay(50);
  }
  for (int i=255; i>-1; i-=15) {
    analogWrite(led,i);
    delay(50);
  }
}

void setup() {
  // set LED pins to output
  pinMode(led_1, OUTPUT);
  pinMode(led_2, OUTPUT);
  pinMode(led_3, OUTPUT);

  // set pressure pin
  pinMode(pressure_in, INPUT);
}

void loop() {
  // LED
  //fadeLED(led_1);
  //fadeLED(led_2);
  //fadeLED(led_3);

  // Pressure
  int pressure_read = analogRead(pressure_in);
  // map raw pressure data to stepper input
  // stepper motor has 48 steps in 1 rotation
  int pressure_map = map(pressure_read, 0, 4095, 0, 48);

  stepper_1.setSpeed(960);
  stepper_1.step(pressure_map);
  //delay(100);
}
