#include <Stepper.h>
#include <Wire.h>

// change this to fit the number of steps per revolution
const int STEPS_PER_REVOLUTION = 200;

// initialize the stepper library
Stepper myStepper(STEPS_PER_REVOLUTION, 12, 11, 10, 9);

// the number of steps the stepper motor has taken
int stepCounter = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  // nothing for the stepper motor
}

void loop() {
  // put your main code here, to run repeatedly:
  // motor speed must be specified in range 0 to 100
  myStepper.setSpeed(25);

  // step one revolution in one direction:
  Serial.println("turning stepper motor clockwise");
  delay(100);
  Serial.flush();
  delay(100);
  myStepper.step(STEPS_PER_REVOLUTION);
  delay(500);

  // step one revolution in the other direction:
  Serial.println("turning stepper motor counterclockwise");
  delay(100);
  Serial.flush();
  delay(100);
  myStepper.step(-STEPS_PER_REVOLUTION);
  delay(500);
}
