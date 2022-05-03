#include <Wire.h>
#include <AccelStepper.h>

// AccelStepper setup
// simple 2 pin driver interface(PUL and DIR)
// arduino pin 2 connected to PUL pin of driver
// arduino pin 3 connected to DIR pin of driver
#define PUL_PIN 2
#define DIR_PIN 3
AccelStepper stepperX(AccelStepper::DRIVER, PUL_PIN, DIR_PIN);

// define the pins used
#define HOME_SWITCH 9 // pin 9 connected to home switch (MicroSwitch)

// stepper travel variables
long travelX; // used to store the x value entered in the serial monitor
int move_finished = 1; // used to check if move is completed
long initial_homing = -1; // used to home stepper at startup

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  pinMode(HOME_SWITCH, INPUT_PULLUP);

  delay(5); // wait for driver to wake up

  // set max speed and acceleration of each stepper at startup for homing
  stepperX.setMaxSpeed(100.0); // set max speed of stepper; slower means better accuracy
  stepperX.setAcceleration(100.0); // set acceleration of stepper

  // start homing procedure of stepper motor at startup
  Serial.print("stepper is homing...");
  while (digitalRead(HOME_SWITCH)) { // make the stepper move CCW until the switch is activated
    stepperX.moveTo(initial_homing); // set the position to move to
    initial_homing--; // decrease by 1 for next move if needed
    stepperX.run(); // start moving the stepper
    delay(5);
  }

  stepperX.setCurrentPosition(0); // set the current position as zero for now
  stepperX.setMaxSpeed(100.0); // set max speed of stepper; slower means better accuracy
  stepperX.setAcceleration(100.0); // set acceleration of stepper
  initial_homing = 1;
  
  while (!digitalRead(HOME_SWITCH)) { // make the stepper move CW until the switch is deactivated
    stepperX.moveTo(initial_homing);
    stepperX.run();
    initial_homing++;
    delay(5);
  }

  stepperX.setCurrentPosition(0);
  Serial.println("homing completed");
  Serial.println("");
  stepperX.setMaxSpeed(1000.0); // set max speed of stepper; faster for regular movements
  stepperX.setAcceleration(1000.0); // set acceleration of stepper

  // print out instructions on serial monitor at start
  Serial.println("enter travel distance (positive for CW / negative for CCW / zero for back to home");
}

void loop() {
  // put your main code here, to run repeatedly:
  while (Serial.available() > 0) { // check if values are available in the serial buffer
    move_finished = 0; // set variable for checking move of the stepper

    travelX = Serial.parseInt(); // put numeric value from buffer into travelX variable
    if (travelX < 0 || travelX > 1350) { // make sure the value entered is not beyond home or max pos
      Serial.println("");
      Serial.println("Please enter value > 0 and <= 1350");
      Serial.println("");
    } else {
      Serial.print("Moving stepper into position: ");
      Serial.println(travelX);

      stepperX.moveTo(travelX); // set new move to position of stepper

      delay(1000); // wait for one second
    }
  }

  if (travelX >= 0 && travelX <= 1350) {
    // check if the stepper has reached the desired position
    if (stepperX.distanceToGo() != 0) {
      stepperX.run(); // move stepper into position
    }

    // if move is completed, then display message on serial monitor
    if (move_finished == 0 && stepperX.distanceToGo() == 0) {
      Serial.println("COMPLETED");
      Serial.println("");
      Serial.println("enter travel distance (positive for CW / negative for CCW / zero for back to home");
      move_finished = 1; // reset move variable
    }
  }
}
