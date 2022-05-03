// define pins
#define PIN_REVERSE_SWITCH 2 // push button for reverse
#define PIN_DRIVER_PUL 7 // pul pin
#define PIN_DRIVER_DIR 6 // dir pin
#define PIN_SPD A0 // potentiometer(resistor)

// variables
int pd = 200; // pulse delay period
boolean setDir = LOW; // set direction

// interrupt handler
void revMotor() {
  // toggle the direction of the motor
  setDir = !setDir;
}

void setup() {
  // put your setup code here, to run once:
  pinMode(PIN_DRIVER_PUL, OUTPUT);
  pinMode(PIN_DRIVER_DIR, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_REVERSE_SWITCH), revMotor, FALLING);

}

void loop() {
  // put your main code here, to run repeatedly:
  //pd = map(analogRead(PIN_SPD), 0, 1023, 2000, 50);
  digitalWrite(PIN_DRIVER_DIR, setDir);
  digitalWrite(PIN_DRIVER_PUL, HIGH);
  delayMicroseconds(pd);
  digitalWrite(PIN_DRIVER_PUL, LOW);
  delayMicroseconds(pd);
}
