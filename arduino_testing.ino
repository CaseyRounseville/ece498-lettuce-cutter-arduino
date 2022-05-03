#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

// temp: 110f to 70f
// temp: 43.3333c to 21.1111c
// hum: 90% to 40%

#define MAX_TEMP 43.3333f
#define MIN_TEMP 21.1111f

#define MAX_HUM 90.0f
#define MIN_HUM 40.0f

// use pin 7 for the fan
const int PIN_FAN = 7;

// use pin 3 for the relay
const int PIN_RELAY = 3;

#define SEA_LEVEL_PRESSURE_HPA (103.25)

// initialize bme280
// temperature, humidity, and barometric pressure sensor
Adafruit_BME280 bme280;

void setup() {
  // put your setup code here, to run once:
  // nothing for the stepper motor

  // initialize the serial port for debugging
  Serial.begin(9600);

  // initialize pin 7 for analog output to control the fan
  pinMode(PIN_FAN, OUTPUT);

  // initialize pin 3 for digital output to control the relay
  pinMode(PIN_RELAY, OUTPUT);

  // make sure thr bme280 is wired correctly
  if (!bme280.begin(0x76)) {
    Serial.println("could not find a valid BME280 sensor, check the wiring");
    Serial.println("entering infinite loop");
    Serial.flush();
    while (1);
  }
}

void lightOn() {
  // activate the relay
  Serial.println("activating the relay");
  Serial.flush();
  digitalWrite(PIN_RELAY, HIGH);
}

void lightOff() {
  // deactivate the relay
  Serial.println("deactivating the relay");
  Serial.flush();
  digitalWrite(PIN_RELAY, LOW);
}

void fanOn() {
  // high speed for the fan
  Serial.println("putting fan on high speed");
  Serial.flush();
  analogWrite(PIN_FAN, 255);
  delay(5000);
}

void fanOff() {
  // stop the fan
  Serial.println("stopping fan");
  Serial.flush();
  analogWrite(PIN_FAN, 0);
  delay(5000);
}

void loop() {
  float tempCelsius = bme280.readTemperature();
  float humPercent = bme280.readHumidity();

  if (tempCelsius > MAX_TEMP && humPercent > MAX_HUM) {
    lightOff();
    fanOn();
  } else if (tempCelsius > MAX_TEMP && humPercent < MIN_HUM) {
    lightOn();
    fanOff();
  } else if (tempCelsius < MIN_TEMP && humPercent > MAX_HUM) {
    lightOn();
    fanOn();
  } else if (tempCelsius < MIN_TEMP && humPercent < MIN_HUM) {
    lightOn();
    fanOff();
  }

  // display temperature, humidity, and barometric pressure
  Serial.print("Temperature = ");
  Serial.print(tempCelsius);
  Serial.println("*C");
  Serial.flush();

  Serial.print("Pressure = ");
  Serial.print(bme280.readPressure() / 100.0F);
  Serial.println("hPa");
  Serial.flush();

  Serial.print("Approx. Altitude = ");
  Serial.print(bme280.readAltitude(SEA_LEVEL_PRESSURE_HPA));
  Serial.println("m");
  Serial.flush();

  Serial.print("Humidity = ");
  Serial.print(humPercent);
  Serial.println("%");
  Serial.flush();

  delay(3000);
}
