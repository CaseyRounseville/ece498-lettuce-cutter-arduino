# Lettuce Cutter Arduino
This is the code for the Arduino, which controls the stepper motors and
solenoid, and takes input from the optical sensors. It uses the AccelStepper
Arduino library for controlling the motors.

# Which File To Use?
The main file with the most code and progress is the file connect_serial_rpi.
All the other files have smaller test programs, but the connect_serial_rpi file
contains the main code that runs on the harvester machine, which includes code
for controlling the motors, actuating the pusher using the solenoid, receiving
commands from the Raspberry PI, and reading input from the sensors. The
arduino_testing.ino file contains the code for the environment chamber we used
to stress-test the electrical components to see if they can withstand the
temperature and humidity in a greenhouse.

# Uploading Code to the Arduino
Download the Arduino IDE, open the .ino Arduino code file in the Arduino IDE,
and click the "Upload" button. Note that the Arduino must be connected to your
computer with a USB cable to upload code to it.
