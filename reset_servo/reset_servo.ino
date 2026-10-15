#include <Servo.h>

// Create two servo objects
Servo servo1;
Servo servo2;

// Define digital pins for the servos
const int servoPin1 = 9;
const int servoPin2 = 10;

void setup() {
  // Attach the servo objects to their respective pins
  servo1.attach(servoPin1);
  servo2.attach(servoPin2);

  // Set both servos to 90 degrees
  servo1.write(90);
  servo2.write(90);
}

void loop() {
  // Nothing needed here, both servos stay at 90 degrees
}
