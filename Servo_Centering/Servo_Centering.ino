#include <Servo.h>
Servo actuatorX;
Servo actuatorY;

float servoInputX;
float servoInputY;
float motorAngleX = 0;
float motorAngleY = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  actuatorX.attach(9);
  actuatorY.attach(6);
}

void loop() {
 
  servoInputX = (0.0171 * motorAngleX * motorAngleX) - (2.4744 * motorAngleX) + 99;
  servoInputY = (0.0171 * motorAngleY * motorAngleY) - (2.4744 * motorAngleY) + 94;
  actuatorX.write(servoInputX);
  actuatorY.write(servoInputY);
}
