#include <Servo.h>
Servo actuatorX;
Servo actuatorY;

float motorAngleX = 0;
float motorAngleY = 0;
float motorAngleTargetX = 0;
float motorAngleTargetY = 0;
float servoInputX = 0;
float servoInputY = 0;
float timeStep = 0.1;
float timeSinceStart = 0;
float pi = 3.14159;

void setup() {
  Serial.begin(9600);
  
  actuatorX.attach(9);
  actuatorY.attach(6);
}

void loop() {
//servo is mad bc flight computer tells it too much too fast; slow down commands
  
  motorAngleTargetX = 10 * sin(timeSinceStart * pi / 180);

  if (motorAngleTargetX > motorAngleX + 0.06) {
    motorAngleX = motorAngleX + 0.06;
  }
  else if (motorAngleTargetX < motorAngleX - 0.06) {
    motorAngleX = motorAngleX - 0.06;
  }
  else {
    motorAngleX = motorAngleTargetX;
  }
  
  servoInputX = (0.02 * motorAngleX * motorAngleX) - (2.5 * motorAngleX) + 94;
  actuatorX.write(servoInputX);
  
  motorAngleTargetY = 10 * cos(timeSinceStart * pi / 180);

  if (motorAngleTargetY > motorAngleY + 0.06) {
    motorAngleY = motorAngleY + 0.06;
  }
  else if (motorAngleTargetY < motorAngleY - 0.06) {
    motorAngleY = motorAngleY - 0.06;
  }
  else {
    motorAngleY = motorAngleTargetY;
  }
  
  servoInputY = (0.02 * motorAngleY * motorAngleY) - (2.5 * motorAngleY) + 94;
  actuatorY.write(servoInputY);

  timeSinceStart = timeSinceStart + timeStep;
  delay(0);
}
