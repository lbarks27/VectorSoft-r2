#include <Servo.h>

Servo servoX;
Servo servoY;

float servoStartPos = 80;

void setup() {
  servoX.attach(3);
  servoY.attach(5);
  servoX.write(servoStartPos);
  servoY.write(servoStartPos);
}

void loop() {
  servoY.write(83);
  delay(200);
  servoY.write(90);
  delay(200);
  servoY.write(97);
  delay(200);
  servoY.write(90);
  delay(200);
  servoX.write(83);
  delay(200);
  servoX.write(90);
  delay(200);
  servoX.write(97);
  delay(200);
  servoX.write(90);
  delay(200);
}
