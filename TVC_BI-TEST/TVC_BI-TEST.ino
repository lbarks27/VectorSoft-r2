#include <Servo.h>

Servo sx;
Servo sy;

float xcenter = 91;
float ycenter = 92;

float xpos = 0; //s3
float ypos = 10; //s2

float t;

void setup() {
sx.attach(3);
sy.attach(2);
}

void loop() {
sx.write(xcenter + xpos);
sy.write(ycenter + ypos);

xpos = 15 * sin(t);
ypos = 15 * cos(t);

t += 0.01;

delay(10);
}
