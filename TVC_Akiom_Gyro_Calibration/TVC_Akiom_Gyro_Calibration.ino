//RUN MULTIPLE TRIALS PER ANGLE

#include <Wire.h>
#include <Servo.h>
#include <MPU6050.h>

MPU6050 mpu;

Servo pitch;
Servo roll;

//parameters
float targetPitch;
float targetRoll;
int accelPitchZero;
int accelRollZero;
float accelPitch;
float accelRoll;

//roll range ---(use angles instead) servo input; current zero is 90
//pitch range ---(use angles instead) servo input; current zero is 110
float servoPitchInput = 0;
float servoRollInput = 0;

void setup() {
  Serial.begin(115200);

  while (!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))
  {
    Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
    delay(500);
  }

  pitch.attach(8);
  roll.attach(9);
  pitch.write(110);
  roll.write(90);

  Serial.println("MPU6050 is initialized.");
  Serial.println();
  Serial.print("Select target pitch angle: ");

  while (Serial.available() == 0) {
  }

  targetPitch = Serial.parseFloat();
  Serial.println(targetPitch);
  Serial.end();
  Serial.begin(115200);

  Serial.println();
  Serial.print("Select target roll angle: ");

  while (Serial.available() == 0) {
  }

  targetRoll = Serial.parseFloat();
  Serial.println(targetRoll);
  Serial.end();
  Serial.begin(115200);

  Serial.println("Begin calibration.");

  //zero stuff

  delay(100);

  while (Serial.available() == 0) {
    Vector normAccel = mpu.readNormalizeAccel();

    float accelPitchZero = -(atan2(normAccel.XAxis, sqrt(normAccel.YAxis * normAccel.YAxis + normAccel.ZAxis * normAccel.ZAxis)) * 180.0) / M_PI;
    float accelRollZero = (atan2(normAccel.YAxis, normAccel.ZAxis) * 180.0) / M_PI;

    Serial.print("Zeros:");
    Serial.print(accelPitchZero);
    Serial.print("      ");
    Serial.println(accelRollZero);
    delay(10);
  }

}

void loop() {

  //sensor fetch

  Vector normAccel = mpu.readNormalizeAccel();

  delay(10);

  accelPitch = -((atan2(normAccel.XAxis, sqrt(normAccel.YAxis * normAccel.YAxis + normAccel.ZAxis * normAccel.ZAxis)) * 180.0) / M_PI) - accelPitchZero;
  accelRoll = ((atan2(normAccel.YAxis, normAccel.ZAxis) * 180.0) / M_PI) - accelRollZero;

  delay(10);

  //logic loop

  if (accelPitch > targetPitch) {
    servoPitchInput = servoPitchInput - 0.05;
  }

  else {
    servoPitchInput = servoPitchInput + 0.05;
  }

  if (accelRoll < targetRoll) {
    servoRollInput = servoRollInput - 0.05;
  }

  else {
    servoRollInput = servoRollInput + 0.05;
  }

  pitch.write(servoPitchInput + 110);
  roll.write(servoRollInput + 90);

  //output to serial

  Serial.print("      Pitch vs. Input:");
  Serial.print(accelPitch - accelPitchZero);
  Serial.print("      ");
  Serial.print(servoPitchInput);
  Serial.print("      Roll vs. Input:");
  Serial.print(accelRoll - accelRollZero);
  Serial.print("      ");
  Serial.println(servoRollInput);
}
