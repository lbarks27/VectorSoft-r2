#include "NineAxesMotion.h"        //Contains the bridge code between the API and the Arduino Environment
#include <Wire.h>
#include <Servo.h>
#include "math.h"

NineAxesMotion mySensor;         //Object that for the sensor 
Servo R;
Servo P;

const float pi = 3.142;
long liftoffTS; //time since liftoff will be millis - liftoffTS

//orientation info
float q_meas[4];
float q_err[4];
float q_err_SHORT[4];
float q_sp[4] = {0.5,0.5,0.5,0.5}; //constant SP for now
float w_[3];

//torque control solvers
float torque[3];
const float CM_TVC = 0.089; 
float thrust; //newtons?

//guidance gains
float Pq[3] = {1,1,1};
float Pw[3] = {1,1,1};

//output settings
float PS; //pitch servo angle degrees
float RS; //roll servo angle degrees
float RW; //reac wheel: obsolete

void thrustCurve() {
  
}

void controlUpdate() {

//tvc section torque-based controller; no need for gain schedules on thrust, but will still need for mass changes
torque[0] = (-1 * Pq[0] * q_err_SHORT[1]) - (Pw[0] * w_[0]);
torque[1] = (-1 * Pq[1] * q_err_SHORT[2]) - (Pw[1] * w_[1]);
torque[2] = (-1 * Pq[2] * q_err_SHORT[3]) - (Pw[2] * w_[2]);

PS = (180 / pi) * asin(torque[0] / (thrust * CM_TVC));
RS = (180 / pi) * asin(torque[2] / (thrust * CM_TVC));

//to find q_err with complex conj of q_meas by q_sp
q_err[0] = (q_sp[0] * q_meas[0]) - (q_sp[1] * q_meas[1] * -1) - (q_sp[2] * q_meas[2] * -1) - (q_sp[3] * q_meas[3] * -1);
q_err[1] = (q_sp[0] * q_meas[1]) + (q_sp[1] * q_meas[0] * -1) + (q_sp[2] * q_meas[3] * -1) - (q_sp[3] * q_meas[2] * -1);
q_err[2] = (q_sp[0] * q_meas[2]) - (q_sp[1] * q_meas[3] * -1) + (q_sp[2] * q_meas[0] * -1) + (q_sp[3] * q_meas[1] * -1);
q_err[3] = (q_sp[0] * q_meas[3]) + (q_sp[1] * q_meas[2] * -1) - (q_sp[2] * q_meas[1] * -1) + (q_sp[3] * q_meas[0] * -1);

if (q_err[0] < 0) {
  q_err_SHORT[0] = q_err[0] * -1;
  q_err_SHORT[1] = q_err[1] * -1;
  q_err_SHORT[2] = q_err[2] * -1;
  q_err_SHORT[3] = q_err[3] * -1;
}

else {
  q_err_SHORT[0] = q_err[0];
  q_err_SHORT[1] = q_err[1];
  q_err_SHORT[2] = q_err[2];
  q_err_SHORT[3] = q_err[3];
}

//reac wheel section: obsolete
  
}

void inertialUpdate() { //list runtimes later on for each void

mySensor.updateQuat();
mySensor.updateGyro();
mySensor.updateEuler();

q_meas[0] = mySensor.readQuatW() / pow(pow(mySensor.readQuatW(), 2) + pow(mySensor.readQuatX(), 2) + pow(mySensor.readQuatY(), 2) + pow(mySensor.readQuatZ(), 2), 0.5);
q_meas[1] = mySensor.readQuatX() / pow(pow(mySensor.readQuatW(), 2) + pow(mySensor.readQuatX(), 2) + pow(mySensor.readQuatY(), 2) + pow(mySensor.readQuatZ(), 2), 0.5);
q_meas[2] = mySensor.readQuatY() / pow(pow(mySensor.readQuatW(), 2) + pow(mySensor.readQuatX(), 2) + pow(mySensor.readQuatY(), 2) + pow(mySensor.readQuatZ(), 2), 0.5);
q_meas[3] = mySensor.readQuatZ() / pow(pow(mySensor.readQuatW(), 2) + pow(mySensor.readQuatX(), 2) + pow(mySensor.readQuatY(), 2) + pow(mySensor.readQuatZ(), 2), 0.5);

w_[0] = mySensor.readGyroX();
w_[1] = mySensor.readGyroY();
w_[2] = mySensor.readGyroZ();

}

void servo() {


//actuator hard limits
  if (PS > 4) {
    PS = 4;
  }

  else if (PS < -4) {
    PS = -4;
  }

  else {
    PS = PS;
  }
  
  if (RS > 4) {
    RS = 4;
  }

  else if (RS < -4) {
    RS = -4;
  }

  else {
    RS = RS;
  }
  
  P.write((PS * PS * -0.231) + (PS * 8.711) + 0.529 + 90);
  R.write((RS * RS * -0.441) + (RS * -7.935) - 0.647 + 135);
  
}

void dataOutput() {
  
}

void setup() {
  //Peripheral Initialization
  Serial.begin(115200);           //Initialize the Serial Port to view information on the Serial Monitor
  Serial.flush();
  I2C.begin();                    //Initialize I2C communication to the let the library communicate with the sensor.
  //Sensor Initialization
  mySensor.initSensor();          //The I2C Address can be changed here inside this function in the library
  mySensor.setOperationMode(OPERATION_MODE_NDOF);   //Can be configured to other operation modes as desired
  mySensor.setUpdateMode(MANUAL);  //The default is AUTO. Changing to MANUAL requires calling the relevant update functions prior to calling the read functions
  //Setting to MANUAL requires fewer reads to the sensor

  R.attach(9);
  P.attach(10);
  R.write(135); //zeros, R corresponds to last number of O
  P.write(90);

}

void loop() {
  inertialUpdate();
  controlUpdate();

Serial.print(q_err_SHORT[1]);
Serial.print("  ");
Serial.print(q_err_SHORT[2]);
Serial.print("  ");
Serial.print(q_err_SHORT[3]);
Serial.println();

delay(10);

}
