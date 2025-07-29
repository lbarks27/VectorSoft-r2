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
float q_sp[4] = {0.5,0.5,0.5,0.5}; //constant SP for now

float q_meas[4] = {0,0,0,0};
float aa_meas[4] = {0,0,0,0};
float angular_velocity[3] = {0,0,0};

float vec_rot[4] = {0,0,0,0};
float rotation_delta[4] = {0,0,0,0};

float w_velocity[3] = {0,0,0};
float w_euler_target[3] = {0,0,0};
float w_euler_error[3] = {0,0,0};
float target_w = 0;

float hamilton_helper[4];

void inertialUpdate() {

  mySensor.updateQuat();
  mySensor.updateGyro();
  mySensor.updateEuler();

  w_velocity[0] = mySensor.readGyroX();
  w_velocity[1] = mySensor.readGyroY();
  w_velocity[2] = mySensor.readGyroZ();
  
  q_meas[0] = mySensor.readQuatW() / pow(pow(mySensor.readQuatW(), 2) + pow(mySensor.readQuatX(), 2) + pow(mySensor.readQuatY(), 2) + pow(mySensor.readQuatZ(), 2), 0.5);
  q_meas[1] = mySensor.readQuatX() / pow(pow(mySensor.readQuatW(), 2) + pow(mySensor.readQuatX(), 2) + pow(mySensor.readQuatY(), 2) + pow(mySensor.readQuatZ(), 2), 0.5);
  q_meas[2] = mySensor.readQuatY() / pow(pow(mySensor.readQuatW(), 2) + pow(mySensor.readQuatX(), 2) + pow(mySensor.readQuatY(), 2) + pow(mySensor.readQuatZ(), 2), 0.5);
  q_meas[3] = mySensor.readQuatZ() / pow(pow(mySensor.readQuatW(), 2) + pow(mySensor.readQuatX(), 2) + pow(mySensor.readQuatY(), 2) + pow(mySensor.readQuatZ(), 2), 0.5);

  angular_velocity[0] = mySensor.readGyroX();
  angular_velocity[1] = mySensor.readGyroY();
  angular_velocity[2] = mySensor.readGyroZ();

  rotation_delta[0] = (q_meas[0] * q_sp[0]) - (q_meas[1] * q_sp[1] * -1) - (q_meas[2] * q_sp[2] * -1) - (q_meas[3] * q_sp[3] * -1);
  rotation_delta[1] = (q_meas[0] * q_sp[1]) + (q_meas[1] * q_sp[0] * -1) + (q_meas[2] * q_sp[3] * -1) - (q_meas[3] * q_sp[2] * -1);
  rotation_delta[2] = (q_meas[0] * q_sp[2]) - (q_meas[1] * q_sp[3] * -1) + (q_meas[2] * q_sp[0] * -1) + (q_meas[3] * q_sp[1] * -1);
  rotation_delta[3] = (q_meas[0] * q_sp[3]) + (q_meas[1] * q_sp[2] * -1) - (q_meas[2] * q_sp[1] * -1) + (q_meas[3] * q_sp[0] * -1);

  aa_meas[0] = 2 * acos(rotation_delta[0]);
  aa_meas[1] = rotation_delta[1] / sin(acos(aa_meas[0]));
  aa_meas[2] = rotation_delta[2] / sin(acos(aa_meas[0]));
  aa_meas[3] = rotation_delta[3] / sin(acos(aa_meas[0]));

  target_w = aa_meas[0] / (pi / 2);
/*
  hamilton_helper[0] = (q_meas[0] * ) + (q_meas[1] * ) + (q_meas[2] * ) + (q_meas[3] * );
  hamilton_helper[1] = (q_meas[0] * ) + (q_meas[1] * ) + (q_meas[2] * ) + (q_meas[3] * );
  hamilton_helper[2] = (q_meas[0] * ) + (q_meas[1] * ) + (q_meas[2] * ) + (q_meas[3] * );
  hamilton_helper[3] = (q_meas[0] * ) + (q_meas[1] * ) + (q_meas[2] * ) + (q_meas[3] * );
*///wait to implement vel
  
w_euler_target[0] = target_w * aa_meas[1];
w_euler_target[1] = target_w * aa_meas[2];
w_euler_target[2] = target_w * aa_meas[3];

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

}

void loop() {
  
  inertialUpdate();

  Serial.print(w_euler_target[0]);
  Serial.print("  ");
  Serial.print(w_euler_target[1]);
  Serial.print("  ");
  Serial.print(w_euler_target[2]);
  Serial.println(); 
  
}
