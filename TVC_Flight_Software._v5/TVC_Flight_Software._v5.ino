#include "NineAxesMotion.h"        //Contains the bridge code between the API and the Arduino Environment
#include <Wire.h>
#include <Servo.h>
#include "math.h"
#include <SPI.h>
#include <SD.h>

const int chipSelect = 10;

NineAxesMotion mySensor;         //Object that for the sensor 
Servo R;
Servo P;

float PS_zero = 55;
float RS_zero = 80;

float PS = 0; //pitch output deg
float RS = 0; //roll output deg

const float pi = 3.142;
long liftoffTS = 0; //time since liftoff will be millis - liftoffTS
float arcsinHelper = 0;
int flightState = 1; //pad idle (1), active flight (2), inactive flight (3), parachutes (4), landed (5)

//position info
float accel_meas[3];

//orientation info
float q_meas[4]; //wxyz
float euler_meas[3]; //pyr
float gyro_meas[3]; //pyr

//acceleration control solver
float m = 0.505; //kg
float accel_sp[2]; //pitch, roll
float I = 0.00545; //kgm^2
const float T_C = 0.11; //meters
float thrust = 5; //newtons, figure it out on flight

//float euler_sp[3] = {0,0,0}; //pitch, yaw, roll
//float euler_error[3]; will implement for guidance

//guidance gains for pitch and roll axis
float KP[3] = {0.35,0,0.65}; //PID
float KR[3] = {0.35,0,0.65}; //PID

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void servo() {

//actuator hard limits
   if (PS > 10) {
    PS = 10;
  }
  else if (PS < -10) {
    PS = -10;
  }
  else {
    PS = PS;
  }

   if (RS > 3) {
    RS = 3;
  }
  else if (RS < -3) {
    RS = -3;
  }
  else {
    RS = RS;
  }
  
  P.write((4 * PS) + PS_zero);
  R.write((4.546 * RS * -1) + RS_zero);
  
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void guidance() {
  //vector control in future :)
}

void navigation() {

  //sensor rebase
  mySensor.updateEuler();
  mySensor.updateGyro();
  mySensor.updateQuat();
  mySensor.updateAccel();

  //quaternions for post-flight analysis
  q_meas[0] = mySensor.readQuatW() / pow(pow(mySensor.readQuatW(), 2) + pow(mySensor.readQuatX(), 2) + pow(mySensor.readQuatY(), 2) + pow(mySensor.readQuatZ(), 2), 0.5);
  q_meas[1] = mySensor.readQuatX() / pow(pow(mySensor.readQuatW(), 2) + pow(mySensor.readQuatX(), 2) + pow(mySensor.readQuatY(), 2) + pow(mySensor.readQuatZ(), 2), 0.5);
  q_meas[2] = mySensor.readQuatY() / pow(pow(mySensor.readQuatW(), 2) + pow(mySensor.readQuatX(), 2) + pow(mySensor.readQuatY(), 2) + pow(mySensor.readQuatZ(), 2), 0.5);
  q_meas[3] = mySensor.readQuatZ() / pow(pow(mySensor.readQuatW(), 2) + pow(mySensor.readQuatX(), 2) + pow(mySensor.readQuatY(), 2) + pow(mySensor.readQuatZ(), 2), 0.5);

  euler_meas[0] = mySensor.readEulerPitch();
  euler_meas[1] = mySensor.readEulerHeading();
  euler_meas[2] = mySensor.readEulerRoll();

  gyro_meas[0] = mySensor.readGyroX();
  gyro_meas[1] = mySensor.readGyroY();
  gyro_meas[2] = mySensor.readGyroZ();

  accel_meas[0] = mySensor.readAccelerometer(X_AXIS);
  accel_meas[1] = mySensor.readAccelerometer(Y_AXIS);
  accel_meas[2] = mySensor.readAccelerometer(Z_AXIS);
  
}

void control() {

  //thrust calc
  thrust = accel_meas[1] * m;

  //main PD loop
  accel_sp[0] = (KP[0] * (euler_meas[0] +  90) * -1) + (KP[2] * gyro_meas[0]);
  accel_sp[1] = (KR[0] * euler_meas[2] * -1) - (KR[2] * gyro_meas[2]);

  //accel to motor angle
  arcsinHelper = accel_sp[0] * I / (thrust * T_C);

   if (arcsinHelper > 1) {
    arcsinHelper = 0.9;
  }
  else if (arcsinHelper < -1) {
    arcsinHelper = -0.9;
  }
  else {
    arcsinHelper = arcsinHelper;
  }
  
  PS = (180 / pi) * asin(arcsinHelper);

  arcsinHelper = accel_sp[1] * I / (thrust * T_C);

   if (arcsinHelper > 1) {
    arcsinHelper = 0.9;
  }
  else if (arcsinHelper < -1) {
    arcsinHelper = -0.9;
  }
  else {
    arcsinHelper = arcsinHelper;
  }
  
  RS = (180 / pi) * asin(arcsinHelper);

  servo();
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void dataLogger() { //configure for csv file
  
  Serial.print(millis());
  Serial.print(",");
  Serial.print(flightState);
  Serial.print(",");
  Serial.print(q_meas[0]);
  Serial.print(",");
  Serial.print(q_meas[1]);
  Serial.print(",");
  Serial.print(q_meas[2]);
  Serial.print(",");
  Serial.print(q_meas[3]);
  Serial.print(",");
  Serial.print(euler_meas[0]);
  Serial.print(",");
  Serial.print(euler_meas[1]);
  Serial.print(",");
  Serial.print(euler_meas[2]);
  Serial.print(",");
  Serial.print(gyro_meas[0]);
  Serial.print(",");
  Serial.print(gyro_meas[1]);
  Serial.print(",");
  Serial.print(gyro_meas[2]);
  Serial.print(",");
  Serial.print(accel_meas[0]);
  Serial.print(",");
  Serial.print(accel_meas[1]);
  Serial.print(",");
  Serial.print(accel_meas[2]);
  Serial.print(",");
  Serial.print(accel_sp[0]);
  Serial.print(",");
  Serial.print(accel_sp[1]);
  Serial.print(",");
  Serial.print(PS);
  Serial.print(",");
  Serial.print(RS);
  Serial.print(",");
  Serial.print(I);
  Serial.print(",");
  Serial.print(thrust);
  Serial.print(",");
  Serial.println(T_C);
  
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void valueCurves() { //use polynomials, much more processor-friendly
  
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void setup() {

  //Serial.begin(115200);           //Initialize the Serial Port to view information on the Serial Monitor
  //Serial.flush();
  I2C.begin();                    //Initialize I2C communication to the let the library communicate with the sensor.
  //Sensor Initialization
  mySensor.initSensor();          //The I2C Address can be changed here inside this function in the library
  mySensor.setOperationMode(OPERATION_MODE_M4G);
  mySensor.setUpdateMode(MANUAL);  //The default is AUTO. Changing to MANUAL requires calling the relevant update functions prior to calling the read functions
  //Setting to MANUAL requires fewer reads to the sensor


  Serial.println("T+boot,flightState,QuatW,QuatX,QuatY,QuatZ,Pitch,Yaw,Roll,dPitch,dYaw,dRoll,ddX,ddY,ddZ,ddSP_Pitch,ddSP_Yaw,PS,RS,MMOI,thrust,TVC-CM"); //d is derivative
  

  R.attach(6);
  P.attach(9);
  //zeros, R corresponds to last number of O
  R.write(RS_zero); 
  P.write(PS_zero);

  while (accel_meas[1] < 11.5) {
    navigation();
    delay(1);
  }
}

void loop() {
  navigation();
  control();
  delay(1);

}
