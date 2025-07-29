#include <SPI.h>
#include <SD.h>
#include <SoftTimer.h>

const int chipSelect = 10;

#include <Servo.h>
Servo actuatorX;
Servo actuatorY;

#include "NineAxesMotion.h"        //Contains the bridge code between the API and the Arduino Environment
#include <Wire.h>

NineAxesMotion mySensor;         //Object that for the sensor 

float pi = 3.14159;

//rocket state and response setup
float servoInputX;
float servoInputY;
double motorAngleX;
double motorAngleY;
double setpointX = 0;
double setpointY = 0;
double vehicleAngleX;
double vehicleAngleY;
double vehicleAngleZ;

//control parameters
double p = 0.25;
double i = 0;
double d = 0.6;

//timing
  double launchTimeStamp;
  double timeSinceLiftoff = 0;
  Task t1(200, callBack1);

  void callBack1(Task* me) {

    //MASTER RESPONSE FUNCTION; all these functions run at the same rate in the same interval

    fetchSensor();
    outputServos();
    dataOut();

    Serial.println("Cycle");

  }








void setup() {

  //configuration for flight-readiness
  File dataFile = SD.open("datalog.txt", FILE_WRITE);
  Serial.begin(115200);

  actuatorX.attach(9);
  actuatorY.attach(6);

//sensor setup
  I2C.begin();                    //Initialize I2C communication to the let the library communicate with the sensor.
  //Sensor Initialization
  mySensor.initSensor();          //The I2C Address can be changed here inside this function in the library
  mySensor.setOperationMode(OPERATION_MODE_NDOF);   //Can be configured to other operation modes as desired
  mySensor.setUpdateMode(MANUAL);	//The default is AUTO. Changing to manual requires calling the relevant update functions prior to calling the read functions
  //Setting to MANUAL requires lesser reads to the sensor
  mySensor.updateAccelConfig();


  //setup has ended, flight loop is active


  Serial.println("Pre flight");

//wait for launch
  fetchSensor();
  while (mySensor.readLinearAcceleration(Y_AXIS) > -1) {
    //waiting for launch loop
    fetchSensor();
  }

  Serial.println("In flight");
  launchTimeStamp = millis(); //exact moment of liftoff

  //sensor and response loop
  SoftTimer.add(&t1);

  //flight loop time based on time after launch; this holds the setup function to wait for burnout
  delay(1900);

  //unpowered flight
  //idk


  //post-flight
  SoftTimer.remove(&t1);
  dataFile.close();
  Serial.println("Post flight");
}













void outputServos() {
  
  servoInputX = (0.0171 * motorAngleX * motorAngleX) - (2.4744 * motorAngleX) + 99;
  servoInputY = (0.0171 * motorAngleY * motorAngleY) - (2.4744 * motorAngleY) + 93;
  actuatorX.write(servoInputX);
  actuatorY.write(servoInputY);
  
}

void fetchSensor() {

  // refreshing sensor values

    mySensor.updateEuler();
    mySensor.updateGyro();   
    mySensor.updateAccel();        //Update the Accelerometer data
    mySensor.updateLinearAccel();  //Update the Linear Acceleration data
    mySensor.updateGravAccel();    //Update the Gravity Acceleration data
    mySensor.updateCalibStatus();  //Update the Calibration Status

    vehicleAngleX = mySensor.readEulerRoll();
    vehicleAngleY = mySensor.readEulerPitch() - 90;
    vehicleAngleZ = mySensor.readEulerHeading();

}

void dataOut() {

    Serial.print(millis());
    Serial.print("          ");
    Serial.println(motorAngleX);

}

void responseDynamics() {

  //PD loop configuration and motor delay
  motorAngleX = -(p * vehicleAngleX) - (d * mySensor.readGyroZ() * -1);
  motorAngleY = -(p * vehicleAngleY) - (d * mySensor.readGyroX() * -1);

  //actuation limit modeling
  
  if (motorAngleX > 12) {
    motorAngleX = 12;
  }
  else if (motorAngleX < -20) {
    motorAngleX = -20;
  }
  else {
    motorAngleX = motorAngleX;
  }


  if (motorAngleY > 12) {
    motorAngleY = 12;
  }
  else if (motorAngleY < -20) {
    motorAngleY = -20;
  }
  else {
    motorAngleY = motorAngleY;
  }  

  //use time since launch for gain scheduling later

}

//loop does not happen