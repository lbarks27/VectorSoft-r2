#include <SPI.h>
#include <SD.h>

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
float vehicleAngleX;
float vehicleAngleY;
float vehicleAngleZ;
int flightState = 0;

//control parameters
double p = 0.15;
double i = 0;
double d = 0.65;

//timing
unsigned long launchTimeStamp;
unsigned long timeSinceLiftoff = 0;
//unsigned long lastStreamTime_dataLoop = 0;     //To store the last streamed time stamp
//const int streamPeriod_dataLoop = 100;          //ms

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

  File dataFile = SD.open("datalog.txt", FILE_WRITE);

    dataFile.print(millis());
    dataFile.print("                    ");
    dataFile.print(timeSinceLiftoff);
    dataFile.print("                    ");
    dataFile.print(motorAngleX);
    dataFile.print("                    ");
    dataFile.print(motorAngleY);
    dataFile.print("                    ");
    dataFile.print(vehicleAngleX);
    dataFile.print("                    ");
    dataFile.print(vehicleAngleY);
    dataFile.print("                    ");
    dataFile.print(vehicleAngleZ);
    dataFile.print("                    ");
    dataFile.print(mySensor.readGyroX());
    dataFile.print("                    ");
    dataFile.print(mySensor.readGyroY());
    dataFile.print("                    ");
    dataFile.print(mySensor.readGyroZ());
    dataFile.print("                    ");
    dataFile.print(mySensor.readAccelerometer(X_AXIS));
    dataFile.print("                    ");
    dataFile.print(mySensor.readAccelerometer(Y_AXIS));
    dataFile.print("                    ");
    dataFile.print(mySensor.readAccelerometer(Z_AXIS));
    dataFile.print("                    ");
    dataFile.print(mySensor.readLinearAcceleration(X_AXIS));
    dataFile.print("                    ");
    dataFile.print(mySensor.readLinearAcceleration(Y_AXIS));
    dataFile.print("                    ");
    dataFile.print(mySensor.readLinearAcceleration(Z_AXIS));
    dataFile.print("                    ");
    dataFile.print(mySensor.readGravAcceleration(X_AXIS));
    dataFile.print("                    ");
    dataFile.print(mySensor.readGravAcceleration(Y_AXIS));
    dataFile.print("                    ");
    dataFile.print(mySensor.readGravAcceleration(Z_AXIS));
    dataFile.print("                    ");
    dataFile.print(mySensor.readAccelCalibStatus());
    dataFile.print("                    ");
    dataFile.print(mySensor.readMagCalibStatus());
    dataFile.print("                    ");
    dataFile.print(mySensor.readGyroCalibStatus());
    dataFile.print("                    ");
    dataFile.print(mySensor.readSystemCalibStatus());
    dataFile.print("                    ");
    dataFile.print(flightState);
    dataFile.println("                    ");

  dataFile.close();

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
  mySensor.setUpdateMode(MANUAL);  //The default is AUTO. Changing to manual requires calling the relevant update functions prior to calling the read functions
  //Setting to MANUAL requires lesser reads to the sensor
  mySensor.updateAccelConfig();


  //setup has ended, flight loop is active
  dataFile.println("FLIGHT TEST");
  dataOut();
  dataFile.close();
  delay(1);
  flightState = 1;

  Serial.println("Pre flight");

  //wait for launch
  fetchSensor();
  while (mySensor.readLinearAcceleration(Y_AXIS) > -1) {
    //waiting for launch loop
    fetchSensor();
    dataOut();
  }

  flightState = 2;
  Serial.println("In flight");
  launchTimeStamp = millis(); //exact moment of liftoff

  while (timeSinceLiftoff < 2000) {
    
    timeSinceLiftoff = millis() - launchTimeStamp;
    
    fetchSensor();
    responseDynamics();
    outputServos();
    dataOut();
    
  }

  //unpowered flight
  flightState = 3;
  Serial.println("Unpowered flight");
  motorAngleX = 0;
  motorAngleY = 0;
  outputServos();
  dataFile.close();
}

void loop() {
  
}
