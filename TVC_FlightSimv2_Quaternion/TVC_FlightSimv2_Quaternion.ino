//simulation constants
const float pi = 3.14159;
const float timeStep = 0.01; //1000Hz

//orientation info
float q_vel[4] = {0,0,0,0};
float q_meas[4] = {0,0,0,0};
float q_err[4] = {0,0,0,0};
float q_err_SHORT[4] = {0,0,0,0};
float q_sp[4] = {0.5,0.5,0.5,0.5}; //constant SP for now
float w_[3] = {0,0,0}; //angular velocity
float a_[3] = {0,0,0}; //angular acceleration

//torque control solvers
float torque[3] = {1,1,1};
const float CM_TVC = 0.089; 
float thrust = 1; //newtons?
const float I[3] = {0.1,0.1,0.1};

//math interim helpers
const float I_inverse[3] = {10,10,10};
float inertia_by_velocity[3] = {0,0,0};
float a_previous[3] = {0,0,0};

//guidance gains
float Pq[3] = {1,1,1};
float Pw[3] = {1,1,1};

//output settings
float PS = 0; //pitch servo
float RS = 0; //roll servo
float true_PS = 0; 
float true_RS = 0; 






void thrustCurve() {
  
}

void controlUpdate() {

  //control system
  torque[0] = (-1 * Pq[0] * q_err_SHORT[1]) - (Pw[0] * w_[0]);
  torque[1] = (-1 * Pq[1] * q_err_SHORT[2]) - (Pw[1] * w_[1]);
  torque[2] = (-1 * Pq[2] * q_err_SHORT[3]) - (Pw[2] * w_[2]);

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

}

void physicsEngine() { //inertial update equivalent

  //inertia helper
  inertia_by_velocity[0] = I[0] * w_[0];
  inertia_by_velocity[1] = I[1] * w_[1];
  inertia_by_velocity[2] = I[2] * w_[2];

  //angular acceleration vector calcs
  a_[0] = (I_inverse[0] * torque[0]) - (I_inverse[0] * ((w_[1] * inertia_by_velocity[2]) - (w_[2] * inertia_by_velocity[1])));
  a_[1] = (I_inverse[1] * torque[1]) - (I_inverse[1] * ((w_[2] * inertia_by_velocity[0]) - (w_[0] * inertia_by_velocity[2])));
  a_[2] = (I_inverse[2] * torque[2]) - (I_inverse[2] * ((w_[0] * inertia_by_velocity[1]) - (w_[1] * inertia_by_velocity[0])));

  //angular acceleration integrator
  w_[0] = (a_[0] + a_previous[0]) * 0.5 * timeStep + w_[0];
  w_[1] = (a_[1] + a_previous[1]) * 0.5 * timeStep + w_[1];
  w_[2] = (a_[2] + a_previous[2]) * 0.5 * timeStep + w_[2];
  
  a_previous[0] = a_[0];
  a_previous[1] = a_[1];
  a_previous[2] = a_[2];

  //velocity quaternion calcs
  q_vel[0] = (-0.5 * 0 * q_meas[0]) + (-0.5 * w_[0] * q_meas[1]) + (-0.5 * w_[1] * q_meas[2]) + (-0.5 * w_[2] * q_meas[3]);
  q_vel[1] = (-0.5 * 0 * q_meas[1]) + (-0.5 * w_[0] * q_meas[0]) + (-0.5 * w_[1] * q_meas[3]) + (-0.5 * w_[2] * q_meas[2]);
  q_vel[2] = (-0.5 * 0 * q_meas[2]) + (-0.5 * w_[0] * q_meas[3]) + (-0.5 * w_[1] * q_meas[0]) + (-0.5 * w_[2] * q_meas[1]);
  q_vel[3] = (-0.5 * 0 * q_meas[3]) + (-0.5 * w_[0] * q_meas[2]) + (-0.5 * w_[1] * q_meas[1]) + (-0.5 * w_[2] * q_meas[0]);

  //quaternion integrator/adder
  q_meas[0] = q_meas[0] + (q_vel[0] * timeStep);
  q_meas[1] = q_meas[1] + (q_vel[1] * timeStep);
  q_meas[2] = q_meas[2] + (q_vel[2] * timeStep);
  q_meas[3] = q_meas[3] + (q_vel[3] * timeStep);

  //quaternion normalize
  q_meas[0] = q_meas[0] / pow(pow(q_meas[0], 2) + pow(q_meas[1], 2) + pow(q_meas[2], 2) + pow(q_meas[3], 2), 0.5);
  q_meas[1] = q_meas[1] / pow(pow(q_meas[0], 2) + pow(q_meas[1], 2) + pow(q_meas[2], 2) + pow(q_meas[3], 2), 0.5);
  q_meas[2] = q_meas[2] / pow(pow(q_meas[0], 2) + pow(q_meas[1], 2) + pow(q_meas[2], 2) + pow(q_meas[3], 2), 0.5);
  q_meas[3] = q_meas[3] / pow(pow(q_meas[0], 2) + pow(q_meas[1], 2) + pow(q_meas[2], 2) + pow(q_meas[3], 2), 0.5);
  
}





void setup() {
  
  Serial.begin(115200);

  Serial.println();
  Serial.println("Quaternion Stabiltiy FLight Simulation");
  Serial.println();

}

void loop() {

  Serial.print(q_meas[0]);
  Serial.print("  ");
  Serial.print(q_meas[1]);
  Serial.print("  ");
  Serial.print(q_meas[2]);
  Serial.print("  ");
  Serial.print(q_meas[3]);
  Serial.println();

  controlUpdate();
  physicsEngine();

}
