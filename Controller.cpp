#include "Controller.h"
#include <math.h>
#include <bcm2835.h>

Controller::Controller() { }

Controller::~Controller() {
  servo.sleep();
}


bool Controller::initialize() {
  servo.initialize();
  return 1;
}


void Controller::control(float thrust, float yaw, float yaw_set, Vector3D g_direction, Vector3D g_direction_set) {
  ServoData power;
  power.UR = power.UL = power.DR = power.DL = thrust;
  
  float t = bcm2835_st_read() / 1000000;
  dt = t - t0;
  t0 = t;

  balanceAlg (g_direction, g_direction_set, power);
  yawAlg (yaw, yaw_set, power);
  setServo (power);  
}


void Controller::yawAlg (float yaw_now, float yaw_set, ServoData &output) {
  float yaw = yaw_now - yaw_set;
  //yaw correction formula: y_corr = cy1 y + cy2 dy/dt
  float yaw_corr = c_yaw_lin * yaw + c_yaw_diff * ((yaw-yaw_0)/dt);
  yaw_0 = yaw;

  //convert yaw_corr data to power of motors
  output.UL += yaw_corr;
  output.DR += yaw_corr;
  output.UR -= yaw_corr;
  output.DL -= yaw_corr;
}


void Controller::balanceAlg (Vector3D g_dir_now, Vector3D g_dir_set, ServoData &output) {
  //calculate angle deviation form pendulum line
  Vector3D z (0,0,-1);
  float theta_g0 = g_dir_now.getAngle (z);
  //calculate angle deviation form g_dir_now and g_dir_set
  float theta = g_dir_set.getAngle (g_dir_now);

  //differentation and integration of theta
  float theta_diff = (theta - theta_0) / dt;
  theta_int += (theta + theta_0) * dt / 2;
  theta_0 = theta;
  
  //correction vector formula: (^ stands for unit vector in x-y plane)
  //f = -c0 sin(theta_g0) ^(g_now - g0) - (c1 theta + c2 dtheta/dt + c3 int(dtheta dt)) ^(g_set - g_now) 
  Vector3D tmp = g_dir_now - z;
  tmp.z = 0;
  tmp.normalize();
  Vector3D f_corr = -c_con * sin(theta_g0) * tmp;
  tmp = g_dir_set - g_dir_now;
  tmp.z = 0;
  tmp.normalize();
  f_corr = f_corr - (c_lin*theta + c_diff*theta_diff + c_int*theta_int) * tmp;
  
  //convert the correction vector to the power of motors
  output.UR += f_corr.x + f_corr.y;
  output.DR += f_corr.x - f_corr.y;
  output.UL += -f_corr.x + f_corr.y;
  output.DL += -f_corr.x - f_corr.y;
}


void Controller::setServo (ServoData input) {
  #pragma omp critical (I2C_access)
  {
  servo.setPWM (0, input.UR);
  servo.setPWM (1, input.UL);
  servo.setPWM (2, input.DL);
  servo.setPWM (3, input.DR);
  }
}
