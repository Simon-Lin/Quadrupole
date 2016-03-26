#include "Controller.h"
#include <math.h>
#include <bcm2835.h>
#include <stdio.h>

//default constructor - initialize class with default parameters
Controller::Controller() {
  
}

Controller::Controller(ControlParameters parameters) {
  setParameters(parameters);
}

Controller::~Controller() {
  servo.sleep();
}

bool Controller::initialize() {
  theta_int = 0;
  x_z = 0;
  t0 = bcm2835_st_read() / 1000000.0;
  servo.initialize();
  return 1;
}

void Controller::setParameters(ControlParameters parameters) {
  para.bal_lin = parameters.bal_lin;
  para.bal_diff = parameters.bal_diff;
  para.bal_int = parameters.bal_int;
  para.yaw_lin = parameters.yaw_lin;
  para.yaw_diff = parameters.yaw_diff;
  para.att_con = parameters.att_con;
  para.att_lin = parameters.att_lin;
  para.att_diff = parameters.att_diff;
  para.att_int = parameters.att_int;

  //set boundary values to integrated results
  theta_int_bound = abs(1/para.bal_int);
  x_z_bound = abs(1/para.att_int);
}


void Controller::control (float thrust, float yaw, float yaw_set, Vector3D g_direction, Vector3D g_direction_set) {
  ServoData power;
  power.UR = power.UL = power.DR = power.DL = thrust;
  
  float t = bcm2835_st_read() / 1000000.0;
  dt = t - t0;
  t0 = t;

  balanceAlg (g_direction, g_direction_set, power);
  yawAlg (yaw, yaw_set, power);
  setServo (power);  
}

void Controller::control_HoldAtt (float z_speed, float yaw, float yaw_set, Vector3D g_direction, Vector3D g_direction_set) {
  ServoData power;

  float t = bcm2835_st_read() / 1000000.0;
  dt = t - t0;
  t0 = t;

  attAlg (z_speed, power);
  balanceAlg (g_direction, g_direction_set, power);
  yawAlg (yaw, yaw_set, power);
  setServo (power);
}


void Controller::attAlg (float v_z, ServoData &output) {
  //attitude holding thrust formula: thrust = ca0 + ca1 v_z + ca2 a_z + ca3 x_z
  float a_z = (v_z - v_z0) / dt;
  x_z += (v_z + v_z0) * dt / 2.0;
  if (abs(x_z) > x_z_bound) {
    if (signbit(x_z) ) {
      x_z = -x_z_bound;
    } else {
      x_z = x_z_bound;
    }
  }
  
  float thrust = para.att_con + para.att_lin * v_z + para.att_diff * a_z + para.att_int * x_z;
  v_z0 = v_z;
  output.UL = output.UR = output.DL = output.DR = thrust;
}


void Controller::yawAlg (float yaw_now, float yaw_set, ServoData &output) {
  float yaw = yaw_now - yaw_set;
  //yaw correction formula: y_corr = cy1 y + cy2 dy/dt
  float yaw_corr = para.yaw_lin * yaw + para.yaw_diff * ((yaw-yaw_0)/dt);
  yaw_0 = yaw;

  //convert yaw_corr data to power of motors
  output.UL += yaw_corr;
  output.DR += yaw_corr;
  output.UR -= yaw_corr;
  output.DL -= yaw_corr;
}


void Controller::balanceAlg (Vector3D g_dir_now, Vector3D g_dir_set, ServoData &output) {
  //calculate angle deviation form g_dir_now and g_dir_set
  float theta = g_dir_set.getAngle (g_dir_now);

  //differentation and integration of theta
  float theta_diff = (theta - theta_0) / dt;
  theta_int += (theta + theta_0) * dt / 2.0;
  if (abs(theta_int) > theta_int_bound) {
    if (signbit(theta_int) ) {
      theta_int = -theta_int_bound;
    } else {
      theta_int = theta_int_bound;
    }
  }
  theta_0 = theta;
  
  //correction vector formula: (^ stands for unit vector in x-y plane)
  //f = - (c1 theta + c2 dtheta/dt + c3 int(dtheta dt)) ^(g_set - g_now) 
  Vector3D tmp = g_dir_set - g_dir_now;
  tmp.z = 0;
  tmp.normalize();
  Vector3D  f_corr = - (para.bal_lin*theta + para.bal_diff*theta_diff + para.bal_int*theta_int) * tmp;

  //convert the correction vector to the power of motors
  output.UR += f_corr.x + f_corr.y;
  output.DR += f_corr.x - f_corr.y;
  output.UL += -f_corr.x + f_corr.y;
  output.DL += -f_corr.x - f_corr.y;
}


void Controller::setServo (ServoData input) {  
  pthread_spin_lock (&I2C_ACCESS);
  servo.setPWM (0, input.UR);
  servo.setPWM (1, input.UL);
  servo.setPWM (2, input.DL);
  servo.setPWM (3, input.DR);
  pthread_spin_unlock (&I2C_ACCESS);
}
