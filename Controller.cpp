#include "Controller.h"
#include <math.h>
#include <bcm2835.h>
#include <stdio.h>

//default constructor - initialize class with default parameters
Controller::Controller(Data *DATA_ref) {
  DATA = DATA_ref;
  PWM_freq = 200;
  PWM_period = 1000 / PWM_freq;
  // throttle range: 1100(min) - 1900(max) us
  min_duty_cycle = 1.1 / PWM_period;
  max_duty_cycle = 1.9 / PWM_period;
  duty_range = max_duty_cycle - min_duty_cycle;
}

Controller::Controller(Data *DATA_ref, ControlParameters parameters) {
  DATA = DATA_ref;
  setParameters(parameters);
  PWM_freq = 200;
  PWM_period = 1000 / PWM_freq;
  // throttle range: 1100(min) - 1900(max) us
  min_duty_cycle = 1.0 / PWM_period;
  max_duty_cycle = 2.0 / PWM_period;
  duty_range = max_duty_cycle - min_duty_cycle;
}

Controller::~Controller() {
  servo.sleep();
}

bool Controller::initialize() {
  theta_int = 0;
  x_z = 0;
  t0 = bcm2835_st_read() / 1000000.0;
  servo.set_PWM_Frequency(PWM_freq);
  servo.initialize();
  
  printf ("Preparing to intialize ESCs, please turn the power of ESCs on.\n");
  ServoData power;
  power.UR = power.UL = power.DR = power.DL = 0.0;
  setServo(power);
  bcm2835_delay(5000);
  printf ("ESC initialization complete.\n");
  
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
  theta_int_bound = 0.2 / para.bal_int;
  x_z_bound = 0.1 / para.att_int;
}


void Controller::control (float thrust, float yaw, float yaw_set, Eigen::Vector3f g_direction, Eigen::Vector3f g_direction_set) {
  ServoData power;
  power.UR = power.UL = power.DR = power.DL = thrust * 0.8;
  
  float t = bcm2835_st_read() / 1000000.0;
  dt = t - t0;
  t0 = t;

  balanceAlg (g_direction, g_direction_set, power);
//  yawAlg (yaw, yaw_set, power);
  setServo (power);  
}

void Controller::control_HoldAtt (float z_speed, float yaw, float yaw_set, Eigen::Vector3f g_direction, Eigen::Vector3f g_direction_set) {
  ServoData power;

  float t = bcm2835_st_read() / 1000000.0;
  dt = t - t0;
  t0 = t;
  
  power.UR = power.UL = power.DR = power.DL = 0.15;
//  attAlg (z_speed, power);
  balanceAlg (g_direction, g_direction_set, power);
//  yawAlg (yaw, yaw_set, power);
  setServo (power);
}


void Controller::attAlg (float v_z, ServoData &output) {
  //attitude holding thrust formula: thrust = ca0 + ca1 v_z + ca2 a_z + ca3 x_z
  float a_z = (v_z - v_z0) / dt;
  x_z += (v_z + v_z0) * dt / 2.0;
  if (abs(x_z) > x_z_bound) {
    if (x_z < 0) {
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


void Controller::balanceAlg (Eigen::Vector3f g_dir_now, Eigen::Vector3f g_dir_set, ServoData &output) {
  //calculate angle deviation form g_dir_now and g_dir_set
  Eigen::Vector3f tmp = g_dir_set - g_dir_now;
  Eigen::Vector2f theta (asin(tmp[0]), asin(tmp[1]));
  
  //differentation and integration of theta
  theta_diff = (theta - theta_0) / dt;
  theta_int += (theta + theta_0) * dt / 2.0;

  if (abs(theta_int[0]) > theta_int_bound) {
    if (theta_int[0] < 0) {
      theta_int[0] = -theta_int_bound;
    } else {
      theta_int[0] = theta_int_bound;
    }
  }
  if (abs(theta_int[1]) > theta_int_bound) {
    if (theta_int[1] < 0) {
      theta_int[1] = -theta_int_bound;
    } else {
      theta_int[1] = theta_int_bound;
    }
  }
  theta_0 = theta;
  
  //correction vector formula:
  //f_i = (c1 theta_i + c2 dtheta_i/dt + c3 int(dtheta_i dt)
  Eigen::Vector2f f_corr = para.bal_lin*theta + para.bal_diff*theta_diff + para.bal_int*theta_int;

  //convert the correction vector to the power of motors
  if (f_corr[0] > 0) {
    output.UR += f_corr[0];
    output.DR += f_corr[0];
  } else {
    output.UL -= f_corr[0];
    output.DL -= f_corr[0];
  }
  if (f_corr[1] > 0) {
    output.UR += f_corr[1];
    output.UL += f_corr[1];
  } else {
    output.DR -= f_corr[1];
    output.DL -= f_corr[1];
  }
}


void Controller::setServo (ServoData input) {
  printf ("% f  % f  % f  % f\n", input.UR, input.UL, input.DL, input.DR);
  fflush(stdout);
  pthread_spin_lock (&(DATA->I2C_ACCESS));
  servo.setPWM (0, min_duty_cycle + input.UR * duty_range);
  servo.setPWM (1, min_duty_cycle + input.UL * duty_range);
  servo.setPWM (2, min_duty_cycle + input.DL * duty_range);
  servo.setPWM (3, min_duty_cycle + input.DR * duty_range);
  pthread_spin_unlock (&(DATA->I2C_ACCESS));
}
