#ifndef _CONTROLLER_
#define _CONTROLLER_

#include "Drivers/PCA9685.h"
#include "Eigen/Dense"
#include "Data.h"
#include <pthread.h>

struct ServoData {
  float UR, UL, DL, DR;
};

struct ControlParameters {
  float bal_lin, bal_diff, bal_int;
  float yaw_lin, yaw_diff;
  float att_con, att_lin, att_diff, att_int;
};

class Controller {
 public:
  Controller(Data *DATA_ref);
  Controller(Data *DATA_ref, ControlParameters parameters);
  ~Controller();

  bool initialize ();
  
  //set autocontrol formula paremeters
  void setParameters (ControlParameters parameters);
  
  //auto control methods
  void control (float thrust, float yaw, float yaw_set, Eigen::Vector3f g_direction, Eigen::Vector3f g_direction_set);
  void control_HoldAtt (float z_speed, float yaw, float yaw_set, Eigen::Vector3f g_direction, Eigen::Vector3f g_direction_set);
  void control_Hover (float yaw, float yaw_set, Eigen::Vector3f speed);

  void setServo (ServoData input);
  
 private:
  Data *DATA;
  
  //algorithm and hardware control methods
  void attAlg (float v_z, ServoData &output);
  void yawAlg (float yaw_now, float yaw_set, ServoData &output);
  void balanceAlg (Eigen::Vector3f g_dir_now, Eigen::Vector3f g_dir_set, ServoData &output);
  
  //algorithm data & parameters
  ControlParameters para;
  Eigen::Vector2f theta_0, theta_diff, theta_int;
  float yaw_0, v_z0, t0, dt;
  float x_z, theta_int_bound, x_z_bound;
  float PWM_freq, PWM_period, max_duty_cycle, min_duty_cycle, duty_range;

  PCA9685 servo;
};

#endif
