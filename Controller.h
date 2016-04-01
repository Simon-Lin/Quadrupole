#include "Drivers/PCA9685.h"
#include "Vector3D.h"
#include <pthread.h>

extern pthread_spinlock_t I2C_ACCESS;

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
  Controller();
  Controller(ControlParameters parameters);
  ~Controller();

  bool initialize ();
  
  //set autocontrol formula paremeters
  void setParameters (ControlParameters parameters);
  
  //auto control methods
  void control (float thrust, float yaw, float yaw_set, Vector3D g_direction, Vector3D g_direction_set);
  void control_HoldAtt (float z_speed, float yaw, float yaw_set, Vector3D g_direction, Vector3D g_direction_set);
  void control_Hover (float yaw, float yaw_set, Vector3D speed);
    
 private:
  //algorithm and hardware control methods
  void attAlg (float v_z, ServoData &output);
  void yawAlg (float yaw_now, float yaw_set, ServoData &output);
  void balanceAlg (Vector3D g_dir_now, Vector3D g_dir_set, ServoData &output);
  void setServo (ServoData input);
  
  //algorithm data & parameters
  ControlParameters para;
  float theta_0, yaw_0, v_z0, t0, dt;
  float theta_int, x_z, theta_int_bound, x_z_bound;

  PCA9685 servo;
};
