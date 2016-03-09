#include "Drivers/PCA9685.h"
#include "Vector3D.h"

struct ServoData {
  float UR, UL, DL, DR;
};

class Controller {
 public:
  Controller();
  ~Controller();

  bool initialize ();

  //auto control methods
  void control (float thrust, float yaw, float yaw_set, Vector3D g_direction, Vector3D g_direction_set);
  void control_HoldAtt (float z_speed ,float yaw, float yaw_set, Vector3D g_direction, Vector3D g_direction_set);
  void control_hover (float yaw, float yaw_set, Vector3D speed);
    
 private:
  //algorithm and hardware control methods
  void attAlg (float v_z, ServoData &output);
  void yawAlg (float yaw_now, float yaw_set, ServoData &output);
  void balanceAlg (Vector3D g_dir_now, Vector3D g_dir_set, ServoData &output);
  void setServo (ServoData input);
  
  //algorithm data & parameters
  float c_int, c_con, c_diff, c_lin;
  float c_yaw_lin, c_yaw_diff;
  float theta_0, yaw_0, t0, dt;
  float theta_int;
  
  PCA9685 servo;
};
