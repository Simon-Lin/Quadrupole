//User Interface and remote communication class
#include <ncurses.h>
#include <linux/joystick.h>
#include "Vector3D.h"

struct InterfaceData {
  //control signal revieced from user
  float throttle, yaw_set;
  Vector3D g_direction_set;
  bool att_hold;

  //data passed to the UI display
  float pressure, temperature, height;
  Vector3D acceleration, speed, angular_speed, g_direction;
};

class Interface {
 public:
  Interface(InterfaceData *DATA_ref);
  ~Interface();

  bool initialize();
  void start(); // Used with OpenMP parallellism feature. not realized for now.
  void update();
  void joystickTest();
  
 private:
  InterfaceData *DATA;

  //joystick data
  int js_fd;
  js_event js;
  bool button[32];
  int16_t axis[8];
  
  int row_max, col_max;
};
