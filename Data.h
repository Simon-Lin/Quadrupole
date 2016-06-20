#ifndef _DATA_
#define _DATA_

//Data structure shared by Sensor/Controller/Interface instances

#include <pthread.h>
#include "Eigen/Dense"

struct Data {
  
  //measurement data
  Eigen::Vector3f acceleration, speed, angular_speed, g_direction;
  float pressure, temperature, batt_voltage, altitute;

  //control signal
  Eigen::Vector3f g_direction_set;
  float throttle, yaw_set;
  bool startup_lock, att_hold, power_off;

  //process control
  bool terminate;
  pthread_spinlock_t I2C_ACCESS;

};

#endif
