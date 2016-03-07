#ifndef _SENSOR_
#define _SENSOR_

#include "Vector3D.h"
#include "Drivers/MPU6050.h"
#include "Drivers/BMP085.h"
#include <vector>

class Sensor {
 public:
  Sensor(float sampling_rate);
  ~Sensor();

  bool initialize();

  //Obtain instaneous and integrated results (depends on update time)
  void getMotionData (Vector3D &accel_t, Vector3D &speed_t, Vector3D &angular_speed_t, Vector3D &normal_vector_t);

  //Obtain instaneous sensor data
  void getAccel (Vector3D &accel);
  void getGyro (Vector3D &angular_speed);
  float getPressure ();
  float getAltitude ();
  float GetTemp ();

  //Calibration
  void IMU_Calibrate ();
  void IMU_GetOffsets ();
  void IMU_SelfTest ();
  
 private:
  //in miliseconds
  float sampling_time;

  //LSB normalization const
  float LSB_accel;
  float LSB_gyro;
  
  //measurement data
  Vector3D accel;
  Vector3D accel_0;
  Vector3D gyro;
  Vector3D gyro_0;
  float    measure_time;
  float    measure_time_0;
  Vector3D speed;
  Vector3D normal_vector;

  //Sensors
  MPU6050 IMU;
  BMP085  TPU;
};


#endif
