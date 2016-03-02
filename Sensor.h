#ifndef _SENSOR_
#define _SENSOR_

#include "Vector3D.h"
#include "Drivers/MPU6050.h"
#include "Drivers/BMP080.h"
#include <Vector>

class Sensor() {
 public:
  Sensor(double update_rate, double sampling_rate); //Initialize sensor
  ~Sensor();

  //Obtain instaneous and integrated results (depends on update time)
  bool getMotionData (Vector3D &accel, Vector3D &speed, Vector3D &angular_speed, Vector3D &normal_vector, Vector3D &front_vector);
  
  //Obtain instaneous sensor data
  bool getAccel (Vector3D &accel);
  bool getGyro (Vector3D &angular_speed);
  double getPressure ();
  double GetTemp ();
  
 private:
  std::vector<Vector3D> accel_buffer;
  std::vector<Vector3D> angular_buffer;
  std::vector<double>   measure_time;
  
  MPU6050 IMU;
  BMP080  TPU;

  void calibrate();
  void integrate_accel();
  void integrate_gyro();
};


#endif
