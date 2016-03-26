#ifndef _SENSOR_
#define _SENSOR_

#include "Vector3D.h"
#include "Drivers/MPU6050.h"
#include "Drivers/BMP085.h"
#include "Drivers/PCF8591.h"
#include <vector>

struct SensorData {
  //instaneous measurement data
  Vector3D acceleration, angular_speed;
  float pressure, temperature, batt_voltage;
  //integrated data
  Vector3D speed, position, g_direction;
};

class Sensor {
 public:
  Sensor(float sampling_rate);
  Sensor(SensorData *DATA_ref, float sampling_rate);
  ~Sensor();

  bool initialize();

  //Start data collection and related calculation.
  //Update the results peridocally according to sampling rate.
  //Should only be called within an OpenMP section.
  void start(bool &terminate);
  
  //Obtain instaneous and integrated results
  void getMotionData (Vector3D &acceleration, Vector3D &speed, Vector3D &position, Vector3D &angular_speed, Vector3D &g_direction);
  
  //Obtain instaneous sensor data
  void getAccel (Vector3D &acceleration);
  void getGyro (Vector3D &angular_speed);
  float getPressure ();
  float getAltitude ();
  float getTemperature ();
  float getBattVoltage ();

  //Calibration
  void accelCalibrate();
  void gyroCalibrate();
  void IMU_GetOffsets ();
  void IMU_SelfTest ();
  
 private:
  //interface data
  SensorData *DATA;
  
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
  float    time;
  float    time_0;
  Vector3D velo;
  Vector3D velo_0;
  Vector3D pos;
  Vector3D g_dir;

  //Sensors
  MPU6050 IMU;
  BMP085  TPU;
  PCF8591 ADC;

  bool TPU_connected, ADC_connected;
  float ref_voltage;
  
  //utility
  void fixOffset(float &cal_1, float &cal_0, float &off, float &fix);
};


#endif
