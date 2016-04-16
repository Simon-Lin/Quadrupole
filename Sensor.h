#ifndef _SENSOR_
#define _SENSOR_

#include "Data.h"
#include "Drivers/MPU6050.h"
#include "Drivers/BMP085.h"
#include "Drivers/PCF8591.h"
#include <pthread.h>

class Sensor {
 public:
  Sensor(float sampling_rate);
  Sensor(Data *DATA_ref, float sampling_rate);
  ~Sensor();

  bool initialize();

  //Start data collection and related calculation.
  //Update the results peridocally according to sampling rate.
  //Should only be called with pthread_create()
  static void* start(void *context);
  
  //Obtain instaneous and integrated results
  void getMotionData (Eigen::Vector3f &angular_direction, Eigen::Vector3f &g_direction);
  
  //Obtain instaneous sensor data
  void getAccel (Eigen::Vector3f &acceleration);
  void getGyro (Eigen::Vector3f &angular_speed);
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
  Data *DATA;

  //pthread start wraps to here
  void* _start();
  
  //in miliseconds
  float sampling_time;

  //LSB normalization const
  float LSB_accel;
  float LSB_gyro;
  
  //measurement data
  float gyro_diff;
  float time;
  float time_0;

  //Kalman filter data
  //state vector (gdir_x, gdir_y, gdir_z, gz, gy, gz)
  Eigen::Matrix<float, 6, 1> state, state_observ, state_predict;
  Eigen::Matrix<float, 6, 6> cor, cor_predict;
  Eigen::Matrix<float, 6, 6> noise_predict, noise_observ;
    
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
