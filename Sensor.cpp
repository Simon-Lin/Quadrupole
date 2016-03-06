#include "Sensor.h"
#include <iostream>
#include <math.h>

//default initializer
Sensor::Sensor (float sampling_rate) {
  //set up interval informations
  sampling_time = 1000/sampling_rate;

  //set integrated data initial value
  speed.setValue (0, 0, 0);
  normal_vector.setValue (0, 0, 1);
}

Sensor::~Sensor() {
  //turn off mesurment units
  IMU.setSleepEnabled(true);
}

bool Sensor::initialize () {
  //initialize mesurement units
  I2Cdev::initialize();

  if(!IMU.testConnection()) {
    std::cout << "IMU connection test failed! Aborting initialization...\n";
    return 0;
  }
  //  IMU_SelfTest();
  
  IMU.setClockSource(MPU6050_CLOCK_PLL_XGYRO);
  IMU.setRate(0);
  IMU.setDLPFMode(MPU6050_DLPF_BW_188);
  IMU.setDHPFMode(MPU6050_DHPF_RESET);
  IMU.setFullScaleGyroRange(MPU6050_GYRO_FS_500);
  IMU.setFullScaleAccelRange(MPU6050_ACCEL_FS_4);
  LSB_accel = 8192;
  LSB_gyro  = 65.5; 
  
  IMU.setSleepEnabled(false);

  if(!TPU.testConnection()) {
    std::cout << "TPU connection test failed! Ignoring PTU.\n";
  } else {
    TPU.initialize();
  }
  
  //set initial value to buffer
  int16_t ax, ay, az, gx, gy, gz;
  IMU.getMotion6 (&ax, &ay, &az, &gx, &gy, &gz);
  accel_0.setValue(ax/LSB_accel, ay/LSB_accel, az/LSB_accel);
  gyro_0.setValue(gx/LSB_gyro, gy/LSB_gyro, gz/LSB_gyro);
  measure_time_0 = bcm2835_st_read() / 1000.0;
  
  return 1;
}

void Sensor::calibrate() {

}

void Sensor::getMotionData (Vector3D &accel_t, Vector3D &speed_t, Vector3D &angular_speed_t, Vector3D &normal_vector_t) {
  //obtain data from motion sensor
  int16_t ax, ay, az, gx, gy, gz;
  IMU.getMotion6 (&ax, &ay, &az, &gx, &gy, &gz);
  measure_time = bcm2835_st_read() / 1000.0;
  accel.setValue(ax/LSB_accel, ay/LSB_accel, az/LSB_accel);
  gyro.setValue(gx/LSB_gyro, gy/LSB_gyro, gz/LSB_gyro);
  float dt = (measure_time - measure_time_0) / 1000.0;
    
  //numerical integration of acceleration
  Vector3D dv = accel + accel_0;
  dv.scale(dt/2.0);
  speed = speed + dv;
    
  //numerical integration of angular velocity
  float dtheta = gyro.norm() * dt * M_PI / 90;
  normal_vector.rotate (dtheta, gyro);
  
  //initialize next measurement's initial value
  accel_0 = accel;
  gyro_0 = gyro;
  measure_time_0 = measure_time;
  
  //set value to output references
  speed_t = speed;
  normal_vector_t = normal_vector;
  accel_t = accel;
  angular_speed_t = gyro;

  bcm2835_delay(sampling_time);
}


void Sensor::IMU_SelfTest () {
  std::cout << "preforming self-tests:\n";
  IMU.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
  IMU.setFullScaleAccelRange(MPU6050_ACCEL_FS_8);
  IMU.setSleepEnabled(false);

  //obtain Factory Trim Values
  int FT = IMU.getGyroXSelfTestFactoryTrim();
  float gx_FT = FT==0 ? 0 : 25*131*pow(1.046, FT-1);
  FT = IMU.getGyroYSelfTestFactoryTrim();
  float gy_FT = FT==0 ? 0 : -25*131*pow(1.046, FT-1);
  FT = IMU.getGyroZSelfTestFactoryTrim();
  float gz_FT = FT==0 ? 0 : 25*131*pow(1.046, FT-1);
  FT = IMU.getAccelXSelfTestFactoryTrim();
  float ax_FT = FT==0 ? 0 : 4096*0.34*pow(0.92/0.34, (FT-1)/30.0);
  FT = IMU.getAccelYSelfTestFactoryTrim();
  float ay_FT = FT==0 ? 0 : 4096*0.34*pow(0.92/0.34, (FT-1)/30.0);
  FT = IMU.getAccelZSelfTestFactoryTrim();
  float az_FT = FT==0 ? 0 : 4096*0.34*pow(0.92/0.34, (FT-1)/30.0);

  //obtain Self Test Values;
  Vector3D gyro_unenabled, gyro_enabled, accel_unenabled, accel_enabled;
  int16_t x, y, z;
  std::cout << ".";
  
  for (int i = 0; i < 20; i++) {
    IMU.getRotation(&x, &y, &z);
    gyro_unenabled = gyro_unenabled + Vector3D(x, y, z);
    IMU.getAcceleration(&x, &y, &z);
    accel_unenabled = accel_unenabled + Vector3D(x, y, z);
    bcm2835_delay(10);
  }
  gyro_unenabled.scale(0.05);
  accel_unenabled.scale(0.05);
  IMU.setGyroXSelfTest(1);
  IMU.setGyroYSelfTest(1);
  IMU.setGyroZSelfTest(1);
  bcm2835_delay(500);
  std::cout << "."; 
  for (int i = 0; i < 20; i++) {
    IMU.getRotation(&x, &y, &z);
    gyro_enabled = gyro_enabled + Vector3D(x, y, z);
    bcm2835_delay(10);
  }
  gyro_enabled.scale(0.05);
  IMU.setGyroXSelfTest(0);
  IMU.setGyroYSelfTest(0);
  IMU.setGyroZSelfTest(0);
  
  IMU.setAccelXSelfTest(1);
  IMU.setAccelYSelfTest(1);
  IMU.setAccelZSelfTest(1);
  bcm2835_delay(500);
  std::cout << ".";
  for (int i = 0; i < 20; i++) {
    IMU.getAcceleration(&x, &y, &z);
    accel_enabled = accel_enabled + Vector3D(x, y, z);
    bcm2835_delay(50);
  }
  accel_enabled.scale(0.05);
  IMU.setAccelXSelfTest(0);
  IMU.setAccelYSelfTest(0);
  IMU.setAccelZSelfTest(0);
  IMU.setSleepEnabled(true);
  
  //calculate error from factory trim value
  Vector3D gyro_STR = gyro_enabled - gyro_unenabled;
  Vector3D accel_STR = accel_enabled - accel_unenabled;
  float err_gx = 100 * (gyro_STR.x - gx_FT) / gx_FT;
  float err_gy = 100 * (gyro_STR.y - gy_FT) / gz_FT;
  float err_gz = 100 * (gyro_STR.z - gz_FT) / gz_FT;
  float err_ax = 100 * (accel_STR.x - ax_FT) / ax_FT;
  float err_ay = 100 * (accel_STR.y - ay_FT) / ay_FT;
  float err_az = 100 * (accel_STR.z - az_FT) / az_FT;

  //display results
  std::cout << "\nSelf-test done. Displaying results:\n";
  std::cout << "===============================================================================\n";
  std::cout << "gyroscope error        \tgx: " << err_gx << " %     \tgy: " << err_gy << " %     \tgz: " << err_gz << "%\n";
  std::cout << "accelerometer error    \tax: " << err_ax << " %     \tay: " << err_ay << " %     \taz: " << err_az << "%\n";
  std::cout << "(Errors accepeable are 14% for both on specification sheet)\n";
  std::cout << "===============================================================================\n\n";
}
