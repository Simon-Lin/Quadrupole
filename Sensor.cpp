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
  IMU.setClockSource(MPU6050_CLOCK_PLL_XGYRO);
  IMU.setRate(0);
  IMU.setDLPFMode(MPU6050_DLPF_BW_188);
  IMU.setDHPFMode(MPU6050_DHPF_RESET);
  IMU.setFullScaleGyroRange(MPU6050_GYRO_FS_500);
  IMU.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
  LSB_accel = 16384;
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

void Sensor::IMU_Calibrate() {
  std::cout << "you are going to perform an HARD calibration.\n"
            << "This is achieved by resetting the offsets and gains in IMU chip.\n";
  std::cout << "Proceed (y/n)? ";
  char input;
  do {
    std::cin >> input;
  } while (input != 'Y' && input != 'y' && input != 'n' && input != 'N');
  if (input == 'n' || input == 'N') return;
  std::cout << "Please place the IMU with z axis pointing directly down. Ready (y)? ";
  do {
    std::cin >> input;
  } while (input != 'Y' && input != 'y');
  std::cout << "Performing calibration. Do not move the device...\n";
  
  //initial guess of offset
  Vector3D accel_off(0,0,0), gyro_off(0,0,0);
  IMU.setXAccelOffset((int16_t)accel_off.x);
  IMU.setYAccelOffset((int16_t)accel_off.y);
  IMU.setZAccelOffset((int16_t)accel_off.z);
  IMU.setXGyroOffset((int16_t)gyro_off.x);
  IMU.setYGyroOffset((int16_t)gyro_off.y);
  IMU.setZGyroOffset((int16_t)gyro_off.z);
  bcm2835_delay(100);

  Vector3D accel_cal_0, gyro_cal_0, accel_cal_1, gyro_cal_1;
  Vector3D accel_fix(1024, 1024, 1024), gyro_fix(256, 256, 256);
  
  //obtain initial offset value
  for (int j = 0; j < 100; j++) {
    int16_t ax, ay, az, gx, gy, gz;
    IMU.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    accel_cal_0 = accel_cal_0 + Vector3D(ax, ay, az);
    gyro_cal_0 = gyro_cal_0 + Vector3D(gx, gy, gz);
    bcm2835_delay(10);
  }
  accel_cal_0.scale(0.01);
  gyro_cal_0.scale(0.01);

  for (int i = 0; i < 30; i++) {
    
    //obtain sensor value
    for (int j = 0; j < 500; j++) {
      int16_t ax, ay, az, gx, gy, gz;
      IMU.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
      accel_cal_1 = accel_cal_1 + Vector3D(ax, ay, az);
      gyro_cal_1 = gyro_cal_1 + Vector3D(gx, gy, gz);
      bcm2835_delay(10);
    }
    accel_cal_1.scale(0.002);
    gyro_cal_1.scale(0.002);
    
    //fix the offset value
    fixOffset (gyro_cal_1.x, gyro_cal_0.x, gyro_off.x, gyro_fix.x);
    fixOffset (gyro_cal_1.y, gyro_cal_0.y, gyro_off.y, gyro_fix.y);
    fixOffset (gyro_cal_1.z, gyro_cal_0.z, gyro_off.z, gyro_fix.z);
    fixOffset (accel_cal_1.x, accel_cal_0.x, accel_off.x, accel_fix.x);
    fixOffset (accel_cal_1.y, accel_cal_0.y, accel_off.x, accel_fix.y);
    fixOffset (LSB_accel+accel_cal_1.z, LSB_accel+accel_cal_0.z, accel_off.z, accel_fix.z);

    //Write new offsets
    IMU.setXAccelOffset((int16_t)round(accel_off.x));
    IMU.setYAccelOffset((int16_t)round(accel_off.y));
    IMU.setZAccelOffset((int16_t)round(accel_off.z));
    IMU.setXGyroOffset((int16_t)round(gyro_off.x));
    IMU.setYGyroOffset((int16_t)round(gyro_off.y));
    IMU.setZGyroOffset((int16_t)round(gyro_off.z));
    bcm2835_delay(100);
    std::cout << ">" << std::flush;
  }

  IMU_GetOffsets();
  std::cout << "\ncalibration complete.\n";
}

void Sensor::IMU_GetOffsets () {
  //  std::cout << "Obtaining offsets and gains data from IMU:\n";
  float ax_off, ay_off, az_off, gx_off, gy_off, gz_off, x_gain, y_gain, z_gain;

  ax_off = IMU.getXAccelOffset();
  ay_off = IMU.getYAccelOffset();
  az_off = IMU.getZAccelOffset();
  gx_off = IMU.getXGyroOffset(); 
  gy_off = IMU.getYGyroOffset();
  gz_off = IMU.getZGyroOffset();
  x_gain = IMU.getXFineGain();
  y_gain = IMU.getYFineGain();
  z_gain = IMU.getZFineGain();

  std::cout << "gyroscope offsets (deg/s):   gx: " << gx_off << "   gy: " << gy_off << "   gz: " << gz_off << std::endl;
  std::cout << "accelerometer offsets (g):   ax: " << ax_off << "   ay: " << ay_off << "   az: " << az_off << std::endl;
  std::cout << "fine gains:   x:" << x_gain << "   y: " << y_gain << "   z: " << z_gain << std::endl;
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
  std::cout << "." << std::flush;
  
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
  std::cout << "." << std::flush; 
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
  std::cout << "." << std::flush;
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

void Sensor::fixOffset (float &cal_1, float &cal_0, float off&, float &fix) {
  //offset fixing utility used by IMU_calibration method
  if (signbit(cal_1) != signbit(cal_0)) {
    if (abs(cal_1) < abs(cal_0)) {
      if (cal_1 > 0) {
        off -= fix;
      } else {
	    off += fix;
      } 
        cal_0 = cal_1;
      } else {
        if (cal_0 > 0) {
          off += fix;
	    } else {
	      off -= gfix;
        }
      }
      fix /= 2;
    } else {
      if (cal_1 > 0) {
        off -= fix;
      } else {
        off += fix;
      }
      cal_0 = cal_1;
    }
}
