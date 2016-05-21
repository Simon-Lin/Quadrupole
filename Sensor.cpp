#include "Sensor.h"
#include "Eigen/LU"
#include "Eigen/Geometry"
#include <iostream>
#include <math.h>

Sensor::Sensor(float sampling_rate) {
//plain initializer (only used in testing) Sensor::Sensor (float sampling_rate) {
  DATA = NULL;
  sampling_time = 1000/sampling_rate;
  ref_voltage = 5.0;
}

Sensor::Sensor (Data *DATA_ref, float sampling_rate) {
//initializer with DATA Sensor::Sensor (SensorData *DATA_ref, float sampling_rate) {
  DATA = DATA_ref;
  sampling_time = 1000/sampling_rate;
  ref_voltage = 5.0;
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
  IMU.setSleepEnabled(false);
  IMU.setClockSource(MPU6050_CLOCK_PLL_XGYRO);
  IMU.setRate(0);
  IMU.setDLPFMode(MPU6050_DLPF_BW_20);
  IMU.setDHPFMode(MPU6050_DHPF_RESET);
  IMU.setFullScaleGyroRange(MPU6050_GYRO_FS_500);
  IMU.setFullScaleAccelRange(MPU6050_ACCEL_FS_4);
  LSB_accel = 8192;
  LSB_gyro = 65.5;

  if(!TPU.testConnection()) {
    std::cout << "TPU connection test failed! Ignoring PTU.\n";
    TPU_connected = false;
  } else {
    TPU.initialize();
    TPU_connected = true;
  }

  if(!ADC.testConnection()) {
    std::cout << "ADC connection test failed! Ignoring ADC.\n"
              << "Please watch out for battery capacity.\n";
    ADC_connected = false;
  } else {
    ADC.initialize();
    ADC_connected = true;
  }
  bcm2835_delay (500);

  //initailize Kalamn filter
  int16_t ax, ay, az, gx, gy, gz;
  IMU.getMotion6 (&ax, &ay, &az, &gx, &gy, &gz);
  state << ax/LSB_accel, ay/LSB_accel, az/LSB_accel, gx/LSB_gyro, gy/LSB_gyro, gz/LSB_gyro;
  cor <<
    0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0;

  //initialize sensor data
  DATA->acceleration << 0, 0, 0;
  DATA->temperature = 0;
  DATA->altitute  = 0;
  DATA->batt_voltage = 0;
  return 1;
}


void* Sensor::start(void *context) {
  return ((Sensor*)context) -> _start();
}

void* Sensor::_start() {
  int cycle = 0;
  while (!(DATA->terminate)) {
    getMotionData (DATA->angular_speed, DATA->g_direction);

    //these data are updated per ten sensor cycle
    if (cycle >= 10) {
      if (TPU_connected) {
	DATA->pressure = this->getPressure();
	DATA->temperature = getTemperature();
      } else {
	DATA->pressure = NAN;
	DATA->temperature = NAN;
      }
      if (ADC_connected) {
	DATA->batt_voltage = getBattVoltage();
      } else {
	DATA->batt_voltage = NAN;
      }
      cycle = 0;
    }
    
    bcm2835_delay(sampling_time);
    cycle++;
  }

  pthread_exit(NULL);
}


void Sensor::getMotionData (Eigen::Vector3f &angular_speed, Eigen::Vector3f &g_direction) {
  //obtain data from motion sensor
  float ax, ay, az, gx, gy, gz;
  time = bcm2835_st_read() / 1000.0;
  float dt = (time - time_0) / 1000.0;
  
  //==========Kalman Filter============
  Eigen::Vector3f g_dir(state[0], state[1], state[2]);
  Eigen::Vector3f gyro(state[3], state[4], state[5]);
  float dtheta = -gyro.norm() * dt * M_PI / 180.0;
  float c = cos(dtheta);
  float s = sin(dtheta);
  gyro.normalize();
  ax = g_dir[0]; ay = g_dir[1]; az = g_dir[2];
  gx = gyro[0]; gy = gyro[1]; gz = gyro[2];
  float adotg = gx*ax + gy*ay + gz*az;
  //prediction matrix
  Eigen::Matrix<float, 6, 6> F;
  F <<
    c+gx*gx*(1-c), gx*gy*(1-c)-gz*s, gx*gz*(1-c)+gy*s, (1-c)*(adotg+gx*ax), (1-c)*ay*gx+az*s, (1-c)*az*gx-ay*s,
    gx*gy*(1-c)+gz*s, c+gy*gy*(1-c), gy*gz*(1-c)-gx*s, (1-c)*ax*gy-az*s, (1-c)*(adotg+gy*ay), (1-c)*az*gy+ax*s,
    gz*gx*(1-c)-gy*s, gz*gy*(1-c)+gx*s, c+gz*gz*(1-c), (1-c)*ax*gz+ay*s, (1-c)*ay*gz-ax*s, (1-c)*(adotg+az*gz),
    0, 0, 0, 1, 0, 0,
    0, 0, 0, 0, 1, 0,
    0, 0, 0, 0, 0, 1;
  //prediction
  Eigen::AngleAxisf rotate(dtheta, gyro);
  g_dir = rotate * g_dir;
  state_predict << g_dir[0], g_dir[1], g_dir[2], state[3], state[4], state[5];
  float error_dynamic = 5e-4 * gyro_diff;
  noise_predict <<
    3e-2+error_dynamic, 0, 0, 0, 0, 0,
    0, 3e-2+error_dynamic, 0, 0, 0, 0,
    0, 0, 3e-2+error_dynamic, 0, 0, 0,
    0, 0, 0, 3+gyro_diff, 0, 0,
    0, 0, 0, 0, 3+gyro_diff, 0,
    0, 0, 0, 0, 0, 3+gyro_diff;
  cor_predict = F * cor * F.transpose() + noise_predict;
  
  //observation
  int16_t ax_r, ay_r, az_r, gx_r, gy_r, gz_r;
  pthread_spin_lock (&(DATA->I2C_ACCESS));
  IMU.getMotion6 (&ax_r, &ay_r, &az_r, &gx_r, &gy_r, &gz_r);
  pthread_spin_unlock (&(DATA->I2C_ACCESS));
  g_dir << ax_r/LSB_accel, ay_r/LSB_accel, az_r/LSB_accel;
  gyro  << gx_r/LSB_gyro, gy_r/LSB_gyro, gz_r/LSB_gyro;
  g_dir.normalize();
  state_observ << g_dir[0], g_dir[1], g_dir[2], gyro[0], gyro[1], gyro[2];
  Eigen::Vector3f gyro_0 (state[3], state[4], state[5]);
  error_dynamic = 5e-4 * ((gyro_0 - gyro).squaredNorm());
  noise_observ <<
    3e-3+error_dynamic, 0, 0, 0, 0, 0,
    0, 3e-3+error_dynamic, 0, 0, 0, 0,
    0, 0, 3e-3+error_dynamic, 0, 0, 0,
    0, 0, 0, 3, 0, 0,
    0, 0, 0, 0, 3, 0,
    0, 0, 0, 0, 0, 3;
  
  //update
  Eigen::Matrix<float, 6, 6> K;
  K = (cor_predict + noise_observ).inverse();
  state = cor_predict * K * state_observ + noise_observ * K * state_predict;
  cor = cor_predict * K * noise_observ;
  g_dir << state[0], state[1], state[2];
  g_dir.normalize();
  state[0] = g_dir[0]; state[1] = g_dir[1]; state[2] = g_dir[2];
  //=========Kalman Filter=============

 //std::cout << "predict\n" << state_predict << "\nobserve\n" << state_observ << "\ncombined\n" << state << "\n";
 //std::cout << "cor_predict\n" << cor_predict << "\nnoise_observed\n" << noise_observ << "\nK\n" << K << "\ncor_combined\n" << cor << "\n";
 //std::cout << "====================\n";
  
  //set value to output references
  g_direction = g_dir;
  angular_speed = gyro;
}

float Sensor::getPressure() {
  pthread_spin_lock (&(DATA->I2C_ACCESS));
  TPU.setControl(BMP085_MODE_PRESSURE_3) ; //taking reading in highest accuracy measurement mode
  bcm2835_delay( TPU.getMeasureDelayMicroseconds() / 1000 );
  float tmp = TPU.getPressure();
  pthread_spin_unlock (&(DATA->I2C_ACCESS));
  return tmp;
}

float Sensor::getTemperature() {
  pthread_spin_lock (&(DATA->I2C_ACCESS));
  TPU.setControl(BMP085_MODE_TEMPERATURE);
  bcm2835_delay(5); // wait 5 ms for conversion
  float tmp = TPU.getTemperatureC();
  pthread_spin_unlock (&(DATA->I2C_ACCESS));
  return tmp;
}

float Sensor::getBattVoltage() {
  ADC.setInputMode (0, 0);
  pthread_spin_lock (&(DATA->I2C_ACCESS));
  float tmp = ADC.ADConversion() * ref_voltage * 3;
  pthread_spin_unlock (&(DATA->I2C_ACCESS));
  return tmp;
}

void Sensor::accelCalibrate() {
  std::cout << "Performing accelerometer calibration. Do not move the device...\n";
  
  //initial guess of offset
  Eigen::Vector3f accel_off(0,0,0);
  IMU.setXAccelOffset((int16_t)accel_off[0]);
  IMU.setYAccelOffset((int16_t)accel_off[1]);
  IMU.setZAccelOffset((int16_t)accel_off[2]);
  bcm2835_delay(100);

  Eigen::Vector3f accel_cal_0, accel_cal_1;
  Eigen::Vector3f accel_fix (512, 512, 512);
  
  //obtain initial offset value
  for (int j = 0; j < 100; j++) {
    int16_t ax, ay, az;
    IMU.getAcceleration(&ax, &ay, &az);
    accel_cal_0 = accel_cal_0 + Eigen::Vector3f(ax, ay, az);
    bcm2835_delay(5);
  }
  accel_cal_0 *= 0.01;
  accel_cal_0[2] -= LSB_accel;

  for (int i = 0; i < 30; i++) {
    
    //obtain sensor value
    for (int j = 0; j < 100; j++) {
      int16_t ax, ay, az;
      IMU.getAcceleration(&ax, &ay, &az);
      accel_cal_1 = accel_cal_1 + Eigen::Vector3f(ax, ay, az);
      bcm2835_delay(5);
    }
    accel_cal_1 *= 0.01;
    
    //fix the offset value
    fixOffset (accel_cal_1[0], accel_cal_0[0], accel_off[0], accel_fix[0]);
    fixOffset (accel_cal_1[1], accel_cal_0[1], accel_off[1], accel_fix[1]);
    accel_cal_1[2] -= LSB_accel;
    fixOffset (accel_cal_1[2], accel_cal_0[2], accel_off[2], accel_fix[2]);

    //Write new offsets
    IMU.setXAccelOffset((int16_t)round(accel_off[0]));
    IMU.setYAccelOffset((int16_t)round(accel_off[1]));
    IMU.setZAccelOffset((int16_t)round(accel_off[2]));
    bcm2835_delay(100);
    std::cout << ">" << std::flush;
  }

  std::cout << " done\n";
}


void Sensor::gyroCalibrate() {
  std::cout << "Performing gyroscope calibration. Do not move the device...\n";
  
  //initial guess of offset
  Eigen::Vector3f gyro_off(0,0,0);
  IMU.setXGyroOffset((int16_t)gyro_off[0]);
  IMU.setYGyroOffset((int16_t)gyro_off[1]);
  IMU.setZGyroOffset((int16_t)gyro_off[2]);
  bcm2835_delay(100);

  Eigen::Vector3f gyro_cal_0, gyro_cal_1;
  Eigen::Vector3f gyro_fix(256, 256, 256);
  
  //obtain initial offset value
  for (int j = 0; j < 100; j++) {
    int16_t gx, gy, gz;
    IMU.getRotation(&gx, &gy, &gz);
    gyro_cal_0 = gyro_cal_0 + Eigen::Vector3f(gx, gy, gz);
    bcm2835_delay(5);
  }
  gyro_cal_0 *= 0.01;

  for (int i = 0; i < 30; i++) {
    
    //obtain sensor value
    for (int j = 0; j < 100; j++) {
      int16_t gx, gy, gz;
      IMU.getRotation(&gx, &gy, &gz);
      gyro_cal_1 = gyro_cal_1 + Eigen::Vector3f(gx, gy, gz);
      bcm2835_delay(5);
    }
    gyro_cal_1 *= 0.01;
    
    //fix the offset value
    fixOffset (gyro_cal_1[0], gyro_cal_0[0], gyro_off[0], gyro_fix[0]);
    fixOffset (gyro_cal_1[1], gyro_cal_0[1], gyro_off[1], gyro_fix[1]);
    fixOffset (gyro_cal_1[2], gyro_cal_0[2], gyro_off[2], gyro_fix[2]);

    //Write new offsets
    IMU.setXGyroOffset((int16_t)round(gyro_off[0]));
    IMU.setYGyroOffset((int16_t)round(gyro_off[1]));
    IMU.setZGyroOffset((int16_t)round(gyro_off[2]));
    bcm2835_delay(100);
    std::cout << ">" << std::flush;
  }
  
  std::cout << " done\n";
}


void Sensor::IMU_GetOffsets () {
  // std::cout << "Obtaining offsets and gains data from IMU:\n";
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

  std::cout << "gyroscope offsets (deg/s): gx: " << gx_off << " gy: " << gy_off << " gz: " << gz_off << std::endl;
  std::cout << "accelerometer offsets (g): ax: " << ax_off << " ay: " << ay_off << " az: " << az_off << std::endl;
  std::cout << "fine gains: x:" << x_gain << " y: " << y_gain << " z: " << z_gain << std::endl;
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
  Eigen::Vector3f gyro_unenabled, gyro_enabled, accel_unenabled, accel_enabled;
  int16_t x, y, z;
  std::cout << "." << std::flush;
  
  for (int i = 0; i < 20; i++) {
    IMU.getRotation(&x, &y, &z);
    gyro_unenabled = gyro_unenabled + Eigen::Vector3f(x, y, z);
    IMU.getAcceleration(&x, &y, &z);
    accel_unenabled = accel_unenabled + Eigen::Vector3f(x, y, z);
    bcm2835_delay(10);
  }
  gyro_unenabled *= 0.05;
  accel_unenabled *= 0.05;
  IMU.setGyroXSelfTest(1);
  IMU.setGyroYSelfTest(1);
  IMU.setGyroZSelfTest(1);
  bcm2835_delay(500);
  std::cout << "." << std::flush;
  for (int i = 0; i < 20; i++) {
    IMU.getRotation(&x, &y, &z);
    gyro_enabled = gyro_enabled + Eigen::Vector3f(x, y, z);
    bcm2835_delay(10);
  }
  gyro_enabled *= 0.05;
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
    accel_enabled = accel_enabled + Eigen::Vector3f(x, y, z);
    bcm2835_delay(50);
  }
  accel_enabled *= 0.05;
  IMU.setAccelXSelfTest(0);
  IMU.setAccelYSelfTest(0);
  IMU.setAccelZSelfTest(0);
  IMU.setSleepEnabled(true);
  
  //calculate error from factory trim value
  Eigen::Vector3f gyro_STR = gyro_enabled - gyro_unenabled;
  Eigen::Vector3f accel_STR = accel_enabled - accel_unenabled;
  float err_gx = 100 * (gyro_STR[0] - gx_FT) / gx_FT;
  float err_gy = 100 * (gyro_STR[1] - gy_FT) / gz_FT;
  float err_gz = 100 * (gyro_STR[2] - gz_FT) / gz_FT;
  float err_ax = 100 * (accel_STR[0] - ax_FT) / ax_FT;
  float err_ay = 100 * (accel_STR[1] - ay_FT) / ay_FT;
  float err_az = 100 * (accel_STR[2] - az_FT) / az_FT;

  //display results
  std::cout << "\nSelf-test done. Displaying results:\n";
  std::cout << "===============================================================================\n";
  std::cout << "gyroscope error \tgx: " << err_gx << " % \tgy: " << err_gy << " % \tgz: " << err_gz << "%\n";
  std::cout << "accelerometer error \tax: " << err_ax << " % \tay: " << err_ay << " % \taz: " << err_az << "%\n";
  std::cout << "(Errors accepeable are 14% for both on specification sheet)\n";
  std::cout << "===============================================================================\n\n";
}

void Sensor::fixOffset (float &cal_1, float &cal_0, float &off, float &fix) {
  //offset fixing utility used by IMU_calibration method
  if (std::signbit(cal_1) != std::signbit(cal_0)) {
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
	  off -= fix;
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
