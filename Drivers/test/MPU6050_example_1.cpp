/*
I2Cdev library collection - MPU6050 RPi example
Based on the example in Arduino/MPU6050/

==============================================
I2Cdev device library code is placed under the MIT license

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================

To compile on a Raspberry Pi (1 or 2)
  1. install the bcm2835 library, see http://www.airspayce.com/mikem/bcm2835/index.html
  2. enable i2c on your RPi , see https://learn.adafruit.com/adafruits-raspberry-pi-lesson-4-gpio-setup/configuring-i2c
  3. connect your i2c devices
  4. then from bash
      $ PATH_I2CDEVLIB=~/i2cdevlib/
      $ gcc -o MPU6050_example_1 ${PATH_I2CDEVLIB}RaspberryPi_bcm2835/MPU6050/examples/MPU6050_example_1.cpp \
         -I ${PATH_I2CDEVLIB}RaspberryPi_bcm2835/I2Cdev ${PATH_I2CDEVLIB}RaspberryPi_bcm2835/I2Cdev/I2Cdev.cpp \
         -I ${PATH_I2CDEVLIB}/Arduino/MPU6050/ ${PATH_I2CDEVLIB}/Arduino/MPU6050/MPU6050.cpp -l bcm2835 -l m
      $ sudo ./MPU6050_example_1

*/

#include <stdio.h>
#include <bcm2835.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include <math.h>
#include <iostream>
#include <fstream>
#include <sys/mman.h>

using namespace std;

int main(int argc, char **argv) {
  mlockall(MCL_CURRENT | MCL_FUTURE);
  
  printf("MPU6050 3-axis acceleromter example program\n");
  I2Cdev::initialize();
  
  MPU6050 IMU;
  int16_t rax, ray, raz, rgx, rgy, rgz;
  float ax, ay, az, gx, gy, gz;
  if ( IMU.testConnection() ) 
    printf("MPU6050 connection test successful\n") ;
  else {
    fprintf( stderr, "MPU6050 connection test failed! something maybe wrong, continuing anyway though ...\n");
    //return 1;
  }
  //accelgyro.initialize();
  IMU.setClockSource(MPU6050_CLOCK_PLL_XGYRO);
  IMU.setRate(0);
  IMU.setDLPFMode(MPU6050_DLPF_BW_98);
  IMU.setDHPFMode(MPU6050_DHPF_RESET);
  IMU.setFullScaleGyroRange(MPU6050_GYRO_FS_500);
  IMU.setFullScaleAccelRange(MPU6050_ACCEL_FS_4);
  printf ("%d  %d\n", IMU.getFullScaleGyroRange(), IMU.getFullScaleAccelRange());
  float LSB_accel = 8192;
  float LSB_gyro  = 65.5;
  IMU.setSleepEnabled(false);
  bcm2835_delay(100);
  
  // use the code below to change accel/gyro offset values
  /*
  printf("Updating internal sensor offsets...\n");
  // -76	-2359	1688	0	0	0
  printf("%i \t %i \t %i \t %i \t %i \t %i\n", 
	 accelgyro.getXAccelOffset(),
	 accelgyro.getYAccelOffset(),
	 accelgyro.getZAccelOffset(),
	 accelgyro.getXGyroOffset(),
	 accelgyro.getYGyroOffset(),
	 accelgyro.getZGyroOffset());
  accelgyro.setXGyroOffset(220);
  accelgyro.setYGyroOffset(76);
  accelgyro.setZGyroOffset(-85);
  printf("%i \t %i \t %i \t %i \t %i \t %i\n", 
	 accelgyro.getXAccelOffset(),
	 accelgyro.getYAccelOffset(),
	 accelgyro.getZAccelOffset(),
	 accelgyro.getXGyroOffset(),
	 accelgyro.getYGyroOffset(),
	 accelgyro.getZGyroOffset());
  */

  ofstream fax, fay, faz, fgx, fgy, fgz;
  fax.open("ax.txt");
  fay.open("ay.txt");
  faz.open("az.txt");
  fgx.open("gx.txt");
  fgy.open("gy.txt");
  fgz.open("gz.txt");
  
  printf("\n");
  printf("    ax    \t    ay    \t    az    \t    gx    \t    gy    \t    gz:\n");
  double timer0 = (double) bcm2835_st_read() * 1E-6;
  while (true) {
    IMU.getMotion6(&rax, &ray, &raz, &rgx, &rgy, &rgz);
    ax = rax / LSB_accel;
    ay = ray / LSB_accel;
    az = raz / LSB_accel;
    gx = rgx / LSB_gyro;
    gy = rgy / LSB_gyro;
    gz = rgz / LSB_gyro;
    printf("  %f \t %f \t %f \t %f \t %f \t %f\r", ax, ay, az, gx, gy, gz);
    fflush(stdout);
    double timer = (double) bcm2835_st_read() * 1E-6;
    timer -= timer0;
    fax << timer << "\t" << ax << endl;
    fay << timer << "\t" << ay << endl;
    faz << timer << "\t" << az << endl;
    fgx << timer << "\t" << gx << endl;
    fgy << timer << "\t" << gy << endl;
    fgz << timer << "\t" << gz << endl;
    bcm2835_delay(100);
  }
  fax.close();
  fay.close();
  faz.close();
  fgx.close();
  fgy.close();
  fgz.close();
  return 1; 
}
