#include "Sensor.h"
#include "Interface.h"
#include "Controller.h"
#include <bcm2835.h>
#include <iostream>
#include <pthread.h>
#include <sys/mman.h>
#include <string.h>

bool controlcycle (Sensor &sensor, Interface &interface, Controller &controller, Data &DATA) {
  bcm2835_delay (100);

  interface.update();
    
  if (DATA.att_hold) {
    std::cout << "ATTHOLD   ";
    controller.control_HoldAtt (DATA.speed[2], DATA.angular_speed[2], DATA.yaw_set, DATA.g_direction, DATA.g_direction_set);
  } else {
    controller.control (DATA.throttle, DATA.angular_speed[2], DATA.yaw_set, DATA.g_direction, DATA.g_direction_set);  
  }

  if (DATA.power_off) {
    DATA.terminate = true;
    return false;
  } else {
    return true;
  }
}

int main (int argc, char *argv[]) {
  system("clear");
  std::cout << "Quadrupole controller side ver 0.1\n";
  bool calibrate = false, resume = false;
  for (int i = 1; i <= argc; i++) {
    if (!strcmp (argv[i], "-c")) {
      calibrate = true;
    } else if (!strcmp (argv[i], "-r")) {
      resume = true;
    } else {
      std::cout << "usage: Quadrupole [-c] [-r]\n";
      std::cout << "\t[-c] Calibrate IMU sensors when starting up.\n";
      std::cout << "\t[-r] Resume connection option. Ignore handshaking process when intializing the network server.\n"
    }
  }
  
  std::cout << "Initializing sensors...\n";
  Data DATA;  
  Sensor sensor(&DATA, 50);
  if (!sensor.initialize()) return 1;
  if (calibrate) {
    sensor.accelCalibrate();
    sensor.gyroCalibrate();
  }

  std::cout << "Initializing servo controller...\n";
  ControlParameters PARA;
  PARA.bal_lin = 0.005;
  PARA.bal_diff = 0.000;
  PARA.bal_int = 0.000;
  PARA.yaw_lin = 0.00001;
  PARA.yaw_diff = 0;
  PARA.att_con = PARA.att_lin = PARA.att_diff = PARA.att_int = 0.0001;
  Controller controller (&DATA, PARA);
  if (!controller.initialize()) {
    return 1;
  }

  std::cout << "Intiailzing network server...\n";
  Interface interface(&DATA);
  if (resume) {
    if (!interface.resume()) return 1;
  } else {
    if (!interface.initialize()) return 1;
  }

  
  //setting scheduling policy of main thread
  sched_param sp_main;
  sp_main.sched_priority = 98;
  sched_setscheduler(0, SCHED_FIFO, &sp_main);

  //force program to RAM
  mlockall(MCL_CURRENT | MCL_FUTURE);

  //spinlock initialization
  if (pthread_spin_init(&(DATA.I2C_ACCESS), 0)) {
    std::cout << "pthread spinlock initialization failed.\n";
   return 1;
  }
  
  //start collecting sensor data
  DATA.terminate = false;
  pthread_t thread_sensor;
  pthread_attr_t attr;
  pthread_attr_init(&attr);
  pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
  sched_param sp_sensor;
  sp_sensor.sched_priority = 99;
  pthread_attr_setschedparam(&attr, &sp_sensor);
  pthread_attr_setschedpolicy(&attr, SCHED_FIFO);
  if (pthread_create (&thread_sensor, NULL, &Sensor::start, &sensor)) {
    std::cout << "pthread create failed.\n" << std::flush;
    return 1;
  }

  bcm2835_delay(500);
  std::cout << "Startup process complete! Waiting for UNLOCK signal...\n";
  interface.startupLock();
  std::cout << "UNLOCK recieved!\n" << std::flush;
  
  //main loop
  while (controlcycle(sensor, interface, controller, DATA));

  //destructors
  pthread_join (thread_sensor, NULL);
  pthread_spin_destroy (&(DATA.I2C_ACCESS));
  
  std::cout << "bye!\n";
  return 0;
}
