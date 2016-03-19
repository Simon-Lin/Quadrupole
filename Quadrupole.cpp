#include "Sensor.h"
#include "Interface.h"
#include "Controller.h"
#include "Vector3D.h"
#include <bcm2835.h>
#include <iostream>

bool controlcycle (bool &terminate) {
  bcm2835_delay (100);
  
  INT_DATA.g_direction = SEN_DATA.g_direction;
  INT_DATA.acceleration = SEN_DATA.acceleration;
  INT_DATA.speed = SEN_DATA.speed;
  INT_DATA.angular_speed = SEN_DATA.angular_speed;
  INT_DATA.pressure = SEN_DATA.pressure;
  INT_DATA.temperature = SEN_DATA.temperature;
  interface.update();
    
  if (INT_DATA.atthold) {
    controller.control_HoldAtt (SEN_DATA.speed.z, SEN_DATA.angular_speed.z, INT_DATA.yaw_set, SEN_DATA.g_direction, INT_DATA.g_direction_set);
  } else {
    controller.control (INT_DATA.throttle, SEN_DATA.angular_speed.z, INT_DATA.yaw_set, SEN_DATA.g_direction, INT_DATA.g_direction_set);  
  }

  if (INT_DATA.power_off) {
    terminate = true;
    return false;
  } else {
    return true;
  }
}

int main (int argc, char *argv[]) {
  system("clear");
  std::cout << "Quadrupole controller side ver 0.1\n";
  std::cout << "Initializing sensors...\n"
  SensorData SEN_DATA;
  Sensor sensor(&SEN_DATA, 50);
  sensor.initialize();

  std::cout << "Intiailzing network server...\n";
  InterfaceData INT_DATA;
  Interface interface(&INT_DATA);
  interface.initialize();

  std::cout << "Initializing servo controller...\n";
  Controller controller;
  controller.initialize();

  std::cout << "Startup process complete! Ready for a flight.\n";
  bool terminate = false;
  #pragma omp parallel sections
  {
    #pragma omp section
    {
      sensor.start (terminate);
    }
    #pragma omp section
    {
      while (controlcycle(terminate));
    }
  }
  
  return 0;
}