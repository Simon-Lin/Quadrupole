#include "../Sensor.h"
#include <iostream>

int main() {
  Sensor sensor(10);
  sensor.initialize();
  
  bcm2835_delay(500);
  
  while (true) {
    bcm2835_delay(100);
    sensor.getMotionData_PIPE();
  }
  
  return 0;
}
