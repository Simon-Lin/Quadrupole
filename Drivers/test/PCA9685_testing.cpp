#include <bcm2835.h>
#include <iostream>
#include "../I2Cdev.h"
#include "../PCA9685.h"
#include <sys/select.h>

int kbhit();

int main () {
  I2Cdev::initialize();
  PCA9685 servo;
  servo.initialize();
  while (1) {
    for (float i = 0; i <= 1; i += 0.05) {
      servo.setPWM(0,i);
      servo.setPWM(1,1-i);
      bcm2835_delay(50);
      if (kbhit()) goto END;
    }
    for (float i = 1; i >= 0; i -= 0.05) {
      servo.setPWM(0,i);
      servo.setPWM(1,1-i);
      bcm2835_delay(50);
      if (kbhit()) goto END;
    }
  }
 END:
  servo.sleep();
  return 1;
}

int kbhit()
{
  struct timeval tv;
  fd_set read_fd;

  tv.tv_sec=0;
  tv.tv_usec=0;
  FD_ZERO(&read_fd);
  FD_SET(0,&read_fd);

  if(select(1, &read_fd, NULL, NULL, &tv) == -1)
    return 0;

  if(FD_ISSET(0,&read_fd))
    return 1;

  return 0;
}
