#include "Interface.h"
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <math.h>

Interface::Interface (InterfaceData *DATA_ref) {
  DATA = DATA_ref;
}

Interface::~Interface () {
  endwin();
  close(js_fd);
}

bool Interface::initialize() {
  //load joystick driver
  if ((js_fd = open("/dev/input/js0", O_RDONLY | O_NONBLOCK)) == -1) {
    printf ("Cannot open jotstick. Exiting...\n");
    return -1;
  }
  for (int i = 0; i < 32; i++) {
    button[i] = 0;
  }
  for (int i = 0; i < 8; i++) {
    axis[i] = 0;
  }

  //ncurses intialization
  initscr();
  cbreak();
  keypad(stdscr, TRUE);
  noecho();

  //draw basis scene
  getmaxyx (stdscr, row_max, col_max);
  mvprintw (0, col_max/2 - 16, "Quadrupole User Control Interface");
  mvprintw (row_max-1, col_max/2 - 13, "version 0.01 by Simon Lin");
  mvchgat (0, 0, -1, A_STANDOUT, 0, NULL);
  mvchgat (row_max-1, 0, -1, A_STANDOUT, 0, NULL);
  refresh ();
  return 1;
}

void Interface::update () {
  //calculate roll and pitch
  float roll = asin (DATA->g_direction.y);
  float pitch = asin (DATA->g_direction.x);

  //print data
  mvprintw (4, 2, "roll: % f   pitch: % f", roll, pitch);
  mvprintw (6, 2, "angular speed: (% f, % f, % f)", DATA->angular_speed.x, DATA->angular_speed.y, DATA->angular_speed.z);
  mvprintw (9, 2, "velocity: (% f, % f, % f)", DATA->speed.x, DATA->speed.y, DATA->speed.z);
  mvprintw (11, 2, "acceleration: (% f, % f, % f)", DATA->acceleration.x, DATA->acceleration.y, DATA->acceleration.z);
  mvprintw (13, 2, "height: % f", DATA->height);
  mvprintw (16, 2, "tempreature: % f   pressure: % f", DATA->temperature, DATA->pressure);
  refresh();

  //read data from joystick
  while (read (js_fd, &js, sizeof(js_event)) > 0) {
    switch (js.type & ~JS_EVENT_INIT) {
    case JS_EVENT_BUTTON:
      button [js.number] = js.value ? true : false;
      break;
    case JS_EVENT_AXIS:
      axis [js.number] = js.value;
      break;
    } 
  }

  //set control signals
  DATA->throttle = (float)axis[1] / 32767;
  DATA->yaw_set = (float)axis[0] / 32767;
  DATA->g_direction_set.x = (float)axis[2] / 32767 / 1.414;
  DATA->g_direction_set.y = (float)axis[3] / 32767 / 1.414;
  DATA->g_direction_set.z = -sqrt(1 - DATA->g_direction_set.x*DATA->g_direction_set.x - DATA->g_direction_set.y*DATA->g_direction_set.y);
  DATA->att_hold = button[0];
}

void Interface::joystickTest () {
  //get joystick information
  int naxis = 0, nbutton = 0;
  char name[80];
  ioctl(js_fd, JSIOCGAXES, &naxis);
  ioctl(js_fd, JSIOCGBUTTONS, &nbutton);
  ioctl(js_fd, JSIOCGNAME(80), &name);
  mvprintw (3, 2, "Joystick name: %s", name);
  mvprintw (4, 2, "with %d axis and %d buttons", naxis, nbutton);
  mvprintw (5, 2, "(type e to exit testing...)");
  refresh ();

  //print events
  while (getch() != 'e') {
    read(js_fd, &js, sizeof(js_event));
    switch (js.type & ~JS_EVENT_INIT) {
    case JS_EVENT_BUTTON:
      button [js.number] = js.value ? true : false;
      break;
    case JS_EVENT_AXIS:
      axis [js.number] = js.value;
      break;
    } 
    mvprintw (7, 2, "axis status   1:% d   2:% d   3:% d   4:% d", axis[0], axis[1], axis[2], axis[3]);
    mvprintw (8, 16, "5:% d   6:% d   7:% d   8:% d", axis[4], axis[5], axis[6], axis[7]);
    mvprintw (10, 2,"button status 1: %d  2: %d  3: %d  4: %d  5: %d  6: %d  7: %d  8: %d", button[0], button[1], button[2], button[3], button[4], button[5], button[6], button[7]);
    mvprintw (11, 16,"9: %d  10: %d  11: %d  12: %d  13: %d  14: %d  15: %d  16: %d", button[8], button[9], button[10], button[11], button[12], button[13], button[14], button[15]);
    refresh();
  }
}
