#define PORTNO 13704
#define DELAY_MILLISEC 100

#include <ncurses.h>
#include <stdio.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <netinet/in.h>
#include <string.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <math.h>
#include "Gamepad.h"

struct Vector3D {
  float x, y, z;
  
  Vector3D () {
    x = 0;
    y = 0;
    z = 0;
  }
  
  Vector3D (float a, float b, float c) {
    x = a;
    y = b;
    z = c;
  }
};

void kbstat ();
void serialize (float in, char *out);
void serialize (Vector3D in, char *out);
float decode_f (char *in);
Vector3D decode_v (char *in);


int main(int argc, char *argv[]) {

  if (argc == 1) {
  	printf("please provide Quadrupole server IP\n");
  	return 1;
  }

  //socket initialization
  int sockfd;
  sockaddr_in serv_addr;
  char buffer[100];
  printf("initalizing socket...\n");
  fflush(stdout);

  if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
      perror("cannot open socket");
      return 1;
  }
  
  bzero((char*) &serv_addr, sizeof(serv_addr));
  serv_addr.sin_family = AF_INET;
  serv_addr.sin_port = htons(PORTNO);
  if (inet_aton((argv[1]), &serv_addr.sin_addr) == 0) {
    printf("not valid IP format. exiting...\n");
    return 1;
  }

  int servlen = sizeof(serv_addr);
  if (connect(sockfd, (struct sockaddr*)&serv_addr, servlen) < 0) {
    perror("cannot establish connection");
    return 1;
  }

  //send and check server ACK
  char ackbuf[] =  "QUAD";
  if(send(sockfd, ackbuf, 5, 0) < 0) {
    perror("cannot send ack signal to server");
    return 1;
  }
  if(recv(sockfd, ackbuf, 5, 0) < 0) {
    perror("cannot recieve ack signal from server");
    return 1;
  }
  if (strcmp(ackbuf, "QUAD")) {
    printf("WRONG connection from server. exiting...\n");
    return 1;
  }

  //initialization complete, setting socket to non-blocking mode
  if (fcntl(sockfd, F_SETFL, fcntl(sockfd, F_GETFL, 0) | O_NONBLOCK)) {
    perror("setting socket to non-blocking failed");
    return 1;
  }
  printf ("connection established, socket initialization complete.\n");
  usleep(500000);

  //ncurses initialization
  initscr();
  keypad(stdscr, TRUE);
  noecho();

  //draw basis scene
  int row_max, col_max;
  getmaxyx (stdscr, row_max, col_max);
  mvprintw (0, col_max/2 - 16, "Quadrupole User Control Interface");
  mvprintw (row_max-2, col_max/2 - 13, "version 0.01 by Simon Lin");
  mvchgat (0, 0, -1, A_STANDOUT, 0, NULL);
  mvchgat (row_max-2, 0, -1, A_STANDOUT, 0, NULL);
  refresh ();

  //check for joystick
  Gamepad_init();
  Gamepad_detectDevices();
  Gamepad_device *js = NULL;
  if (Gamepad_numDevices() > 0) {
    js = Gamepad_deviceAtIndex(0);
    mvprintw(row_max-1, 0, "joystick: %s detected", js->description);
    refresh();
  } else {
    mvprintw(row_max-1, 0, "no joysticks detected. exiting...");
    refresh();
    getch();
    endwin();
    return 1;
  }
  
  //display data
  float pressure, temp, height;
  Vector3D accel, speed, omega, g_dir;
  //control data
  float throttle = 0, yaw_set = 0;
  Vector3D g_dir_set;
  bool att_hold = false;
  //inner processing
  float roll, pitch;
  bool att_hold_pressed = false;
  bool exit_pressed = false;
  bool exit = false;
  
  //main loop
  while (true) {
    
    //obtain Quad motion status from server (len = 60 bytes)
    while (recv(sockfd, buffer, 60, 0) > 0) {
      //decode data
      pressure = decode_f(buffer);
      temp = decode_f(buffer+4);
      height = decode_f(buffer+8);
      accel = decode_v(buffer+12);
      speed = decode_v(buffer+24);
      omega = decode_v(buffer+36);
      g_dir = decode_v(buffer+48);
      roll = asin (g_dir.y);
      pitch = asin (g_dir.x);
    }
      
    //print on screen
    mvprintw (4, 2, "roll: % f   pitch: % f", roll, pitch);
    mvprintw (6, 2, "angular speed: (% f, % f, % f)", omega.x, omega.y, omega.z);
    mvprintw (9, 2, "velocity: (% f, % f, % f)", speed.x, speed.y, speed.z);
    mvprintw (11, 2, "acceleration: (% f, % f, % f)", accel.x, accel.y, accel.z);
    mvprintw (13, 2, "height: % f", height);
    mvprintw (16, 2, "tempreature: % f   pressure: % f", temp, pressure);
    refresh();

    //get input
    Gamepad_processEvents();
    throttle = js->axisStates[0];
    yaw_set = js->axisStates[1];
    g_dir_set.x = js->axisStates[2];
    g_dir_set.y = js->axisStates[3];
    //att_hold
    if (js->buttonStates[0]) {
      if(!att_hold_pressed) {
	att_hold = att_hold ? false : true;
	att_hold_pressed = true;
      }
    } else {
      att_hold_pressed = false;
    }
    //exit
    if (js->buttonStates[1]) {
      if (!exit_pressed) {
	if (exit) {
	  break;
	} else {
	  exit = true;
	  mvprintw(row_max-1, 0, "press enter again to exit, cancel to cancel.");
	  refresh();
	}
	exit_pressed = true;
      }
    } else {
      exit_pressed = false;
    }
    //cancel
    if (js->buttonStates[2]) {
      if (exit) {
	mvprintw(row_max-1, 0, "cancelled");
	refresh();
	exit = false;
      }
      }
    
    //upload control signal (21 bytes)
    serialize (throttle, buffer);
    serialize (yaw_set, buffer+4);
    serialize (g_dir_set, buffer+8);
    *((char*)buffer+20) = att_hold;
    send (sockfd, buffer, 21, 0);
    
    usleep(DELAY_MILLISEC * 1000);
  }

  endwin();	
  return 0;
}


void serialize (float in, char *out) {
  memcpy(out, &in, 4);
}

void serialize (Vector3D in, char *out) {
  serialize(in.x, out);
  serialize(in.y, out+4);
  serialize(in.z, out+8);
}

float decode_f (char *in) {
  return *((float*)(void*)(in));
}

Vector3D decode_v (char *in) {
  return Vector3D(decode_f(in), decode_f(in+4), decode_f(in+8));
}
