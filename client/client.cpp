#define PORTNO 13704
#define DELAY_MILLISEC 100

#include <ncurses.h>
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <arpa/inet.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <netinet/in.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/time.h>
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
  system("clear");
  if (argc == 1 || (argc == 3 && strcmp(argv[2], "-r"))) {
  	printf("usage: client [serverside IP] [-r]\n");
  	return 1;
  }
  printf("Quadrupole client side ver 0.1\n");


  //check for joystick
  Gamepad_init();
  Gamepad_detectDevices();
  Gamepad_device *js = NULL;
  if (Gamepad_numDevices() > 0) {
    js = Gamepad_deviceAtIndex(0);
    printf("joystick: %s detected.\n", js->description);
  } else {
    printf("no joysticks detected. exiting...\n");
    return 1;
  }
  
  //socket initialization
  int sockfd;
  sockaddr_in serv_addr, client_addr;
  char buffer[100];
  printf("initalizing socket...\n");
  fflush(stdout);

  if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
      perror("cannot open socket");
      return 1;
  }
  bzero((char*) &client_addr, sizeof(client_addr));
  client_addr.sin_family = AF_INET;
  client_addr.sin_addr.s_addr = htonl (INADDR_ANY);
  client_addr.sin_port = htons(PORTNO);
  if (bind(sockfd, (sockaddr*)&client_addr, sizeof(client_addr)) < 0) {
    perror("socket binding failed");
    return 1;
  }
  
  bzero((char*) &serv_addr, sizeof(serv_addr));
  serv_addr.sin_family = AF_INET;
  serv_addr.sin_port = htons(PORTNO);
  if (inet_aton((argv[1]), &serv_addr.sin_addr) == 0) {
    printf("not valid IP format. exiting...\n");
    return 1;
  }

  printf("establishing connection...\n");
  fflush(stdout);
  int servlen = sizeof(serv_addr);
  if (connect(sockfd, (struct sockaddr*)&serv_addr, servlen) < 0) {
    perror("cannot establish connection");
    return 1;
  }

  //if in resume connection mode then skip handshaking
  if (argc == 2) {
    
    //send ACK signal to server
    char ackbuf[] =  "QUAD";
    if(send(sockfd, ackbuf, 5, 0) < 0) {
      perror("cannot send ack signal to server");
      return 1;
    }
    
    //wait for server's responce
    fd_set fds;
    FD_ZERO (&fds);
    FD_SET (sockfd, &fds);
    timeval tv;
    tv.tv_sec = 10;
    tv.tv_usec = 0;
    int flag = select(FD_SETSIZE, &fds, NULL, NULL, &tv);
    if (flag < 0) {
      perror("select: ");
      return 1;
    } else if (flag == 0) {
    printf("ERROR: connection timeout. exiting...\n");
    return 1;
    }
    if(recv(sockfd, ackbuf, 5, 0) < 0) {
      perror("cannot establish connection");
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
  }
  
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
  
  //display data
  float pressure, temp, height;
  Vector3D accel, speed, omega, g_dir;
  //control data
  float throttle = 0, yaw_set = 0;
  Vector3D g_dir_set;
  bool att_hold = false;
  bool increment_mode = false;
  bool exit = false;	
  //inner processing
  float roll, pitch;
  bool startup_lock = false;
  bool att_hold_pressed = false;
  bool increment_pressed = false;
  bool exit_pressed = false;
  bool exit_check = false;

  mvprintw(row_max-1, 0, "Ready for flight. Press START to unlock.");
  
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
      roll = asin (g_dir.y) * 180 / M_PI;
      pitch = asin (g_dir.x) * 180 / M_PI;
      
      //print on screen
      mvprintw (4, 2, "roll: % f   pitch: % f", roll, pitch);
      mvprintw (5, 2, "angular speed: (% f, % f, % f)", omega.x, omega.y, omega.z);
      mvprintw (6, 2, "velocity: (% f, % f, % f)", speed.x, speed.y, speed.z);
      mvprintw (7, 2, "acceleration: (% f, % f, % f)", accel.x, accel.y, accel.z);
      mvprintw (9, 2, "height: % f", height);
      mvprintw (10, 2, "tempreature: % f   pressure: % f", temp, pressure);
    }

    //get input
    Gamepad_processEvents();
    if (startup_lock) {
      if (!increment_mode) {
	throttle = 0.5 * (1 - js->axisStates[1]);
      }
    }
    yaw_set = js->axisStates[0];
    //max tilt angle: 30 degrees
    g_dir_set.x =  0.5 * js->axisStates[3];
    g_dir_set.y = -0.5 * js->axisStates[4];
    g_dir_set.z = sqrt(1 - g_dir_set.x*g_dir_set.x - g_dir_set.y*g_dir_set.y);
    
    //print on screen
    mvprintw (13, 2, "input value:");
    mvprintw (14, 2, "throttle: % f, yaw_set: % f, roll_set: % f, pitch_set: % f", throttle, yaw_set, asin(g_dir_set.x)*180/M_PI, asin(g_dir_set.y)*180/M_PI);
    
    //att_hold
    if (js->buttonStates[8]) {
      att_hold_pressed = true;
    } else {
      if (att_hold_pressed) {
        att_hold = att_hold ? false : true;
        att_hold_pressed = false;
      }
    }
    
    if(att_hold) {
    mvprintw (15, 2, "ATT_HOLD = ON ");
    } else {
    mvprintw (15, 2, "ATT_HOLD = OFF");
    }
    refresh();

    //increment mode
    if (js->buttonStates[2]) {
      if (!increment_pressed) {
	increment_pressed = true;
      }
    } else {
      if (increment_pressed) {
	if (!increment_mode) {
	  throttle = 0;
	  increment_mode = true;
	  mvprintw (15, 18, "INCREMENT_MODE");
	} else {
	  increment_mode = false;
	  mvprintw (15, 18, "              ");
	}
	increment_pressed = false;
      }
    }

    //increment mode throttle
    if (increment_mode) {
      if (js->axisStates[5] == 0) {
	if (js->axisStates[6] == -1) {
	  throttle += 0.01;
	  if (throttle > 1) throttle = 1;
	}
	if (js->axisStates[6] == 1) {
	  throttle -= 0.01;
	  if (throttle < 0) throttle = 0;
	}
      }
    }
    
    //exit and startup
    if (js->buttonStates[9]) {
      if (!exit_pressed) {
	exit_pressed = true;
      }
    } else {
      if (exit_pressed) {
	if (!startup_lock) {
	  exit = startup_lock = true;
	  mvprintw(row_max-1, 0, "                                          ");
	} else if (exit_check) {
	  startup_lock = false;
	  exit = true;
	} else {
	  exit_check = true;
 	  mvprintw(row_max-1, 0, "press ENTER again to exit, BACK to cancel.");
	  refresh();
	}
	exit_pressed = false;
      }
    }
    
    //cancel
    if (js->buttonStates[10]) {
      if (exit_check) {
	mvprintw(row_max-1, 0, "cancelled                                 ");
	refresh();
	exit_check = false;
      }
    }
    
    //upload control signal (21 bytes)
    serialize (throttle, buffer);
    serialize (yaw_set, buffer+4);
    serialize (g_dir_set, buffer+8);
    buffer[20] = 0;
    if (att_hold) buffer[20] += 1;
    if (exit) buffer[20] += (1 << 4);
    send (sockfd, buffer, 21, 0);

    if (exit && startup_lock) {
      exit = false;
    } else if (exit == true && startup_lock == false) {
      break;
    }
    
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
