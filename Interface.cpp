#include "Interface.h"
#include <stdio.h>
#include <sys/ioctl.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>

Interface::Interface (InterfaceData *DATA_ref) {
  DATA = DATA_ref;
  lost_pack = 0;
}

Interface::~Interface () {
  shutdown(sockfd,2);
}

bool Interface::initialize() {
  //initailze socket
  if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
    perror("cannot open socket");
    return 1;
  }

  bzero((char*) &serv_addr, sizeof(serv_addr));
  serv_addr.sin_family = AF_INET;
  serv_addr.sin_addr.s_addr = htonl(INADDR_ANY);
  serv_addr.sin_port = htons(PORTNO); //portno: Fine structure constant

  if (bind(sockfd, (struct sockaddr*)&serv_addr, sizeof(serv_addr)) < 0) {
    perror("socket binding failed");
    return 1;
  }

  //waiting initialization connection from client side
  printf ("socket initiailization complete on port %d. listening...\n", PORTNO);
  fflush(stdout);
  socklen_t clilen = sizeof(cli_addr);
  char tmpbuf[5] = {0};
  if ((recvfrom(sockfd, tmpbuf, 5, 0, (struct sockaddr*)&cli_addr, &clilen)) < 0) {
    perror("revieving data failed");
    return 1;
  }
  if (strcmp(tmpbuf, "QUAD")) {
    printf("WRONG connection from the client side.\n");
    return 1;
  }
  //send ACK signal
  if (connect(sockfd, (struct sockaddr*)&cli_addr, clilen) < 0) {
    perror("error connecting client");
    return 1;
  }
  if (send(sockfd, tmpbuf, 5, 0) < 0) {
    perror("sending to client failed");
    return 1;
  }

  //initialization complete, setting socket to non-blocking
  if (fcntl(sockfd, F_SETFL, fcntl(sockfd, F_GETFL, 0) | O_NONBLOCK)) {
    perror("setting socket to non-blocking failed");
    return 1;
  }
  
  printf ("connection established. socket initialization complete.\n");
  return 0;
}


void Interface::update () {
  //encode display data
  serialize (DATA->pressure, buffer);
  serialize (DATA->temperature, buffer+4);
  serialize (DATA->height, buffer+8);
  serialize (DATA->acceleration, buffer+12);
  serialize (DATA->speed, buffer+24);
  serialize (DATA->angular_speed, buffer+36);
  serialize (DATA->g_direction, buffer+48);
  
  //send to client (total length = 3*4 + 4*12 = 60 bytes)
  if (send (sockfd, buffer, 60, 0) < 0) {
    perror("ERROR sending data to client");
  }

  //recieve from client (total length = 2*4 + 1*12 + 1 = 21 bytes)
  bool loss = true;
  while (recv (sockfd, buffer, 21, 0) > 0) { //clear buffer
    loss = false;
  }
  if (loss) {
    if (errno != EAGAIN) {
      perror("ERROR recieving controlling packet");
    }
    lost_pack++;
    printf("packet(s) lost: %d\n", lost_pack);
    //enter hovering mode if lost too many packets
    if (lost_pack == MAX_LOSS_PACKET) {
      printf("lost packet number exceeds acceptable value. entering hovering mode.\n");
      DATA->att_hold = true;
      DATA->yaw_set = 0;
      DATA->g_direction_set.setValue(0,0,-1);
    }
  } else {
    lost_pack = 0;
    //decode packet data
    DATA->throttle = decode_f (buffer);
    DATA->yaw_set = decode_f (buffer+4);
    DATA->g_direction_set = decode_v (buffer+8);
    DATA->att_hold = (((buffer + 20) >> 2) == 0);
    DATA->power_off = (((buffer + 20) << 2) == 0);
  }
}


void Interface::serialize (float in, char *out) {
  memcpy(out, &in, 4);
}

void Interface::serialize (Vector3D in, char *out) {
  serialize(in.x, out);
  serialize(in.y, out+4);
  serialize(in.z, out+8);
}

float Interface::decode_f (char *in) {
  return *((float*)(void*)(in));
}

Vector3D Interface::decode_v (char *in) {
  return Vector3D(decode_f(in), decode_f(in+4), decode_f(in+8));
}
