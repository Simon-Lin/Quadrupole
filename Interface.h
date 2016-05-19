//remote communication class - server side
#ifndef _INTERFACE_
#define _INTERFACE_

//port number of server program
#define PORTNO 13704
//maxium continous loss of packet before starting hovering
#define MAX_LOSS_PACKET 20

#include <stdint.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include "Data.h"

class Interface {
 public:
  Interface(Data *DATA_ref);
  ~Interface();

  bool initialize();
  bool resume();
  void startupLock();
  void start(); // Used with OpenMP parallellism feature. not realized for now.
  void update();
  
 private:
  Data *DATA;

  //socket data
  int sockfd, newsockfd;
  sockaddr_in serv_addr, cli_addr;
  char buffer[100];
  int lost_pack;

  //serialization
  void serialize (float in, char *out);
  void serialize (Eigen::Vector3f in, char *out);
  float decode_f (char *in);
  Eigen::Vector3f decode_v (char *out);
};

#endif
