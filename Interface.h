//remote communication class - server side

//port number of server program
#define PORTNO 13704
//maxium continous loss of packet before starting hovering
#define MAX_LOSS_PACKET 20

#include <stdint.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include "Vector3D.h"

struct InterfaceData {
  //control signal revieced from user
  float throttle, yaw_set;
  Vector3D g_direction_set;
  bool att_hold, power_off;

  //data passed to the UI display
  float pressure, temperature, height;
  Vector3D acceleration, speed, angular_speed, g_direction;
};

class Interface {
 public:
  Interface(InterfaceData *DATA_ref);
  ~Interface();

  bool initialize();
  void start(); // Used with OpenMP parallellism feature. not realized for now.
  void update();
  
 private:
  InterfaceData *DATA;

  //socket data
  int sockfd, newsockfd;
  sockaddr_in serv_addr, cli_addr;
  char buffer[100];
  int lost_pack;

  //serialization
  void serialize (float in, char *out);
  void serialize (Vector3D in, char *out);
  float decode_f (char *in);
  Vector3D decode_v (char *out);
};
