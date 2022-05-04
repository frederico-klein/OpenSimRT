#include <vector>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
// trying to make a class out of the example so I can use this upd server 

#include <ros/ros.h>
#include <tf/transform_listener.h>

class SimpleServer {
 public:
	SimpleServer(int, int, bool );
	~SimpleServer();
	char* buffer;
	char* hello;
	int buffersize;
	socklen_t sockfd, len, n;
	struct sockaddr_in servaddr, cliaddr;	
	//std::vector<double> receive();
	bool receive();
	bool ImSimple;
	tf::TransformListener listener;
	std::vector<double> output;
	std::vector<double> readTransformIntoOpensim(std::string);
	
};

