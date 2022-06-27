#ifndef TFSERVER_H_FBK20220627
#define TFSERVER_H_FBK20220627

#include <vector>
/*#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>*/
// trying to make a class out of the example so I can use this upd server 

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include "OrientationProvider.h"

class TfServer: public OrientationProvider {
 public:
	TfServer();
	~TfServer();
	bool receive();
	tf::TransformListener listener;
	//std::vector<double> output;
	std::vector<double> readTransformIntoOpensim(std::string);
	
};
#endif
