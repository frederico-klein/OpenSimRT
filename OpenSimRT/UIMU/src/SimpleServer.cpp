// Server side implementation of UDP client-server model

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include "SimpleServer.h"

#include <iostream>
#include <sstream>
#include <iterator>
#include <vector>

#include <ros/ros.h>
#include <tf/transform_listener.h>

// Driver code
SimpleServer::SimpleServer (int PORT = 8080, int MAXLINE = 4096, bool Simple = true) {

	if (Simple){
		ImSimple = true;	
		//socklen_t sockfd;
		buffer = new char[MAXLINE];
		hello = "Hello from server";
		buffersize = MAXLINE;
		//struct sockaddr_in servaddr, cliaddr;
		
		// Creating socket file descriptor
		if ( (sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0 ) {
			perror("socket creation failed");
			exit(EXIT_FAILURE);
		}
		
		memset(&servaddr, 0, sizeof(servaddr));
		memset(&cliaddr, 0, sizeof(cliaddr));
		
		// Filling server information
		servaddr.sin_family = AF_INET; // IPv4
		servaddr.sin_addr.s_addr = INADDR_ANY;
		servaddr.sin_port = htons(PORT);
		
		// Bind the socket with the server address
		if ( bind(sockfd, (const struct sockaddr *)&servaddr,
				sizeof(servaddr)) < 0 )
		{
			perror("bind failed");
			exit(EXIT_FAILURE);
		}
		
		//socklen_t len, n;

		len = sizeof(cliaddr); //len is value/resuslt
	}
	else
	{
		///read one tf and make it for everyone
		tf::TransformListener listener;
	}
}

SimpleServer::~SimpleServer(void)
{
}

//std::vector<double> SimpleServer::receive()
bool SimpleServer::receive()
{
	if (ImSimple){
		n = recvfrom(sockfd, (char *)buffer, buffersize,
					MSG_WAITALL, ( struct sockaddr *) &cliaddr,
					&len);
		//now I need to find if there is the word BYE in it
		buffer[n] = '\0';
		//rosdebug?
		//printf("Client : %s\n", buffer);
		if (strcmp(buffer, "BYE!") == 0 )
		{
		       std::cout << "received goodbye SS OK." << std::endl;
		       return false;
		}

		std::stringstream ss;
		ss << buffer;
		std::istream_iterator<std::string> begin(ss), end;
		std::vector<std::string> vstrings(begin, end);
		//std::copy(vstrings.begin(), vstrings.end(), std::ostream_iterator<std::string>(std::cout, "\n"));
		std::vector<double> myvec;

		for (auto a_str : vstrings)
		{
			myvec.push_back(std::stod(a_str));
		}
		////THIS IS KINDA WORKING. I THINK
		//for (auto a_double : myvec )
		//	std::cout << a_double << "|";
		//std::cout << std::endl ;

		//printf("Client : %s\n", buffer);
		sendto(sockfd, (const char *)hello, strlen(hello),
			MSG_CONFIRM, (const struct sockaddr *) &cliaddr,
				len);
		//todo: rosdebug or somehting
		//printf("Hello message sent.\n");
		
		output = myvec;
		//return myvec;
		//for( auto i:myvec)
		//	ROS_INFO_STREAM("THIS THING" << i);
		//ROS_INFO_STREAM("THIS THING:" << myvec.size());
		return true;
	}
	else
	{
		std::vector<double> myvec;
		// now i need to set this myvec with the values i read for the quaternions somehow
	        std::vector<std::string> tf_strs = {"/a", "b", "c"};	
		double time = ros::Time::now().toSec();// i need to get this from the transform somehow
		myvec.push_back(time);

		for (auto i:tf_strs)
		{
			std::vector<double> anImu = readTransformIntoOpensim(i);
			myvec.insert(myvec.end(),anImu.begin(), anImu.end());
		}
		//for( auto i:myvec)
		//	ROS_INFO_STREAM("THIS THING" << i);
		//ROS_INFO_STREAM("THIS THING:" << myvec.size());
		
		output = myvec;
		return true;	
	}
}

std::vector<double> SimpleServer::readTransformIntoOpensim(std::string tf_name)
{

	//rosreceive its wrong it should be on a callback basis, 
	tf::StampedTransform transform;
	try{
		listener.waitForTransform("/map", tf_name, ros::Time(0), ros::Duration(3.0));
		listener.lookupTransform("/map", tf_name,
				ros::Time(0), transform);
	}
	catch (tf::TransformException ex){
		ROS_ERROR("%s",ex.what());
		//ros::Duration(1.0).sleep();
	}
	std::vector<double> myvec;
	// now i need to set this myvec with the values i read for the quaternions somehow
	
	auto myq = transform.getRotation();

		myvec.push_back(myq.z());
		myvec.push_back(myq.x());
		myvec.push_back(myq.y());
		myvec.push_back(myq.w());
		// now it is a bunch of zeros
		myvec.insert(myvec.end(), 14, -0.010 );
	//for( auto i:myvec)
	//	ROS_INFO_STREAM("THIS THING" << i);
	//ROS_INFO_STREAM("THIS THING:" << myvec.size());
	
	return myvec;

}

