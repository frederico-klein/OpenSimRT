#include "ros/ros.h"
#include "ros/service_server.h"
#include "signal.h"
#include "Pipeline/include/agrf_pipe.h"

void mySigintHandler(int sig)
{
    // Do custom action, like publishing stop msg
    //perenial.write();
    ros::shutdown();
}
int main(int argc, char **argv) {
	ros::init(argc, argv, "swallow_show");
	ROS_INFO_STREAM("called node AccelerationBasedPhaseDetector. aka swallow_show");
			ros::NodeHandle n;
    try {
	Pipeline::Acc perenial;

	//	signal(SIGINT, mySigintHandler);
	perenial.onInit();		
			//ros::Subscriber sub = n.subscribe<opensimrt_msgs::CommonTimed>("r_data", 1, perenial);	
	ros::ServiceServer seecsv = n.advertiseService("see", &Pipeline::Acc::see, &perenial);
	ros::spin();
    } catch (std::exception& e) {
        std::cout << e.what() << std::endl;
        return -1;
    }
    ROS_INFO_STREAM("Goodbye!");
    return 0;
}

