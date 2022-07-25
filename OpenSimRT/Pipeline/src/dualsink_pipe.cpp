#include "Pipeline/include/dualsink_pipe.h"
#include "ros/node_handle.h"
#include "ros/rate.h"
#include "opensimrt_msgs/LabelsSrv.h"
#include "ros/service.h"
#include "ros/subscriber.h"

Pipeline::DualSink::DualSink()
{
	if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
	   ros::console::notifyLoggerLevelsChanged();
	}
    sub2 = nh.subscribe("input2",5, &DualSink::callback2, this);

    ros::Rate r(1);
    opensimrt_msgs::LabelsSrv l;
    //while(!ros::service::call("in_labels", l))
    while(ros::ok())
    {
    	if(ros::service::call("in_labels2", l))
	{
	    input2_labels = l.response.data;
	    break;
	}
	ros::spinOnce();
	ROS_INFO_STREAM("Waiting to read input2 labels."); 
	r.sleep();
    }

} //constructor
Pipeline::DualSink::~DualSink()
{
	ROS_INFO_STREAM("Closing DualSink!");
}
