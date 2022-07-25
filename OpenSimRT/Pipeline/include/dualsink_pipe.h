#ifndef PIPELINE_DUALSINK_HEADER_FBK_21072022
#define PIPELINE_DUALSINK_HEADER_FBK_21072022

#include "opensimrt_msgs/CommonTimed.h"
#include "std_srvs/Empty.h"
#include "Pipeline/include/common_node.h"

namespace Pipeline
{
	class DualSink:public Pipeline::CommonNode
	{
		public:

			DualSink();
			~DualSink();

			//this is a sink, I think. should be made into a class, but I think it will be complicated, so I didnt do it.
			ros::Subscriber sub2; //input2
			std::vector<std::string> input2_labels;
			virtual void callback2(const opensimrt_msgs::CommonTimedConstPtr& message) 
			{
				ROS_ERROR_STREAM("callback not implemented!");
			}


	};
}
#endif

