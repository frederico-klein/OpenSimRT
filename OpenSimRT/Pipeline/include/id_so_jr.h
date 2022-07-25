#ifndef PIPELINE_ID_SO_JR_HEADER_FBK_21072022
#define PIPELINE_ID_SO_JR_HEADER_FBK_21072022

#include "Pipeline/include/dualsink_pipe.h"
#include "opensimrt_msgs/CommonTimed.h"

namespace Pipeline
{

	class IdSoJr:public Pipeline::DualSink
	{
		public:
			IdSoJr();
			~IdSoJr();

			void callback(const opensimrt_msgs::CommonTimedConstPtr& message); //ik
			void callback2(const opensimrt_msgs::CommonTimedConstPtr& message); //gfr

			// now since I have 2 sinks I will need message_filters



			void onInit();
			// now all the stuff I need to save between inInit and the callbacks

	};

}
#endif
