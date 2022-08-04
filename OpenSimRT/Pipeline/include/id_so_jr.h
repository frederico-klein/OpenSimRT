#ifndef PIPELINE_ID_SO_JR_HEADER_FBK_21072022
#define PIPELINE_ID_SO_JR_HEADER_FBK_21072022

#include "InverseDynamics.h"
#include "Pipeline/include/id.h"
#include "SignalProcessing.h"
#include "Visualization.h"
#include "opensimrt_msgs/CommonTimed.h"
#include "std_srvs/Empty.h"
#include <Common/TimeSeriesTable.h>

namespace Pipeline
{

	class IdSoJr:public Pipeline::Id
	{
		public:
			IdSoJr();
			~IdSoJr();
			// now since I have 2 sinks I will need message_filters
			void callback(const opensimrt_msgs::CommonTimedConstPtr& message_ik, const opensimrt_msgs::CommonTimedConstPtr& message_grf); //ik, grf are received at the same time

			void So();
			void Jr();

			void onInit();
			void onInitSo();
			void onInitJr();
			void finish();
	};

}
#endif

