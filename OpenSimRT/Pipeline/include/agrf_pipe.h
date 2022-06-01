#ifndef PIPELINE_AGRFM_HEADER_FBK_31052022
#define PIPELINE_AGRFM_HEADER_FBK_31052022

#include "opensimrt_msgs/CommonTimed.h"
#include "experimental/AccelerationBasedPhaseDetector.h"
#include "experimental/GRFMPrediction.h"
#include "SignalProcessing.h"
#include "Visualization.h"
#include <Common/TimeSeriesTable.h>
#include "ros/service_server.h"
#include "std_srvs/Empty.h"
#include "Pipeline/include/common_node.h"

namespace Pipeline
{
	class Acc:Pipeline::CommonNode
	{
		//TimeSeriesTable qTable;
		//int simulationLoops;
		//int loopCounter;
		OpenSimRT::LowPassSmoothFilter* filter;
		OpenSimRT::AccelerationBasedPhaseDetector* detector;
		OpenSimRT::GRFMPrediction* grfm;
		double sumDelayMS;
		int sumDelayMSCounter;
		//int i;
		OpenSimRT::InverseDynamics* id;
		OpenSimRT::BasicModelVisualizer* visualizer;
		OpenSimRT::ForceDecorator* rightGRFDecorator;
		OpenSimRT::ForceDecorator* leftGRFDecorator;
		OpenSim::TimeSeriesTable* grfRightLogger;
		OpenSim::TimeSeriesTable* grfLeftLogger;
		OpenSim::TimeSeriesTable* tauLogger;
		SimTK::Vec3 grfOrigin;
		std::string subjectDir;
		int counter;
		double previousTime, previousTimeDifference;

		public:
			Acc();
			~Acc();
			void onInit(); 
		
			void callback(const opensimrt_msgs::CommonTimedConstPtr& message) ;
			void finish(); 
			bool see(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
			void write_(); 
	};
}
#endif
