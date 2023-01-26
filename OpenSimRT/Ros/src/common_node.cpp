#include "Ros/include/common_node.h"
#include "ros/init.h"
#include "ros/node_handle.h"
#include "ros/publisher.h"
#include "ros/rate.h"
#include "ros/ros.h"
#include "opensimrt_msgs/CommonTimed.h"
#include "opensimrt_msgs/PosVelAccTimed.h"
#include "opensimrt_msgs/LabelsSrv.h"
#include "std_srvs/Empty.h"
#include "ros/service_client.h"
#include "ros/service_server.h"
#include "ros/subscriber.h"
#include <Common/TimeSeriesTable.h>
#include <OpenSim/Common/STOFileAdapter.h>
#include <OpenSim/Common/CSVFileAdapter.h>
#include <memory>

//constructor
Ros::CommonNode::CommonNode(bool Debug)
{
	if( Debug && ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
		ros::console::notifyLoggerLevelsChanged();
	}
} 
void Ros::CommonNode::onInit(int num_sinks)
{
	pub = nh.advertise<opensimrt_msgs::CommonTimed>("output", 1000);
	if (publish_filtered)
		pub_filtered = nh.advertise<opensimrt_msgs::PosVelAccTimed>("output_filtered", 1000);
	outLabelsSrv = nh.advertiseService("out_labels", &CommonNode::outLabels, this);

	if (num_sinks == 1)
	{
		ROS_INFO_STREAM("registering ik callback");
		//register the callback
		sub.subscribe(nh, "input",10);
		sub.registerCallback(&CommonNode::callback,this);
		sub_filtered.subscribe(nh, "input_filtered",10);
		sub_filtered.registerCallback(&CommonNode::callback_filtered,this);
		//sub = nh.subscribe("input",5, &CommonNode::callback, this);
	}
	else
	{
		ROS_INFO_STREAM("not single_sink, callback isnt registered yet!");
	}
	ros::Rate r(1);
	//output_labels = qTable.getColumnLabels();
	opensimrt_msgs::LabelsSrv l;
	//while(!ros::service::call("in_labels", l))
	if (num_sinks > 0)
	{
		while(ros::ok())
		{
			if(ros::service::call("in_labels", l))
			{
				input_labels = l.response.data;
				if (input_labels.size()==0)
					ROS_ERROR_STREAM("Got labels with size 0! If try to unshuffle data, this will likely fail!");
				ROS_DEBUG_STREAM("got response for labels: "<< input_labels.size() << " elements.");
				input.set(input_labels);
				break;
			}
			ros::spinOnce();
			ROS_INFO_STREAM("Waiting to read input labels."); 
			r.sleep();
		}
	}
	else if (num_sinks==0)
	{
		ROS_INFO_STREAM("Source node registered (num_sinks = 0)");	    

	}
	else
	{
		ROS_ERROR_STREAM("Unexpected number of sinks:" << num_sinks);

	}
	write_csv = nh.advertiseService("write_csv", &CommonNode::writeCsv, this);
	write_sto = nh.advertiseService("write_sto", &CommonNode::writeSto, this);
	
	startRecordingSrv 	= nh.advertiseService("start_recording", 	&CommonNode::startRecording, this);
	stopRecordingSrv 	= nh.advertiseService("stop_recorging", 	&CommonNode::stopRecording, this);


}

void Ros::Reshuffler::set(std::vector<std::string> labels)
{
	if (desired_label_order.size() == 0)
	{
		ROS_WARN_STREAM("Label order not set. Data may not be aligned properly and results will be incorrect.");	
	}
	else
	{
		deshuffle_xput = find_matches(desired_label_order,labels);
		ROS_INFO_STREAM("Label order set. Make sure you are not directly accessing elements, but using something like 'raw_input[deshuffle_xput[i]]'");
	}
}

//destructor
Ros::CommonNode::~CommonNode()
{
	if(!at_least_one_logger_initialized)
		ROS_ERROR_STREAM("You forgot to initialize loggers!");
}
bool Ros::CommonNode::outLabels(opensimrt_msgs::LabelsSrv::Request & req, opensimrt_msgs::LabelsSrv::Response& res )
{
	if (output_labels.size() == 0)
		ROS_ERROR_STREAM("output_labels variable not set!!!! Your client will not be able to know what is what so this will possibly crash.");
	res.data = output_labels;
	published_labels_at_least_once = true;
	ROS_INFO_STREAM("CALLED LABELS SRV");
	return true;
}


void Ros::CommonNode::initializeLoggers(std::string logger_name, OpenSim::TimeSeriesTable *logger)
{
	//adds loggers to loggers
	//ROS_ERROR_STREAM("initialization not implemented!");
	at_least_one_logger_initialized = true;
	ROS_INFO_STREAM("At least one logger was initialized! Will be able to save this with service");
	NamedTable this_named_table = std::make_pair(logger,logger_name);	
	loggers.push_back(this_named_table);

}
bool Ros::CommonNode::startRecording(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
	ROS_INFO_STREAM("startRecording service called.");
	recording= true;
	return true;
}
bool Ros::CommonNode::stopRecording(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
	ROS_INFO_STREAM("stopRecording service called.");
	recording_count++;
	return true;
}


bool Ros::CommonNode::writeCsv(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
	ROS_INFO_STREAM("Write Csv service called.");
	saveCsvs();
	return true;
}
bool Ros::CommonNode::writeSto(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
	ROS_INFO_STREAM("Write Sto service called.");
	saveStos();
	return true;
}
void Ros::CommonNode::saveStos()
{
	for(NamedTable named_table:loggers)
	{
		auto loggerfilename = data_save_dir()+named_table.second+std::to_string(recording_count)+".sto";
		ROS_INFO_STREAM("trying to save: " << loggerfilename);
		OpenSim::STOFileAdapter::write(*named_table.first, loggerfilename);
	}
}
void Ros::CommonNode::saveCsvs()
{
	for(NamedTable named_table:loggers)
	{
		auto loggerfilename = data_save_dir()+named_table.second+std::to_string(recording_count)+".csv";
		ROS_INFO_STREAM("trying to save: " << loggerfilename);
		OpenSim::CSVFileAdapter::write(*named_table.first, loggerfilename);
	}

}

