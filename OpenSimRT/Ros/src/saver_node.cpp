#include <Ros/include/saver_node.h>
#include "Ros/include/resolv_dir.h"
#include "opensimrt_msgs/SetFileNameSrv.h"
#include "ros/service_client.h"
#include "ros/service_server.h"
#include <Common/TimeSeriesTable.h>
#include <OpenSim/Common/STOFileAdapter.h>
#include <OpenSim/Common/CSVFileAdapter.h>
#include <memory>

//constructor
Ros::SaverNode::SaverNode(bool Debug)
{
	if( Debug && ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
		ros::console::notifyLoggerLevelsChanged();
	}
} 
void Ros::SaverNode::onInit()
{
	write_csv = nh.advertiseService("write_csv", &SaverNode::writeCsv, this);
	write_sto = nh.advertiseService("write_sto", &SaverNode::writeSto, this);
	
	startRecordingSrv 	= nh.advertiseService("start_recording", 	&SaverNode::startRecording, this);
	stopRecordingSrv 	= nh.advertiseService("stop_recording", 	&SaverNode::stopRecording, this);
	clearLoggersSrv 	= nh.advertiseService("clear_loggers", 		&SaverNode::clearLoggers, this);

	setNamePathSrv 		= nh.advertiseService("set_name_and_path", 	&SaverNode::setNamePath, this);

}


//destructor
Ros::SaverNode::~SaverNode()
{
	if(!at_least_one_logger_initialized)
		ROS_ERROR_STREAM("You forgot to initialize loggers!");
}


void Ros::SaverNode::initializeLoggers(std::string logger_name, OpenSim::TimeSeriesTable *logger)
{
	//adds loggers to loggers
	//ROS_ERROR_STREAM("initialization not implemented!");
	at_least_one_logger_initialized = true;
	ROS_INFO_STREAM("At least one logger was initialized! Will be able to save this with service");
	NamedTable this_named_table = std::make_pair(logger,logger_name);	
	loggers.push_back(this_named_table);

}
bool Ros::SaverNode::startRecording(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
	ROS_INFO_STREAM("startRecording service called.");
	recording= true;
	return true;
}
bool Ros::SaverNode::stopRecording(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
	ROS_INFO_STREAM("stopRecording service called.");
	recording_count++;
	recording=false;
	return true;
}

bool Ros::SaverNode::clearLoggers(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
	ROS_INFO_STREAM("clearLoggers service called.");
	clearLoggers();
	return true;
}

bool Ros::SaverNode::writeCsv(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
	ROS_INFO_STREAM("Write Csv service called.");
	saveCsvs();
	return true;
}
bool Ros::SaverNode::writeSto(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
	ROS_INFO_STREAM("Write Sto service called.");
	saveStos();
	return true;
}
void Ros::SaverNode::clearLoggers()
{
	for(NamedTable named_table:loggers)
	{
		//auto these_column_labels = named_table.first->getColumnLabels();
		ROS_WARN_STREAM("num_rows of table:" << named_table.first->getNumRows());
		//delete(named_table.first->updDependentColumn);
		//
		while(named_table.first->getNumRows() >0)
			named_table.first->removeRowAtIndex(0);
		//named_table.first = new OpenSim::TimeSeriesTable;
		//named_table.first->setColumnLabels(these_column_labels);
		ROS_WARN_STREAM("num_rows of table after deletion:" << named_table.first->getNumRows());
	}
}
void Ros::SaverNode::saveStos()
{
	for(NamedTable named_table:loggers)
	{
		auto loggerfilename = data_save_filename()+std::to_string(recording_count)+named_table.second+".sto";
		loggerfilename = resolve_file(data_save_dir(),loggerfilename);
		ROS_INFO_STREAM("trying to save: " << loggerfilename);
		OpenSim::STOFileAdapter::write(*named_table.first, loggerfilename);
	}
}
void Ros::SaverNode::saveCsvs()
{
	for(NamedTable named_table:loggers)
	{
		auto loggerfilename = data_save_filename()+std::to_string(recording_count)+named_table.second+".csv";
		loggerfilename = resolve_file(data_save_dir(),loggerfilename);
		ROS_INFO_STREAM("trying to save: " << loggerfilename);
		OpenSim::CSVFileAdapter::write(*named_table.first, loggerfilename);
	}

}

bool Ros::SaverNode::setNamePath(opensimrt_msgs::SetFileNameSrv::Request &req, opensimrt_msgs::SetFileNameSrv::Response &res)
{
	ROS_INFO_STREAM("name " << req.name << " path:"<< req.path);
	
	std::string dirname = resolve_dir(req.path);
	std::string filename = req.name;
	ROS_INFO_STREAM("dirname: " << dirname);
	nh.setParam("data_save_dir", dirname);
	nh.setParam("data_save_file", filename);
	return true;


}
