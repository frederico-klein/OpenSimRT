#ifndef ROS_HEADER_FBK_01022023
#define ROS_HEADER_FBK_01022023
#include "ros/node_handle.h"
#include "opensimrt_msgs/LabelsSrv.h"
#include "opensimrt_msgs/SetFileNameSrv.h"
#include "std_srvs/Empty.h"
#include "ros/service_client.h"
#include "ros/service_server.h"
#include <Common/TimeSeriesTable.h>

//idk whats going on
#include "Settings.h"

namespace Ros
{
	using NamedTable = std::pair<OpenSim::TimeSeriesTable*, std::string >; 
	using NamedTables = std::vector<NamedTable>;
	class SaverNode
	{
		public:
			SaverNode(bool Debug=true);
			~SaverNode();
			ros::NodeHandle nh{"~"};
			NamedTables loggers;
			bool recording = false;
			uint32_t recording_count = 0;
			std::string data_save_dir()
			{
				std::string data_save_dir_str; 
				nh.param<std::string>("data_save_dir",data_save_dir_str,"/tmp/");
				ROS_INFO_STREAM("current saving prefix: ");
				return data_save_dir_str;

			};
			std::string data_save_filename()
			{
				std::string data_save_filename_str; 
				nh.param<std::string>("data_save_file",data_save_filename_str,"file");
				ROS_INFO_STREAM("current saving file: ");
				return data_save_filename_str;

			};
			void onInit();
			bool at_least_one_logger_initialized = false;
			void saveStos();
			void saveCsvs();
			void clearLoggers();
			ros::ServiceServer write_csv, write_sto;
		protected:
			ros::ServiceServer startRecordingSrv, stopRecordingSrv, clearLoggersSrv, setNamePathSrv;
			bool writeCsv(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
			bool writeSto(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
			bool startRecording(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
			bool stopRecording(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
			bool clearLoggers(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
			bool setNamePath(opensimrt_msgs::SetFileNameSrv::Request &req, opensimrt_msgs::SetFileNameSrv::Response &res);
			void initializeLoggers(std::string logger_name, OpenSim::TimeSeriesTable *logger);
	};


}

#endif
