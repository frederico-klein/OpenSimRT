#include "Ros/include/common_node.h"
#include "ros/init.h"
#include "ros/rate.h"
#include "ros/ros.h"
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
	//so much complexity to save a little bit of bandwidth... anyway, this needs to me moved to reshuffler. Also publishing and subscribing nodes need to be setup from resshuffler inputs. Dual_sinks need to be then simplified to account for this. 
	nh.getParam("input_labels",input_labels);
	if (input_labels.size()>0)
		input_size = input_labels.size();
	//while(!ros::service::call("in_labels", l))
	if (num_sinks > 0)
	{
		while(ros::ok() && input_labels.size()==0)
		{
			if(ros::service::call("in_labels", l))
			{
				input_labels = l.response.data;
				if (input_labels.size()==0)
					ROS_ERROR_STREAM("Got labels with size 0! If try to unshuffle data, this will likely fail!");
				ROS_DEBUG_STREAM("got response for labels: "<< input_labels.size() << " elements.");
				input.set(input_labels);
				input_size = input_labels.size();
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
	Ros::SaverNode::onInit();

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


