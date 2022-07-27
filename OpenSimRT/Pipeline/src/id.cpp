#include "ros/ros.h"
#include "opensimrt_msgs/CommonTimed.h"
#include "opensimrt_msgs/Labels.h"
#include "experimental/AccelerationBasedPhaseDetector.h"
#include "experimental/GRFMPrediction.h"
#include "INIReader.h"
#include "OpenSimUtils.h"
//#include "Settings.h"
#include "SignalProcessing.h"
#include "Utils.h"
#include "Visualization.h"
#include <Actuators/Thelen2003Muscle.h>
#include <Common/TimeSeriesTable.h>
#include <OpenSim/Common/STOFileAdapter.h>
#include <OpenSim/Common/CSVFileAdapter.h>
#include <exception>
#include <numeric>
#include <utility>
#include "ros/service_server.h"
#include "signal.h"
#include "std_srvs/Empty.h"
#include "Pipeline/include/id.h"

using namespace std;
using namespace OpenSim;
using namespace SimTK;
using namespace OpenSimRT;

Pipeline::Id::Id()
{
    
	// subject data
    INIReader ini(INI_FILE);
    auto section = "TEST_ID_FROM_FILE";
    subjectDir = DATA_DIR + ini.getString(section, "SUBJECT_DIR", "");
    auto modelFile = subjectDir + ini.getString(section, "MODEL_FILE", "");
    auto grfMotFile = subjectDir + ini.getString(section, "GRF_MOT_FILE", "");
    auto ikFile = subjectDir + ini.getString(section, "IK_FILE", "");

    auto grfRightApplyBody =
            ini.getString(section, "GRF_RIGHT_APPLY_TO_BODY", "");
    auto grfRightForceExpressed =
            ini.getString(section, "GRF_RIGHT_FORCE_EXPRESSED_IN_BODY", "");
    auto grfRightPointExpressed =
            ini.getString(section, "GRF_RIGHT_POINT_EXPRESSED_IN_BODY", "");
    auto grfRightPointIdentifier =
            ini.getString(section, "GRF_RIGHT_POINT_IDENTIFIER", "");
    auto grfRightForceIdentifier =
            ini.getString(section, "GRF_RIGHT_FORCE_IDENTIFIER", "");
    auto grfRightTorqueIdentifier =
            ini.getString(section, "GRF_RIGHT_TORQUE_IDENTIFIER", "");

    auto grfLeftApplyBody =
            ini.getString(section, "GRF_LEFT_APPLY_TO_BODY", "");
    auto grfLeftForceExpressed =
            ini.getString(section, "GRF_LEFT_FORCE_EXPRESSED_IN_BODY", "");
    auto grfLeftPointExpressed =
            ini.getString(section, "GRF_LEFT_POINT_EXPRESSED_IN_BODY", "");
    auto grfLeftPointIdentifier =
            ini.getString(section, "GRF_LEFT_POINT_IDENTIFIER", "");
    auto grfLeftForceIdentifier =
            ini.getString(section, "GRF_LEFT_FORCE_IDENTIFIER", "");
    auto grfLeftTorqueIdentifier =
            ini.getString(section, "GRF_LEFT_TORQUE_IDENTIFIER", "");

    memory = ini.getInteger(section, "MEMORY", 0);
    cutoffFreq = ini.getReal(section, "CUTOFF_FREQ", 0);
    delay = ini.getInteger(section, "DELAY", 0);
    splineOrder = ini.getInteger(section, "SPLINE_ORDER", 0);

    // setup model
    Object::RegisterType(Thelen2003Muscle());
    Model model(modelFile);
    OpenSimUtils::removeActuators(model);
    model.initSystem();

    // setup external forces
    //Storage grfMotion(grfMotFile);

    ExternalWrench::Parameters grfRightFootPar{
            grfRightApplyBody, grfRightForceExpressed, grfRightPointExpressed};
    grfRightLabels = ExternalWrench::createGRFLabelsFromIdentifiers(
            grfRightPointIdentifier, grfRightForceIdentifier,
            grfRightTorqueIdentifier);
    auto grfRightLoggerTemp = ExternalWrench::initializeLogger();
    grfRightLogger = &grfRightLoggerTemp; 

    ExternalWrench::Parameters grfLeftFootPar{
            grfLeftApplyBody, grfLeftForceExpressed, grfLeftPointExpressed};
    grfLeftLabels = ExternalWrench::createGRFLabelsFromIdentifiers(
            grfLeftPointIdentifier, grfLeftForceIdentifier,
            grfLeftTorqueIdentifier);
    auto grfLeftLoggerTemp = ExternalWrench::initializeLogger();
    grfLeftLogger = &grfLeftLoggerTemp;

    vector<ExternalWrench::Parameters> wrenchParameters;
    wrenchParameters.push_back(grfRightFootPar);
    wrenchParameters.push_back(grfLeftFootPar);

    // get kinematics as a table with ordered coordinates
    /*auto qTable = OpenSimUtils::getMultibodyTreeOrderedCoordinatesFromStorage(
            model, ikFile, 0.01);
	    */

    // setup filters
    LowPassSmoothFilter::Parameters ikFilterParam;
    ikFilterParam.numSignals = model.getNumCoordinates();
    ikFilterParam.memory = memory;
    ikFilterParam.delay = delay;
    ikFilterParam.cutoffFrequency = cutoffFreq;
    ikFilterParam.splineOrder = splineOrder;
    ikFilterParam.calculateDerivatives = true;
    ikfilter = new LowPassSmoothFilter(ikFilterParam);

    LowPassSmoothFilter::Parameters grfFilterParam;
    grfFilterParam.numSignals = 9;
    grfFilterParam.memory = memory;
    grfFilterParam.delay = delay;
    grfFilterParam.cutoffFrequency = cutoffFreq;
    grfFilterParam.splineOrder = splineOrder;
    grfFilterParam.calculateDerivatives = false;
    grfRightFilter = new LowPassSmoothFilter(grfFilterParam);
    grfLeftFilter = new LowPassSmoothFilter(grfFilterParam);

    // test with state space filter
    // StateSpaceFilter ikFilter({model.getNumCoordinates(), cutoffFreq});
    // StateSpaceFilter grfRightFilter({9, cutoffFreq}), grfLeftFilter({9,
    // cutoffFreq});

    // initialize id and logger
    id = new InverseDynamics(model, wrenchParameters);
    auto tauLoggerTemp = id->initializeLogger();
    tauLogger = &tauLoggerTemp;
    auto qLoggerTemp = id->initializeLogger();
    qLogger = &qLoggerTemp;
    auto qDotLoggerTemp = id->initializeLogger();
    qDotLogger = &qDotLoggerTemp;
    auto qDDotLoggerTemp = id->initializeLogger();
    qDDotLogger = &qDDotLoggerTemp;

    // visualizer
    visualizer = new BasicModelVisualizer(model);
    rightGRFDecorator = new ForceDecorator(Green, 0.001, 3);
    visualizer->addDecorationGenerator(rightGRFDecorator);
    leftGRFDecorator = new ForceDecorator(Green, 0.001, 3);
    visualizer->addDecorationGenerator(leftGRFDecorator);

    // mean delay
    sumDelayMS = 0;
    sumDelayMSCounter = 0;

    counter = 0;
}

Pipeline::Id::~Id()
{
	ROS_INFO_STREAM("Shutting down Id");
}

void Pipeline::Id::onInit() {
	previousTime = ros::Time::now().toSec();
	previousTimeDifference = 0;

	// when i am running this it is already initialized, so i have to add the loggers to the list I want to save afterwards
	initializeLoggers("grfRight",grfRightLogger);
	initializeLoggers("grfLeft", grfLeftLogger);
	initializeLoggers("tau",tauLogger);

}

ExternalWrench::Input Pipeline::Id::parse_message(const opensimrt_msgs::CommonTimedConstPtr & msg_grf, std::vector<std::string> grfLabels)
{
	ExternalWrench::Input a;
	
	cout << "grflabels: ";
	for (auto label:grfLabels)
	{
		//point, force, torque

		cout << label << " ";

	}
	cout << endl;

	cout << "input2_labels: ";
	for (auto message_label:input2_labels)
		{
			cout << message_label << " ";

		}
	cout << endl;
	return a;
}

void Pipeline::Id::callback(const opensimrt_msgs::CommonTimedConstPtr& message_ik, const opensimrt_msgs::CommonTimedConstPtr& message_grf) {
//void Pipeline::Id::operator() (const opensimrt_msgs::CommonTimedConstPtr& message) {
// repeat the simulation `simulationLoops` times
	ROS_DEBUG_STREAM("Received message. Running Id loop"); 
	counter++;
	//for (int k = 0; k < qTable.getNumRows() * simulationLoops; k++) {
	
	// well yes, but actually no.
	// get raw pose from table
	//auto qRaw_old = qTable.getRowAtIndex(i).getAsVector();
	//qRaw_old( qRaw_old +"dddd" +1);
	//std::vector<double> sqRaw = std::vector<double>(message->data.begin() + 1, message->data.end());
	SimTK::Vector qRaw(19); //cant find the right copy constructor syntax. will for loop it
	for (int j = 0;j < qRaw.size();j++)
	{
		//qRaw[j] = sqRaw[j];
		qRaw[j] = message_ik->data[j];
	}
	


	//is it the same
	//if (qRaw.size() != qRaw_old.size())
	//	ROS_FATAL("size is different!");
	//OpenSim::TimeSeriesTable i
	//get_from_subscriber(qRaw,t); //this will set qRaw and t from the subscribert

	//double t_old = qTable.getIndependentColumn()[i];
	
	/*
	 * This is kinda important. The time that matters is the time of the acquisition, I think
	 * The variable time it takes to calculate it doesn't matter too much, UNLESS, it is too big,
	 * then we should probably forget about it and not try to calculate anything!
	 * */
	//NO! I AM NOT SENDING TIME LIKE THIS ANYMORE.
	//double t = message_ik->header.stamp.toSec();
	double t = message_ik->time;
	double timediff = t- previousTime;
	double ddt = timediff-previousTimeDifference;
	if (std::abs(ddt) > 1e-5 )
		ROS_WARN_STREAM("Time difference greater than what our filter can handle: "<< std::setprecision(7) << ddt ); 
	previousTime = t;
	previousTimeDifference = timediff;
	ROS_DEBUG_STREAM("T (msg):"<< std::setprecision (15) << t);
	ROS_DEBUG_STREAM("DeltaT :"<< std::setprecision (15) << t);

	/*if (t_old - t > 0.1)
		ROS_ERROR("Reading from different timestamp! Did I lose a frame");
	else // same timestamp, so we check the indexes are okay.
	{
		for (int it=0; it < qRaw.size(); it++)
		{
			if (qRaw[it] - qRaw_old[it] > 0.1)
				ROS_ERROR("Difference too big");
		}
			
	}
*/
	//	ROS_DEBUG_STREAM("TAN" << qRaw);

	// increment the time by the total simulation time plus the sampling
	// period, to keep increasing after each simulation loop
//			t += loopCounter * (qTable.getIndependentColumn().back() + 0.01);

	

	// here I need to get the WRENCH, which is the GRF
	//
	// TODO: get wrench message!!!!!!!!!!

	//auto grfRightWrench = magic(message_grf);
	//auto grfLeftWrench = magic(message_grf);

	// setup ID inputn

	/*ExternalWrench::Input grfRightWrench = {grfmOutput.right.point,
						grfmOutput.right.force,
						grfmOutput.right.torque};
	ExternalWrench::Input grfLeftWrench = {grfmOutput.left.point,
					       grfmOutput.left.force,
					       grfmOutput.left.torque};
	*/
	cout << "right wrench.";
	ExternalWrench::Input grfRightWrench = parse_message(message_grf, grfRightLabels);
	cout << "left wrench.";
	ExternalWrench::Input grfLeftWrench = parse_message(message_grf, grfLeftLabels);

	return;



	// filter
	auto ikFiltered = ikfilter->filter({t, qRaw});
	auto q = ikFiltered.x;
	auto qDot = ikFiltered.xDot;
	auto qDDot = ikFiltered.xDDot;
	ROS_DEBUG_STREAM("Filter ran ok");
	// increment loop
/*			if (++i == qTable.getNumRows()) {
	    i = 0;
	    loopCounter++;
	}*/
	if (!ikFiltered.isValid) {
		ROS_DEBUG_STREAM("filter results are NOT valid");
		return; }
	ROS_DEBUG_STREAM("Filter results are valid");

	//filter wrench!
	//
	
        auto grfRightFiltered =
                grfRightFilter->filter({t, grfRightWrench.toVector()});
        grfRightWrench.fromVector(grfRightFiltered.x);
        auto grfLeftFiltered =
                grfLeftFilter->filter({t, grfLeftWrench.toVector()});
        grfLeftWrench.fromVector(grfLeftFiltered.x);

        if (!ikFiltered.isValid || !grfRightFiltered.isValid ||
            !grfLeftFiltered.isValid) {
            return;
        }

        // perform id
        chrono::high_resolution_clock::time_point t1;
        t1 = chrono::high_resolution_clock::now();
        auto idOutput = id->solve(
                {t, q, qDot, qDDot,
                 vector<ExternalWrench::Input>{grfRightWrench, grfLeftWrench}});

        chrono::high_resolution_clock::time_point t2;
        t2 = chrono::high_resolution_clock::now();
        sumDelayMS +=
                chrono::duration_cast<chrono::milliseconds>(t2 - t1).count();

	sumDelayMSCounter++;

	ROS_DEBUG_STREAM("inverse dynamics ran ok");

	// visualization
	try {
	visualizer->update(q);
	rightGRFDecorator->update(grfRightWrench.point,
				  grfRightWrench.force);
	leftGRFDecorator->update(grfLeftWrench.point, grfLeftWrench.force);
		ROS_DEBUG_STREAM("visualizer ran ok.");
	}
	catch (std::exception& e)
	{
		ROS_ERROR_STREAM("Error in visualizer. cannot show data!!!!!" <<std::endl << e.what());
	}

	try{

	// log data (use filter time to align with delay)
	if(false)
	{
		ROS_WARN_STREAM("THIS SHOULDNT BE RUNNING");

		tauLogger->appendRow(ikFiltered.t, ~idOutput.tau);
		grfRightLogger->appendRow(grfRightFiltered.t, ~grfRightFiltered.x);
		grfLeftLogger->appendRow(grfLeftFiltered.t, ~grfLeftFiltered.x);
		qLogger->appendRow(ikFiltered.t, ~q);
		qDotLogger->appendRow(ikFiltered.t, ~qDot);
		qDDotLogger->appendRow(ikFiltered.t, ~qDDot);

		ROS_INFO_STREAM("Added data to loggers. "<< counter);
	}}
	catch (std::exception& e)
	{
		ROS_WARN_STREAM("Error while updating loggers, data will not be saved" <<std::endl << e.what());
	}
	//if (counter > 720)
	//	write_();
	//}
}	
void Pipeline::Id::finish() {

    cout << "Mean delay: " << (double) sumDelayMS / sumDelayMSCounter << " ms"
         << endl;

    // Compare results with reference tables. Make sure that M, D,
    // spline order, fc are the same as the test.
    SimTK_ASSERT_ALWAYS(memory == 35,
                        "ensure that MEMORY = 35 in setup.ini for testing");
    SimTK_ASSERT_ALWAYS(delay == 14,
                        "ensure that DELAY = 35 setup.ini for testing");
    SimTK_ASSERT_ALWAYS(cutoffFreq == 6,
                        "ensure that CUTOFF_FREQ = 6 setup.ini for testing");
    SimTK_ASSERT_ALWAYS(splineOrder == 3,
                        "ensure that SPLINE_ORDER = 3 setup.ini for testing");
    OpenSimUtils::compareTables(
            *tauLogger,
            TimeSeriesTable(subjectDir + "real_time/inverse_dynamics/tau.sto"));
    OpenSimUtils::compareTables(
            *grfLeftLogger,
            TimeSeriesTable(subjectDir +
                            "real_time/inverse_dynamics/wrench_left.sto"));
    OpenSimUtils::compareTables(
            *grfRightLogger,
            TimeSeriesTable(subjectDir +
                            "real_time/inverse_dynamics/wrench_right.sto"));
    OpenSimUtils::compareTables(
            *qLogger,
            TimeSeriesTable(subjectDir +
                            "real_time/inverse_dynamics/q_filtered.sto"));
    OpenSimUtils::compareTables(
            *qDotLogger,
            TimeSeriesTable(subjectDir +
                            "real_time/inverse_dynamics/qDot_filtered.sto"));
    OpenSimUtils::compareTables(
            *qDDotLogger,
            TimeSeriesTable(subjectDir +
                            "real_time/inverse_dynamics/qDDot_filtered.sto"));
}
bool Pipeline::Id::see(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
	std::stringstream ss;
	//vector<string> data = grfRightLogger.getTableMetaData().getKeys();
	//ss << "Data Retrieved: \n";
	std::copy(grfRightLogger->getTableMetaData().getKeys().begin(), grfRightLogger->getTableMetaData().getKeys().end(), std::ostream_iterator<string>(ss, " "));
	ss << std::endl;
	ROS_INFO_STREAM("grfRightLogger columns:" << ss.str());
	return true;
}
void Pipeline::Id::write_() {
//			std::stringstream ss;
	//vector<string> data = grfRightLogger.getTableMetaData().getKeys();
	//ss << "Data Retrieved: \n";
//		  	std::copy(grfRightLogger.getTableMetaData().getKeys().begin(), grfRightLogger.getTableMetaData().getKeys().end(), std::ostream_iterator<string>(ss, " "));
//		  	ss << std::endl;
//			ROS_INFO_STREAM("grfRightLogger columns:" << ss.str());


/*	ROS_INFO_STREAM("I TRY WRITE sto");
	STOFileAdapter::write(grfRightLogger,"grfRight.sto");
	STOFileAdapter::write(grfLeftLogger,"grfLeft.sto");
	STOFileAdapter::write(tauLogger,"tau.sto");
	ROS_INFO_STREAM("I TRY WRITE csv");
	CSVFileAdapter::write(grfRightLogger,"grfRight.csv");
	CSVFileAdapter::write(grfLeftLogger,"grfLeft.csv");
	CSVFileAdapter::write(tauLogger,"tau.csv");
*/
	saveCsvs();
	saveStos();
	ROS_INFO_STREAM("i write");
}

