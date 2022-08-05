#include "ros/ros.h"
#include "opensimrt_msgs/CommonTimed.h"
#include "opensimrt_msgs/Labels.h"
#include "INIReader.h"
#include "OpenSimUtils.h"
//#include "Settings.h"
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
#include "Pipeline/include/id_so_jr.h"
#include "MuscleOptimization.h"

using namespace std;
using namespace OpenSim;
using namespace SimTK;
using namespace OpenSimRT;

Pipeline::IdSoJr::IdSoJr() // : Pipeline::Id() //this is done automatically I think, so I dont need to call it here
{
	//construct of So
	So();
	//construct of Jr
	Jr();

}

void Pipeline::IdSoJr::So()
{
    ROS_DEBUG_STREAM("fake constructor of SO");
    cout << "Warning" << endl
         << "This test might fail on different machines. "
         << "The performance of the optimization depends on the underlying OS. "
         << "We think it has to do with how threads are scheduled by the OS. "
         << "We did not observed this behavior with OpenSim v3.3." << endl
         << endl;

    // subject data
    INIReader ini(INI_FILE);
    auto section = "TEST_SO_FROM_FILE";
    auto subjectDir = DATA_DIR + ini.getString(section, "SUBJECT_DIR", "");
    auto modelFile = subjectDir + ini.getString(section, "MODEL_FILE", "");
    //auto ikFile = subjectDir + ini.getString(section, "IK_FILE", "");
    //auto idFile = subjectDir + ini.getString(section, "ID_FILE", "");

    // Windows places executables in different folders. When ctest is
    // called on a Linux machine it runs the test from different
    // folders and thus the dynamic library might not be found
    // properly.
//#ifndef WIN32
    auto momentArmLibraryPath =
            LIBRARY_OUTPUT_PATH + "/" +
            ini.getString(section, "MOMENT_ARM_LIBRARY", "");
    ROS_DEBUG_STREAM("momentArmLibraryPath:" << momentArmLibraryPath);
/*#else
    auto momentArmLibraryPath =
            ini.getString(section, "MOMENT_ARM_LIBRARY", "");
#endif*/

    //auto memory = ini.getInteger(section, "MEMORY", 0);
    //auto cutoffFreq = ini.getReal(section, "CUTOFF_FREQ", 0);
    //auto delay = ini.getInteger(section, "DELAY", 0);
    //auto splineOrder = ini.getInteger(section, "SPLINE_ORDER", 0);

    auto convergenceTolerance =
            ini.getReal(section, "CONVERGENCE_TOLERANCE", 0);
    auto memoryHistory = ini.getReal(section, "MEMORY_HISTORY", 0);
    auto maximumIterations = ini.getInteger(section, "MAXIMUM_ITERATIONS", 0);
    auto objectiveExponent = ini.getInteger(section, "OBJECTIVE_EXPONENT", 0);

    Object::RegisterType(Thelen2003Muscle());
    model = new Model(modelFile);
    model->initSystem();
 
    ROS_DEBUG_STREAM("registered model okay.");
    // load and verify moment arm function
    auto calcMomentArmTemp = OpenSimUtils::getMomentArmFromDynamicLibrary(
            *model, momentArmLibraryPath);
    ROS_DEBUG_STREAM("initialized calcMomentArmTemp from dynamic library ok.");

    calcMomentArm = calcMomentArmTemp;

    ROS_DEBUG_STREAM("initialized MomentArm from dynamic library ok.");
    // get kinematics as a table with ordered coordinates
    //auto qTable = OpenSimUtils::getMultibodyTreeOrderedCoordinatesFromStorage(
    //        model, ikFile, 0.01);

    // read external forces
    //auto tauTable = OpenSimUtils::getMultibodyTreeOrderedCoordinatesFromStorage(
    //        model, idFile, 0.01);

    /*if (tauTable.getNumRows() != qTable.getNumRows()) {
        THROW_EXCEPTION("ik and id storages of different size " +
                        toString(qTable.getNumRows()) +
                        " != " + toString(tauTable.getNumRows()));
    }*/

    // initialize so
//    MuscleOptimization::OptimizationParameters optimizationParameters;
    optimizationParameters.convergenceTolerance = convergenceTolerance;
    optimizationParameters.memoryHistory = memoryHistory;
    optimizationParameters.maximumIterations = maximumIterations;
    optimizationParameters.objectiveExponent = objectiveExponent;
    ROS_DEBUG_STREAM("set parameter for optimizer okay.");
    // auto tauResLogger = so.initializeResidualLogger();
    // visualizer
    //BasicModelVisualizer visualizer(model);
    visualizer = new BasicModelVisualizer(*model);
    // mean delay
    //int sumDelayMS = 0;

    so = new MuscleOptimization(*model, optimizationParameters, calcMomentArm);
    ROS_DEBUG_STREAM("initialized MuscleOptimization okay.");
    //so = &so_temp;
    ROS_DEBUG_STREAM("SO fake constructor ran ok.");
}
void Pipeline::IdSoJr::Jr()
{
	ROS_DEBUG_STREAM("fake constructor of Jr");
}
Pipeline::IdSoJr::~IdSoJr()
{
	ROS_INFO_STREAM("Shutting down Id_So_Jr");
}

void Pipeline::IdSoJr::onInit() {
	Pipeline::Id::onInit();
	onInitSo();
	onInitJr();
	
}

void Pipeline::IdSoJr::onInitSo()
{
	ROS_DEBUG_STREAM("onInitSo");
    //these need to be shared with the rest:
    fmLogger = so->initializeMuscleLogger();
    amLogger = so->initializeMuscleLogger();
}

void Pipeline::IdSoJr::onInitJr()
{
	ROS_DEBUG_STREAM("onInitJr");
}


void Pipeline::IdSoJr::callback(const opensimrt_msgs::CommonTimedConstPtr& message_ik, const opensimrt_msgs::CommonTimedConstPtr& message_grf) {
//void Pipeline::IdSoJr::operator() (const opensimrt_msgs::CommonTimedConstPtr& message) {
// repeat the simulation `simulationLoops` times
	ROS_ERROR_STREAM("Received message. Running Id loop"); 
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
	//cout << "right wrench.";
	ExternalWrench::Input grfRightWrench = parse_message(message_grf, grfRightIndexes);
	//cout << "left wrench.";
	//ROS_INFO_STREAM("rw");
	//print_wrench(grfRightWrench);
	ExternalWrench::Input grfLeftWrench = parse_message(message_grf, grfLeftIndexes);
	//ROS_INFO_STREAM("lw");
	//print_wrench(grfLeftWrench);
//	return;



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

	//so part
	//
	// I think that tau is ~idOutput.tau
	auto tau = idOutput.tau;
	for (auto somehting:tau)
	{
		cout << somehting << ";;";	
	}
	cout << endl;
        auto soOutput = so->solve({t, q, tau});

        /*chrono::high_resolution_clock::time_point t2;
        t2 = chrono::high_resolution_clock::now();
        sumDelayMS +=
                chrono::duration_cast<chrono::milliseconds>(t2 - t1).count();
*/
        // visualization
	// so loggers
	
	//jr part
	//
	//
	//
	//
	cout << soOutput.am << endl;
	cout << "two equal numbers:" << soOutput.am.size() << " " << model->getMuscles().getSize() << endl;
	// visualization
	try {
        visualizer->update(q, soOutput.am);
	//visualizer->update(q);
	/*rightGRFDecorator->update(grfRightWrench.point,
				  grfRightWrench.force);
	leftGRFDecorator->update(grfLeftWrench.point, grfLeftWrench.force);*/
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
        	
	// loggers from SO
		// log data (use filter time to align with delay)
        	fmLogger.appendRow(t, ~soOutput.fm);
        	amLogger.appendRow(t, ~soOutput.am);

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
void Pipeline::IdSoJr::finish() {
	Pipeline::Id::finish();

	//finish for other parts

}
