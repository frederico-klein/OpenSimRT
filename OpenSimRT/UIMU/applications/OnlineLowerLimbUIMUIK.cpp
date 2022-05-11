/**
 * -----------------------------------------------------------------------------
 * Copyright 2019-2021 OpenSimRT developers.
 *
 * This file is part of OpenSimRT.
 *
 * OpenSimRT is free software: you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * OpenSimRT is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE.  See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * OpenSimRT. If not, see <https://www.gnu.org/licenses/>.
 * -----------------------------------------------------------------------------
 *
 * @file TestLowerLimbIMUIKFromFile.cpp
 *
 * @brief Test IK with prerecorded NGIMU data on the lower body.
 *
 * @author Filip Konstantinos <filip.k@ece.upatras.gr>
 */
#include "ros/ros.h"
#include "opensimrt_msgs/Common.h"
#include "opensimrt_msgs/Labels.h"

#include "IMUCalibrator.h"
#include "INIReader.h"
#include "InverseKinematics.h"
#include "UIMUInputDriver.h"
#include "OpenSimUtils.h"
#include "Settings.h"
#include "Utils.h"
#include "Visualization.h"
#include <Actuators/Thelen2003Muscle.h>
#include <Common/TimeSeriesTable.h>
//#include <OpenSim/Common/STOFileAdapter.h>
#include <OpenSim/Common/CSVFileAdapter.h>
#include <signal.h> // only for debug. should be removed if it doesnt help

using namespace std;
using namespace OpenSim;
using namespace OpenSimRT;
using namespace SimTK;

void run() {
	signal(SIGPIPE,SIG_IGN); // I think I only need this to debug. remove if it doesn't change anythign
    INIReader ini(INI_FILE);
    auto section = "LOWER_LIMB_NGIMU_OFFLINE";

    // imu calibration settings
    auto imuDirectionAxis = ini.getString(section, "IMU_DIRECTION_AXIS", "");
    auto imuBaseBody = ini.getString(section, "IMU_BASE_BODY", "");
    auto xGroundRotDeg = ini.getReal(section, "IMU_GROUND_ROTATION_X", 0);
    auto yGroundRotDeg = ini.getReal(section, "IMU_GROUND_ROTATION_Y", 0);
    auto zGroundRotDeg = ini.getReal(section, "IMU_GROUND_ROTATION_Z", 0);
    auto imuObservationOrder =
            ini.getVector(section, "IMU_BODIES", vector<string>());

    // driver send rate
    auto rate = ini.getInteger(section, "DRIVER_SEND_RATE", 0);

    // subject data
    auto subjectDir = DATA_DIR + ini.getString(section, "SUBJECT_DIR", "");
    auto modelFile = subjectDir + ini.getString(section, "MODEL_FILE", "");
    auto ngimuDataFile =
            subjectDir + ini.getString(section, "NGIMU_DATA_CSV", "");

    // setup model
    Object::RegisterType(Thelen2003Muscle());
    Model model(modelFile);
    OpenSimUtils::removeActuators(model);

    // imu tasks
    vector<InverseKinematics::IMUTask> imuTasks;
    InverseKinematics::createIMUTasksFromObservationOrder(
            model, imuObservationOrder, imuTasks);

    // ngimu input data driver from file
    //UIMUInputDriver driver(ngimuDataFile, rate);
    //this is now a ROS thing, so we need to set it up as a ros node 
    ros::NodeHandle n;
    UIMUInputDriver driver(8080, rate, true);
    driver.startListening();
    ros::Publisher pub = n.advertise<opensimrt_msgs::Common>("producer",1000);

    // calibrator
    IMUCalibrator clb(model, &driver, imuObservationOrder);
    clb.recordNumOfSamples(10);
    clb.setGroundOrientationSeq(xGroundRotDeg, yGroundRotDeg, zGroundRotDeg);
    clb.computeHeadingRotation(imuBaseBody, imuDirectionAxis);
    clb.calibrateIMUTasks(imuTasks);

    // initialize ik (lower constraint weight and accuracy -> faster tracking)
    InverseKinematics ik(model, {}, imuTasks, SimTK::Infinity, 1e-5);
    auto qLogger = ik.initializeLogger();

    // visualizer
    BasicModelVisualizer visualizer(model);

    SimTK::Vector zv; //zero vector
    std::istringstream is("0.0 0.1 0.2 0.3 0.4 0.5 0.6 0.7 0.8 0.9 0.0 0.1 0.2 0.3 0.4 0.5 0.6 0.7 0.8 ");
    is >> zv;

    cout << "Attempt at showing." << endl;
    visualizer.update(zv);
    cout << "Showing successful." << endl;
    cout << "zero vector: " << zv << endl;

    bool VISUALIZATION = true;

    // mean delay
    int sumDelayMS = 0;
    int numFrames = 0;

    try { // main loop
        while (!(driver.shouldTerminate())) {
            // get input from sensors
	    std::cout << "Getting frame:" << std::endl;
	    auto imuData = driver.getFrame();
	    std::cout << "Solving inverse kinematics:" << std::endl;
            numFrames++;

            if (driver.shouldTerminate())
                    break;

            // solve ik
            chrono::high_resolution_clock::time_point t1;
            t1 = chrono::high_resolution_clock::now();

            auto pose = ik.solve(
                    {imuData.first, {}, clb.transform(imuData.second)});

            chrono::high_resolution_clock::time_point t2;
            t2 = chrono::high_resolution_clock::now();
            sumDelayMS += chrono::duration_cast<chrono::milliseconds>(t2 - t1)
                                  .count();

            // visualize
	    std::cout << "pose is:" << pose.q;
            if(VISUALIZATION)
	    {
		    visualizer.update(pose.q);
	    	    std::cout << "Visualization successful." << std::endl;
	    }
            // record
            qLogger.appendRow(pose.t, ~pose.q);
        }
    } catch (std::exception& e) {
        cout << "Crashed while executing main loop. " << e.what() << endl;

        driver.shouldTerminate(true);
    }

    cout << "Mean delay: " << (double) sumDelayMS / numFrames << " ms" << endl;
    
    CSVFileAdapter::write( qLogger, "test_lower.csv");
    // // store results
    // STOFileAdapter::write(
    //         qLogger, subjectDir + "real_time/inverse_kinematics/q_imu.sto");
}

int main(int argc, char** argv) {
    try {
	ros::init(argc, argv, "online_lower_limb_uimu_ik");
        run();
    } catch (exception& e) {
        cout << "Program crashed while running. Reason: " << e.what() << endl;
        return -1;
    }
    return 0;
}
