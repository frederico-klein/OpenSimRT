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
 * @file GRFMPredictionInsole.h
 *
 * @brief Estimate the ground reaction forces, moments and cop during gait from
 * kinematic data.
 *
 * @Author: Filip Konstantinos <filip.k@ece.upatras.gr>
 */
#pragma once

#include "Exception.h"
#include "InverseDynamics.h"
#include "SlidingWindow.h"
#include "internal/RealTimeExports.h"
#include <OpenSim/Simulation/Model/Model.h>
#include <SimTKcommon.h>
#include "GRFMNonSmooth.h"

namespace OpenSimRT {

class RealTime_API GaitPhaseDetector;

/*******************************************************************************/


/**
 * Ground Reaction Force & Moment Prediction method. It uses an external
 * detection algorithm for determining gait related events (heel-strike,
 * toe-off, etc.). It estimates the total GRF&M from kinematic data during gait
 * using either the Newton-Euler equations or by solving the ID. Seperation into
 * right/left components is achieved using the Smooth Transition Assumption
 * funtions [Ren et al. https://doi.org/10.1016/j.jbiomech.2008.06.001]. The CoP
 * is estimated assuming a smooth transition between two specified station
 * points in heel and toe.
 */
class RealTime_API GRFMPredictionInsole : public GRFMNonSmooth
{
    /**
     * Funtion type used to describe the Smooth Transition Assumption fucntions.
     */
    typedef std::function<double(const double&)> TransitionFuction;

    /**
     * Function type used to describe the CoP trajectory on foot during gait.
     */
    typedef std::function<SimTK::Vec3(const double&, const SimTK::Vec3&)>
            CoPTrajectory;

 public:

    struct Parameters {
        int directionWindowSize;
        Method method;                                  // Newton-Euler or ID
       	std::string insoleDataFromFile; 
	//TODO: Remove Unused params!!
	std::string pelvisBodyName;                     // pelvis body name
        std::string rStationBodyName, lStationBodyName; // foot body names
        SimTK::Vec3 rHeelStationLocation, lHeelStationLocation; // begin of cop
        SimTK::Vec3 rToeStationLocation, lToeStationLocation;   // end of cop
    };

    GRFMPredictionInsole(const OpenSim::Model&, const Parameters&,
                   GaitPhaseDetector*); // ctor

    /**
     * Compute the ground reaction forces, moments and cop.
     */
    Output solve(const Input& input);

 private:
    // transition functions based on the STA
    TransitionFuction reactionComponentTransition; // STA fucntions
    // TransitionFuction anteriorForceTransition; // STA function for F_x

    // function that describes the CoP trajectory during gait
    CoPTrajectory copPosition;

    SimTK::Vec3 totalForceAtThs;  // F_ext at Ths
    SimTK::Vec3 totalMomentAtThs; // M_ext at Ths
    double Tds, Tss; //  double support and single support time period

    // gait direction based on the average direction of the pelvis anterior axis
    SlidingWindow<SimTK::Vec3> gaitDirectionBuffer;

    Parameters parameters;

    // gait phase detection
    SimTK::ReferencePtr<GaitPhaseDetector> gaitPhaseDetector;
    GaitPhaseState::GaitPhase gaitphase; // gait phase during simulation

    // station points forming the cop trajectory
    SimTK::ReferencePtr<OpenSim::Station> heelStationR;
    SimTK::ReferencePtr<OpenSim::Station> heelStationL;
    SimTK::ReferencePtr<OpenSim::Station> toeStationR;
    SimTK::ReferencePtr<OpenSim::Station> toeStationL;

    /**
     * Compute the rotation matrix required to transform the estimated total
     * GRF&M from the global reference frame to the the average heading
     * direction frame during gait computed based on the anterior axis of the
     * pelvis local frame.
     */
    SimTK::Rotation computeGaitDirectionRotation(const std::string& bodyName);

    /**
     * Separate the total reaction components into R/L foot reaction components.
     */
    void separateReactionComponents(
            const double& time, const SimTK::Vec3& totalReactionComponent,
            const SimTK::Vec3& totalReactionAtThs,
            const TransitionFuction& anteriorComponentFunction,
            const TransitionFuction& verticalComponentFunction,
            const TransitionFuction& lateralComponentFunction,
            SimTK::Vec3& rightReactionComponent,
            SimTK::Vec3& leftReactionComponent);

    /**
     * Compute the CoP on each foot.
     */
    void computeReactionPoint(const double& t, SimTK::Vec3& rightPoint,
                              SimTK::Vec3& leftPoint);

    void getInsoleMeasurements();
};

} // namespace OpenSimRT
