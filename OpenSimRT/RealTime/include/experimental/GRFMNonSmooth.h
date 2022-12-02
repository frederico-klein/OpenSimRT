#pragma once

//check if I need these headers.
#include "Exception.h"
#include "InverseDynamics.h"
#include "SlidingWindow.h"
#include "internal/RealTimeExports.h"
#include <OpenSim/Simulation/Model/Model.h>
#include <SimTKcommon.h>

namespace OpenSimRT {

// Gait related states.
class RealTime_API GaitPhaseState {
 public:
    enum class LegPhase { INVALID, SWING, STANCE };

    enum class GaitPhase { INVALID, RIGHT_SWING, LEFT_SWING, DOUBLE_SUPPORT };

    enum class LeadingLeg { INVALID, RIGHT, LEFT };
};

class RealTime_API GRFMNonSmooth {

 public:
    /**
     * Select method for computing the total reactions loads.
     */
    enum class Method { NewtonEuler, InverseDynamics };

    struct Input {
        double t;
        SimTK::Vector q;
        SimTK::Vector qDot;
        SimTK::Vector qDDot;
    };

    struct RealTime_API Output {
        double t;
        ExternalWrench::Input right, left;
    };

    GRFMNonSmooth(const OpenSim::Model&, std::string pelvisBodyName); // ctor

    std::string methodName, pelvisBodyName;
    Method method;
    /**
     * Select the name of the method used to compute the total reaction loads
     * F_ext and M_ext. Computation is performed using either the Newton-Euler
     * equations of motion, or the by solving the Inverse-Dynamics (ID) method.
     */
    static Method selectMethod(const std::string& methodName);

    /**
     * Compute the ground reaction forces, moments and cop.
     */
    Output solve(const Input& input);

    void computeTotalReactionComponents(const Input& input,
                                        SimTK::Vec3& totalReactionForce,
                                        SimTK::Vec3& totalReactionMoment);
// private:

    OpenSim::Model model;
    SimTK::State state;


};

} // namespace OpenSimRT
