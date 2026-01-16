#pragma once
#include <vector>

namespace arm_platform {

/**
 * Single joint state.
 * Fields are optional because not all motors support all feedback.
 */
struct JointStateArray {
    std::vector<double> position;
    std::vector<double> velocity;
    std::vector<double> current;
    std::vector<double> voltage;
    std::vector<double> temperature;
};

/**
 * Abstract joint command (arm level).
 * Controllers decide how to use it.
 */
struct JointCommand {
    std::vector<double> position;
    std::vector<double> velocity;
    std::vector<double> current;
    std::vector<double> p;  // Proportional gain
    std::vector<double> d;  // Derivative gain
};

}
