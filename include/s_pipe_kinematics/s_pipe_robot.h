#ifndef S_PIPE_ROBOT_H
#define S_PIPE_ROBOT_H

#include <array>
#include <vector>
#include <cmath>
#include <Eigen/Dense>

namespace s_pipe {

// DH Parameters structure
struct DHParam {
    double theta; // in radians
    double d;     // in mm
    double a;     // in mm
    double alpha; // in radians
};

// S-pipe robot DH parameters (values provided)
// static constexpr double d1 = 145.3;
// static constexpr double d2 = 117.15;
// static constexpr double d3 = 110.7;
// static constexpr double d4 = 344.0;
// static constexpr double d5 = 94.6;
// static constexpr double d6 = 94.6;
// static constexpr double d7 = 100.0;
// static constexpr double a1 = 286.0;

// S-pipe robot DH parameters (values provided)
static constexpr double d1 = 147.07;
static constexpr double d2 = 117.15;
static constexpr double d3 = 110.64;
static constexpr double d4 = 344.0;
static constexpr double d5 = 94.6;
static constexpr double d6 = 94.6;
static constexpr double d7 = 100.0;
static constexpr double a1 = 286.0;
static constexpr double a9 = 0.04;  // Added X offset

// Joint limits structure
struct JointLimits {
    double min;
    double max;
};

// Corrected joint limits
static const std::array<JointLimits, 6> DEFAULT_JOINT_LIMITS = {{
    {-2*M_PI, 2*M_PI},  // Joint 1 (±360°)
    {-2*M_PI, 2*M_PI},  // Joint 2 (±360°)
    {-2.618, 2.618},    // Joint 3 (±150°) - CRITICAL CORRECTION
    {-2*M_PI, 2*M_PI},  // Joint 4 (±360°)
    {-2*M_PI, 2*M_PI},  // Joint 5 (±360°)
    {-2*M_PI, 2*M_PI}   // Joint 6 (±360°)
}};

// Utility functions
inline double deg2rad(double deg) {
    return deg * M_PI / 180.0;
}

inline double rad2deg(double rad) {
    return rad * 180.0 / M_PI;
}

// Get default DH parameters for the S-pipe robot
// inline std::vector<DHParam> getDefaultDHParams() {
//     return {
//         {0.0, d1, 0.0, deg2rad(-90.0)},    // Link 1
//         {-deg2rad(90.0), -d2, 0.0, 0.0},   // Link 2
//         {0.0, 0.0, a1, 0.0},               // Link 3
//         {deg2rad(90.0), d3, 0.0, 0.0},     // Link 4
//         {0.0, 0.0, 0.0, deg2rad(90.0)},    // Link 5
//         {0.0, d4, 0.0, deg2rad(-90.0)},    // Link 6
//         {0.0, -d5, 0.0, 0.0},              // Link 7
//         {0.0, d6, 0.0, deg2rad(90.0)},     // Link 8
//         {0.0, d7, 0.0, 0.0}                // Link 9
//     };
// }

// Get default DH parameters for the S-pipe robot
inline std::vector<DHParam> getDefaultDHParams() {
    return {
        {0.0, d1, 0.0, deg2rad(-90.0)},                // Link 1
        {-deg2rad(90.0), -d2, 0.0, 0.0},               // Link 2
        {0.0, 0.0, a1, 0.0},                           // Link 3
        {deg2rad(90.0), d3, 0.0, 0.0},                 // Link 4
        {0.0, 0.0, 0.0, deg2rad(90.0)},                // Link 5
        {0.0, d4, 0.0, deg2rad(-90.0)},                // Link 6
        {0.0, -d5, 0.0, 0.0},                          // Link 7
        {0.0, d6, 0.0, deg2rad(90.0)},                 // Link 8
        {0.0, d7, a9, 0.0}                             // Link 9 with X offset
    };
}

// Get standard DH parameter offsets for joint angles
// These represent the deviations from standard DH parameters that are applied to joint angles
inline std::array<double, 6> getJointOffsets() {
    return {
        0.0,              // Joint 1: No offset
        -deg2rad(90.0),   // Joint 2: -90° offset
        deg2rad(90.0),    // Joint 3: +90° offset
        0.0,              // Joint 4: No offset
        0.0,              // Joint 5: No offset
        0.0               // Joint 6: No offset
    };
}

// Get indices of DH parameters that correspond to each joint
inline std::array<int, 6> getJointIndices() {
    return {0, 1, 3, 5, 7, 8};
}

} // namespace s_pipe

#endif // S_PIPE_ROBOT_H