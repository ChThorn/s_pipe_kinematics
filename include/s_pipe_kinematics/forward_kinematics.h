#ifndef FORWARD_KINEMATICS_H
#define FORWARD_KINEMATICS_H

#include "s_pipe_robot.h"
#include <vector>
#include <Eigen/Dense>

namespace s_pipe {

class ForwardKinematics {
public:
    // Constructor
    ForwardKinematics(const std::vector<DHParam>& dh_params = getDefaultDHParams());

    // Calculate forward kinematics for given joint angles (in radians)
    Eigen::Matrix4d compute(const std::vector<double>& joint_angles);

    // Calculate forward kinematics for given joint angles (in degrees)
    Eigen::Matrix4d computeDeg(const std::vector<double>& joint_angles_deg);

    // Calculate transformation matrices for each joint
    std::vector<Eigen::Matrix4d> computeJointTransforms(const std::vector<double>& joint_angles);

private:
    // DH parameters for the robot
    std::vector<DHParam> dh_params_;

    // Joint indices in DH parameters
    std::array<int, 6> joint_indices_;

    // Joint angle offsets
    std::array<double, 6> joint_offsets_;

    // Calculate transformation matrix from DH parameters
    Eigen::Matrix4d calculateTransformMatrix(const DHParam& dh);
};

} // namespace s_pipe

#endif // FORWARD_KINEMATICS_H