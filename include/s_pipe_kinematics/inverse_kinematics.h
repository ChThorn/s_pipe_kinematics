#ifndef INVERSE_KINEMATICS_H
#define INVERSE_KINEMATICS_H

#include "s_pipe_robot.h"
#include "forward_kinematics.h"
#include <vector>
#include <Eigen/Dense>

namespace s_pipe {

class InverseKinematics {
public:
    // Constructor
    InverseKinematics(
        const std::vector<DHParam>& dh_params = getDefaultDHParams(),
        const std::array<JointLimits, 6>& joint_limits = DEFAULT_JOINT_LIMITS);

    // Numerical inverse kinematics using Damped Least Squares
    std::vector<double> compute(
        const Eigen::Matrix4d& target_pose,
        const std::vector<double>& initial_guess,
        double lambda = 0.1,
        double epsilon = 0.000001,
        int max_iterations = 1000);

    // Compute inverse kinematics with target pose in degrees
    std::vector<double> computeDeg(
        const Eigen::Matrix4d& target_pose,
        const std::vector<double>& initial_guess_deg,
        double lambda = 0.1,
        double epsilon = 0.000001,
        int max_iterations = 1000);

    // Get the forward kinematics object
    ForwardKinematics& getForwardKinematics() { return fk_; }

private:
    // Forward kinematics object for computations
    ForwardKinematics fk_;

    // DH parameters for the robot
    std::vector<DHParam> dh_params_;

    // Joint limits
    std::array<JointLimits, 6> joint_limits_;

    // Compute analytical Jacobian
    Eigen::MatrixXd computeAnalyticalJacobian(const std::vector<double>& joint_angles);

    // Extract rotation matrix from transformation matrix
    Eigen::Matrix3d getRotationMatrix(const Eigen::Matrix4d& transform);

    // Extract position vector from transformation matrix
    Eigen::Vector3d getPosition(const Eigen::Matrix4d& transform);

    // Convert rotation matrix to axis-angle representation
    Eigen::Vector3d rotationMatrixToAxisAngle(const Eigen::Matrix3d& rotation_matrix);

    // Check if joint angles are within limits
    bool checkJointLimits(const std::vector<double>& joint_angles);

    // Apply joint limits to ensure joint angles are within bounds
    void applyJointLimits(std::vector<double>& joint_angles);
};

} // namespace s_pipe

#endif // INVERSE_KINEMATICS_H