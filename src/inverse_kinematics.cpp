#include "s_pipe_kinematics/inverse_kinematics.h"
#include <iostream>

namespace s_pipe {

InverseKinematics::InverseKinematics(
    const std::vector<DHParam>& dh_params,
    const std::array<JointLimits, 6>& joint_limits)
    : fk_(dh_params), dh_params_(dh_params), joint_limits_(joint_limits) {
    std::cout << "InverseKinematics constructor called" << std::endl;
}

Eigen::Matrix3d InverseKinematics::getRotationMatrix(const Eigen::Matrix4d& transform) {
    return transform.block<3, 3>(0, 0);
}

Eigen::Vector3d InverseKinematics::getPosition(const Eigen::Matrix4d& transform) {
    return transform.block<3, 1>(0, 3);
}

Eigen::Vector3d InverseKinematics::rotationMatrixToAxisAngle(const Eigen::Matrix3d& rotation_matrix) {
    Eigen::AngleAxisd angleAxis(rotation_matrix);
    return angleAxis.axis() * angleAxis.angle();
}

Eigen::MatrixXd InverseKinematics::computeAnalyticalJacobian(const std::vector<double>& joint_angles) {
    Eigen::MatrixXd jacobian = Eigen::MatrixXd::Zero(6, 6);
    
    // Compute transformation matrices to each joint
    std::vector<Eigen::Matrix4d> transforms = fk_.computeJointTransforms(joint_angles);
    
    // Get the indices of the joints in the DH parameters
    std::array<int, 6> joint_indices = getJointIndices();
    
    // End effector position
    Eigen::Vector3d endEffectorPosition = getPosition(transforms.back());
    
    // Calculate Jacobian for each joint
    for (int i = 0; i < 6; ++i) {
        int idx = joint_indices[i];
        
        // Get z-axis of the joint's coordinate frame (rotation axis)
        Eigen::Vector3d z_axis = transforms[idx].block<3, 1>(0, 2);
        
        // Position of the joint
        Eigen::Vector3d jointPosition = getPosition(transforms[idx]);
        
        // Linear velocity component (cross product of rotation axis and displacement)
        Eigen::Vector3d linear = z_axis.cross(endEffectorPosition - jointPosition);
        
        // Angular velocity component (rotation axis)
        Eigen::Vector3d angular = z_axis;
        
        // Set Jacobian columns
        jacobian.block<3, 1>(0, i) = linear;
        jacobian.block<3, 1>(3, i) = angular;
    }
    
    return jacobian;
}

bool InverseKinematics::checkJointLimits(const std::vector<double>& joint_angles) {
    if (joint_angles.size() != 6) {
        return false;
    }
    
    for (int i = 0; i < 6; ++i) {
        if (joint_angles[i] < joint_limits_[i].min || joint_angles[i] > joint_limits_[i].max) {
            return false;
        }
    }
    
    return true;
}

void InverseKinematics::applyJointLimits(std::vector<double>& joint_angles) {
    for (int i = 0; i < 6; ++i) {
        if (joint_angles[i] < joint_limits_[i].min) {
            joint_angles[i] = joint_limits_[i].min;
        } else if (joint_angles[i] > joint_limits_[i].max) {
            joint_angles[i] = joint_limits_[i].max;
        }
    }
}

std::vector<double> InverseKinematics::compute(
    const Eigen::Matrix4d& target_pose,
    const std::vector<double>& initial_guess,
    double lambda,
    double epsilon,
    int max_iterations) {
    
    if (initial_guess.size() != 6) {
        throw std::invalid_argument("Initial guess vector must have 6 elements");
    }
    
    // Initialize with the provided initial guess
    std::vector<double> current_joint_angles = initial_guess;
    
    // Target position and orientation
    Eigen::Vector3d target_position = getPosition(target_pose);
    Eigen::Matrix3d target_rotation = getRotationMatrix(target_pose);
    
    // Track best solution and its error
    double best_error = std::numeric_limits<double>::max();
    std::vector<double> best_solution = current_joint_angles;
    
    // Main iteration loop
    for (int iter = 0; iter < max_iterations; ++iter) {
        // Current end effector pose
        Eigen::Matrix4d current_pose = fk_.compute(current_joint_angles);
        Eigen::Vector3d current_position = getPosition(current_pose);
        Eigen::Matrix3d current_rotation = getRotationMatrix(current_pose);
        
        // Position error
        Eigen::Vector3d position_error = target_position - current_position;
        
        // Orientation error (using axis-angle representation)
        Eigen::Matrix3d rotation_error = target_rotation * current_rotation.transpose();
        Eigen::Vector3d orientation_error = rotationMatrixToAxisAngle(rotation_error);
        
        // Combine errors with weighting (position in mm, orientation in rad)
        double pos_weight = 1.0;
        double rot_weight = 1.0;
        
        Eigen::VectorXd error(6);
        error.head(3) = pos_weight * position_error;
        error.tail(3) = rot_weight * orientation_error;
        
        double current_error = error.norm();
        
        // Save best solution so far
        if (current_error < best_error) {
            best_error = current_error;
            best_solution = current_joint_angles;
        }
        
        // Check for convergence
        if (current_error < epsilon) {
            //std::cout << "Converged after " << iter << " iterations (error: " << current_error << ")" << std::endl;
            return current_joint_angles;
        }
        
        // Compute Jacobian with analytical method for higher precision
        Eigen::MatrixXd jacobian = computeAnalyticalJacobian(current_joint_angles);
        
        // Compute joint angle update using Damped Least Squares
        Eigen::MatrixXd JJT = jacobian * jacobian.transpose();
        Eigen::VectorXd delta_theta = jacobian.transpose() * 
                                      (JJT + lambda * lambda * Eigen::MatrixXd::Identity(6, 6)).inverse() * 
                                      error;
        
        // Adaptive step size that decreases as we get closer to solution
        double step_size = 0.9 / (1.0 + 0.1 * iter);
        if (current_error < 1.0) step_size = 0.1;
        if (current_error < 0.1) step_size = 0.05;
        
        // Apply update with controlled step size
        for (int j = 0; j < 6; ++j) {
            current_joint_angles[j] += step_size * delta_theta(j);
            
            // Normalize angles to [-π, π]
            while (current_joint_angles[j] > M_PI) current_joint_angles[j] -= 2 * M_PI;
            while (current_joint_angles[j] < -M_PI) current_joint_angles[j] += 2 * M_PI;
        }
        
        // Apply joint limits
        applyJointLimits(current_joint_angles);
        
        // Print progress for every 100 iterations (for long runs)
        if (iter % 100 == 0 && iter > 0) {
            //std::cout << "Iteration " << iter << ", Error: " << current_error << std::endl;
        }
    }
    
    //std::cout << "Maximum iterations reached, returning best solution with error: " << best_error << std::endl;
    return best_solution; // Return the best solution found
}

std::vector<double> InverseKinematics::computeDeg(
    const Eigen::Matrix4d& target_pose,
    const std::vector<double>& initial_guess_deg,
    double lambda,
    double epsilon,
    int max_iterations) {
    
    std::cout << "InverseKinematics::computeDeg called" << std::endl;
    
    // Convert initial guess from degrees to radians
    std::vector<double> initial_guess_rad(initial_guess_deg.size());
    for (size_t i = 0; i < initial_guess_deg.size(); ++i) {
        initial_guess_rad[i] = deg2rad(initial_guess_deg[i]);
    }
    
    // Compute inverse kinematics in radians
    std::vector<double> solution_rad = compute(target_pose, initial_guess_rad, lambda, epsilon, max_iterations);
    
    // Convert solution back to degrees
    std::vector<double> solution_deg(solution_rad.size());
    for (size_t i = 0; i < solution_rad.size(); ++i) {
        solution_deg[i] = rad2deg(solution_rad[i]);
    }
    
    return solution_deg;
}

} // namespace s_pipe