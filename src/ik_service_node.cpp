#include <memory>
#include <vector>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "s_pipe_kinematics/srv/inverse_kinematics.hpp"
#include "s_pipe_kinematics/inverse_kinematics.h"
#include "s_pipe_kinematics/forward_kinematics.h"
#include "s_pipe_kinematics/s_pipe_robot.h"

// Updated to use the newer hpp header per the warning
#include <tf2_eigen/tf2_eigen.hpp>

using namespace std::placeholders;

// Helper function to extract Euler angles from rotation matrix for debugging
std::array<double, 3> getEulerAngles(const Eigen::Matrix3d& rotation) {
    // Extract rotation around Z axis for small angles
    double rz = -atan2(rotation(0, 1), rotation(0, 0));
    
    // If angles are very small, return them directly
    if (fabs(rotation(0, 1)) < 0.01 && fabs(rotation(1, 0)) < 0.01) {
        return {0.0, 0.0, s_pipe::rad2deg(rz)};
    }
    
    // Otherwise use full Euler extraction for complex orientations
    Eigen::Vector3d euler = rotation.eulerAngles(2, 1, 0);
    
    // Convert to degrees and normalize
    std::array<double, 3> result;
    result[0] = s_pipe::rad2deg(euler[2]); // Roll (X)
    result[1] = s_pipe::rad2deg(euler[1]); // Pitch (Y)
    result[2] = s_pipe::rad2deg(euler[0]); // Yaw (Z)
    
    // Normalize to manufacturer's convention
    for (int i = 0; i < 3; i++) {
        while (result[i] > 180.0) result[i] -= 360.0;
        while (result[i] < -180.0) result[i] += 360.0;
    }
    
    return result;
}

class InverseKinematicsService : public rclcpp::Node
{
public:
  InverseKinematicsService()
  : Node("ik_service")
  {
    // Declare and get parameters
    this->declare_parameter("lambda", 0.1);
    this->declare_parameter("epsilon", 0.000001);
    this->declare_parameter("max_iterations", 1000);
    
    lambda_ = this->get_parameter("lambda").as_double();
    epsilon_ = this->get_parameter("epsilon").as_double();
    max_iterations_ = this->get_parameter("max_iterations").as_int();
    
    // Create the inverse kinematics service
    service_ = create_service<s_pipe_kinematics::srv::InverseKinematics>(
      "inverse_kinematics", 
      std::bind(&InverseKinematicsService::handle_service, this, _1, _2));
    
    RCLCPP_INFO(this->get_logger(), "Inverse Kinematics Service started");
    RCLCPP_INFO(this->get_logger(), "Parameters: lambda=%.3f, epsilon=%.6f, max_iterations=%d",
               lambda_, epsilon_, max_iterations_);
  }

private:
  void handle_service(
    const std::shared_ptr<s_pipe_kinematics::srv::InverseKinematics::Request> request,
    std::shared_ptr<s_pipe_kinematics::srv::InverseKinematics::Response> response)
  {
    RCLCPP_INFO(this->get_logger(), "Received IK request");
    
    try {
      // Initialize inverse kinematics object
      s_pipe::InverseKinematics ik;
      
      // Extract position from request
      Eigen::Vector3d position;
      position.x() = request->position[0];
      position.y() = request->position[1];
      position.z() = request->position[2];
      
      // Extract quaternion from request
      Eigen::Quaterniond quaternion;
      quaternion.x() = request->orientation[0];
      quaternion.y() = request->orientation[1];
      quaternion.z() = request->orientation[2];
      quaternion.w() = request->orientation[3];
      
      // Normalize quaternion in case it's not normalized
      quaternion.normalize();
      
      // Create target transformation matrix
      Eigen::Matrix4d target_pose = Eigen::Matrix4d::Identity();
      target_pose.block<3, 3>(0, 0) = quaternion.toRotationMatrix();
      target_pose.block<3, 1>(0, 3) = position;
      
      // Get Euler angles for debugging
      Eigen::Matrix3d rotation = target_pose.block<3, 3>(0, 0);
      std::array<double, 3> euler = getEulerAngles(rotation);
      
      // Log target pose details
      RCLCPP_DEBUG(this->get_logger(), 
                 "Target Position: [%.2f, %.2f, %.2f]",
                 position.x(), position.y(), position.z());
      RCLCPP_DEBUG(this->get_logger(), 
                 "Target Orientation (Euler RPY): [%.2f, %.2f, %.2f]",
                 euler[0], euler[1], euler[2]);
                
        // Log quaternion values for debugging
        RCLCPP_DEBUG(this->get_logger(), 
            "Target Quaternion [x,y,z,w]: [%.8f, %.8f, %.8f, %.8f]",
            quaternion.x(), quaternion.y(), quaternion.z(), quaternion.w());
      
      // Extract initial guess
      std::vector<double> initial_guess_deg(6, 0.0);
      for (size_t i = 0; i < 6; ++i) {
        initial_guess_deg[i] = request->initial_guess[i];
      }
      
      // Compute inverse kinematics
      std::vector<double> joint_angles_deg = ik.computeDeg(
        target_pose, initial_guess_deg, lambda_, epsilon_, max_iterations_);
      
      // Check if solution is valid
      s_pipe::ForwardKinematics fk;
      Eigen::Matrix4d result_pose = fk.computeDeg(joint_angles_deg);
      
      // Compute position error
      Eigen::Vector3d result_position = result_pose.block<3, 1>(0, 3);
      double position_error = (position - result_position).norm();
      
      // Compute orientation error
      Eigen::Matrix3d result_rotation = result_pose.block<3, 3>(0, 0);
      Eigen::Matrix3d target_rotation = target_pose.block<3, 3>(0, 0);
      Eigen::Matrix3d rotation_error = target_rotation * result_rotation.transpose();
      Eigen::AngleAxisd angle_axis(rotation_error);
      double orientation_error = angle_axis.angle();
      
      // Get result Euler angles for debugging
      std::array<double, 3> result_euler = getEulerAngles(result_rotation);
      
      // Set joint angles in response
      for (size_t i = 0; i < 6; ++i) {
        response->joint_angles[i] = joint_angles_deg[i];
      }
      
      // Check if error is within acceptable tolerance
      const double position_tolerance = 1.0;    // 1 mm
      const double orientation_tolerance = 0.01; // ~0.5 degrees
      
      if (position_error <= position_tolerance && orientation_error <= orientation_tolerance) {
        response->success = true;
        response->message = "IK solution found successfully.";
        
        RCLCPP_INFO(this->get_logger(), 
                   "IK computed successfully. Position error: %.3f mm, Orientation error: %.3f rad",
                   position_error, orientation_error);
        
        // Log result pose details
        RCLCPP_DEBUG(this->get_logger(), 
                   "Result Position: [%.2f, %.2f, %.2f]",
                   result_position.x(), result_position.y(), result_position.z());
        RCLCPP_DEBUG(this->get_logger(), 
                   "Result Orientation (Euler RPY): [%.2f, %.2f, %.2f]",
                   result_euler[0], result_euler[1], result_euler[2]);
      } else {
        response->success = false;
        response->message = "IK solution found but exceeds error tolerance. "
                          "Position error: " + std::to_string(position_error) + " mm, "
                          "Orientation error: " + std::to_string(s_pipe::rad2deg(orientation_error)) + " degrees";
        
        RCLCPP_WARN(this->get_logger(), 
                   "IK solution exceeds tolerance. Position error: %.3f mm, Orientation error: %.3f deg",
                   position_error, s_pipe::rad2deg(orientation_error));
      }
    }
    catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Error computing inverse kinematics: %s", e.what());
      
      // Set failure response
      for (int i = 0; i < 6; ++i) {
        response->joint_angles[i] = 0.0;
      }
      response->success = false;
      response->message = std::string("Error computing inverse kinematics: ") + e.what();
    }
  }

  rclcpp::Service<s_pipe_kinematics::srv::InverseKinematics>::SharedPtr service_;
  double lambda_;
  double epsilon_;
  int max_iterations_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<InverseKinematicsService>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}