// #include <memory>
#include <vector>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "s_pipe_kinematics/srv/forward_kinematics.hpp"
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

class ForwardKinematicsService : public rclcpp::Node
{
public:
  ForwardKinematicsService()
  : Node("fk_service")
  {
    // Create the forward kinematics service
    service_ = create_service<s_pipe_kinematics::srv::ForwardKinematics>(
      "forward_kinematics", 
      std::bind(&ForwardKinematicsService::handle_service, this, _1, _2));
    
    RCLCPP_INFO(this->get_logger(), "Forward Kinematics Service started");
  }

private:
  void handle_service(
    const std::shared_ptr<s_pipe_kinematics::srv::ForwardKinematics::Request> request,
    std::shared_ptr<s_pipe_kinematics::srv::ForwardKinematics::Response> response)
  {
    RCLCPP_INFO(this->get_logger(), "Received FK request");
    
    // Extract joint angles from request
    std::vector<double> joint_angles_deg(6);
    for (size_t i = 0; i < 6; ++i) {
      joint_angles_deg[i] = request->joint_angles[i];
    }
    
    try {
      // Initialize forward kinematics object
      s_pipe::ForwardKinematics fk;
      
      // Compute forward kinematics
      Eigen::Matrix4d ee_pose = fk.computeDeg(joint_angles_deg);
      
      // Extract position from transformation matrix
      Eigen::Vector3d position = ee_pose.block<3, 1>(0, 3);
      
      // Extract rotation matrix
      Eigen::Matrix3d rotation = ee_pose.block<3, 3>(0, 0);
      
      // Get Euler angles for debugging
      std::array<double, 3> euler = getEulerAngles(rotation);
      
      // Convert rotation matrix to quaternion
      Eigen::Quaterniond quaternion(rotation);
      
      // Set position in response
      response->position[0] = position.x();
      response->position[1] = position.y();
      response->position[2] = position.z();
      
      // Set quaternion in response (in x, y, z, w order)
      response->orientation[0] = quaternion.x();
      response->orientation[1] = quaternion.y();
      response->orientation[2] = quaternion.z();
      response->orientation[3] = quaternion.w();
      
      RCLCPP_INFO(this->get_logger(), 
                 "FK computed successfully. Position: [%.2f, %.2f, %.2f]",
                 position.x(), position.y(), position.z());
      
      // Log Euler angles for debugging and validation
      RCLCPP_DEBUG(this->get_logger(), 
                 "Orientation (Euler RPY): [%.2f, %.2f, %.2f]",
                 euler[0], euler[1], euler[2]);

        RCLCPP_DEBUG(this->get_logger(), 
            "Quaternion values [x,y,z,w]: [%.8f, %.8f, %.8f, %.8f]",
            quaternion.x(), quaternion.y(), quaternion.z(), quaternion.w());
    }
    catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Error computing forward kinematics: %s", e.what());
      
      // Still need to set a response
      for (int i = 0; i < 3; ++i) {
        response->position[i] = 0.0;
      }
      
      for (int i = 0; i < 4; ++i) {
        response->orientation[i] = 0.0;
      }
      response->orientation[3] = 1.0;  // Identity quaternion
    }
  }

  rclcpp::Service<s_pipe_kinematics::srv::ForwardKinematics>::SharedPtr service_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ForwardKinematicsService>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}