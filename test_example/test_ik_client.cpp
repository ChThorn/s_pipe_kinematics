#include <memory>
#include <chrono>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "s_pipe_kinematics/srv/forward_kinematics.hpp"
#include "s_pipe_kinematics/srv/inverse_kinematics.hpp"

using namespace std::chrono_literals;

class IKTestClient : public rclcpp::Node
{
public:
  IKTestClient()
  : Node("ik_test_client")
  {
    fk_client_ = create_client<s_pipe_kinematics::srv::ForwardKinematics>("forward_kinematics");
    ik_client_ = create_client<s_pipe_kinematics::srv::InverseKinematics>("inverse_kinematics");
  }

  bool test_ik_with_joint_angles(const std::vector<double>& joint_angles_deg)
  {
    RCLCPP_INFO(this->get_logger(), "Testing IK with joint angles: [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]",
               joint_angles_deg[0], joint_angles_deg[1], joint_angles_deg[2], 
               joint_angles_deg[3], joint_angles_deg[4], joint_angles_deg[5]);

    // First get FK result to create a valid pose
    auto fk_result = call_fk_service(joint_angles_deg);
    if (!fk_result) {
      RCLCPP_ERROR(this->get_logger(), "Failed to get FK result");
      return false;
    }

    // Use FK result to call IK
    auto ik_result = call_ik_service(
      fk_result->position, 
      fk_result->orientation, 
      {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}  // Start with all zeros as initial guess for a challenge
    );

    if (!ik_result) {
      RCLCPP_ERROR(this->get_logger(), "Failed to get IK result");
      return false;
    }

    // Print original vs IK solution
    RCLCPP_INFO(this->get_logger(), "Original joint angles: [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]",
               joint_angles_deg[0], joint_angles_deg[1], joint_angles_deg[2], 
               joint_angles_deg[3], joint_angles_deg[4], joint_angles_deg[5]);
    
    RCLCPP_INFO(this->get_logger(), "IK solution: [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]",
               ik_result->joint_angles[0], ik_result->joint_angles[1], ik_result->joint_angles[2], 
               ik_result->joint_angles[3], ik_result->joint_angles[4], ik_result->joint_angles[5]);
    
    RCLCPP_INFO(this->get_logger(), "IK success: %s", ik_result->success ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "IK message: %s", ik_result->message.c_str());

    return ik_result->success;
  }

private:
  std::shared_ptr<s_pipe_kinematics::srv::ForwardKinematics::Response> 
  call_fk_service(const std::vector<double>& joint_angles_deg)
  {
    // Wait for service to be available
    while (!fk_client_->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for FK service.");
        return nullptr;
      }
      RCLCPP_INFO(this->get_logger(), "Waiting for forward kinematics service...");
    }

    // Create request
    auto request = std::make_shared<s_pipe_kinematics::srv::ForwardKinematics::Request>();
    
    // Fill joint angles
    for (size_t i = 0; i < 6; ++i) {
      request->joint_angles[i] = joint_angles_deg[i];
    }

    // Send request
    auto future = fk_client_->async_send_request(request);
    
    // Wait for result
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) ==
      rclcpp::FutureReturnCode::SUCCESS)
    {
      return future.get();
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to call service forward_kinematics");
      return nullptr;
    }
  }

  std::shared_ptr<s_pipe_kinematics::srv::InverseKinematics::Response> 
  call_ik_service(const std::array<double, 3>& position,
                 const std::array<double, 4>& orientation,
                 const std::vector<double>& initial_guess_deg)
  {
    // Wait for service to be available
    while (!ik_client_->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for IK service.");
        return nullptr;
      }
      RCLCPP_INFO(this->get_logger(), "Waiting for inverse kinematics service...");
    }

    // Create request
    auto request = std::make_shared<s_pipe_kinematics::srv::InverseKinematics::Request>();
    
    // Fill position
    for (size_t i = 0; i < 3; ++i) {
      request->position[i] = position[i];
    }
    
    // Fill orientation
    for (size_t i = 0; i < 4; ++i) {
      request->orientation[i] = orientation[i];
    }
    
    // Fill initial guess
    for (size_t i = 0; i < 6; ++i) {
      request->initial_guess[i] = initial_guess_deg[i];
    }

    RCLCPP_INFO(this->get_logger(), "Sending IK request for position [%.2f, %.2f, %.2f]",
               position[0], position[1], position[2]);

    // Send request
    auto future = ik_client_->async_send_request(request);
    
    // Wait for result
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) ==
      rclcpp::FutureReturnCode::SUCCESS)
    {
      return future.get();
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to call service inverse_kinematics");
      return nullptr;
    }
  }

  rclcpp::Client<s_pipe_kinematics::srv::ForwardKinematics>::SharedPtr fk_client_;
  rclcpp::Client<s_pipe_kinematics::srv::InverseKinematics>::SharedPtr ik_client_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  
  auto node = std::make_shared<IKTestClient>();
  
  // Test configurations from your original code
  std::vector<std::vector<double>> test_configs = {
    {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},                      // Standard Configuration
    {0.0, 90.0, 0.0, 0.0, 0.0, 0.0},                     // Extended Arm
    {45.0, 30.0, -60.0, 120.0, 30.0, -90.0},             // Complex Twist
    {175.0, -175.0, 175.0, -175.0, 175.0, -175.0},       // Extreme Angles
    {0.0, 0.0, -90.0, 0.0, 0.0, 0.0}                     // Near-Singular Configuration
  };
  
  // Run all test configurations
  for (const auto& config : test_configs) {
    node->test_ik_with_joint_angles(config);
    // Small delay between tests
    std::this_thread::sleep_for(1s);
  }
  
  rclcpp::shutdown();
  return 0;
}