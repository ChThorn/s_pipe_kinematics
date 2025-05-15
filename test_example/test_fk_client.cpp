#include <memory>
#include <chrono>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "s_pipe_kinematics/srv/forward_kinematics.hpp"

using namespace std::chrono_literals;

class FKTestClient : public rclcpp::Node
{
public:
  FKTestClient()
  : Node("fk_test_client")
  {
    client_ = create_client<s_pipe_kinematics::srv::ForwardKinematics>("forward_kinematics");
  }

  bool send_request(const std::vector<double>& joint_angles_deg)
  {
    // Wait for service to be available
    while (!client_->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for service.");
        return false;
      }
      RCLCPP_INFO(this->get_logger(), "Waiting for forward kinematics service...");
    }

    // Create request
    auto request = std::make_shared<s_pipe_kinematics::srv::ForwardKinematics::Request>();
    
    // Fill joint angles
    for (size_t i = 0; i < 6; ++i) {
      request->joint_angles[i] = joint_angles_deg[i];
    }

    RCLCPP_INFO(this->get_logger(), "Sending FK request with joint angles: [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]",
               joint_angles_deg[0], joint_angles_deg[1], joint_angles_deg[2], 
               joint_angles_deg[3], joint_angles_deg[4], joint_angles_deg[5]);

    // Send request
    auto future = client_->async_send_request(request);
    
    // Wait for result
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) ==
      rclcpp::FutureReturnCode::SUCCESS)
    {
      auto result = future.get();
      RCLCPP_INFO(this->get_logger(), "FK result received:");
      RCLCPP_INFO(this->get_logger(), "Position: [%.2f, %.2f, %.2f]",
                 result->position[0], result->position[1], result->position[2]);
      RCLCPP_INFO(this->get_logger(), "Orientation (quaternion): [%.2f, %.2f, %.2f, %.2f]",
                 result->orientation[0], result->orientation[1], 
                 result->orientation[2], result->orientation[3]);
      return true;
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to call service forward_kinematics");
      return false;
    }
  }

private:
  rclcpp::Client<s_pipe_kinematics::srv::ForwardKinematics>::SharedPtr client_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  
  auto node = std::make_shared<FKTestClient>();
  
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
    node->send_request(config);
    // Small delay between tests
    std::this_thread::sleep_for(500ms);
  }
  
  rclcpp::shutdown();
  return 0;
}