#include <iostream>
#include <vector>
#include "s_pipe_kinematics/s_pipe_robot.h"
#include "s_pipe_kinematics/forward_kinematics.h"
#include "s_pipe_kinematics/inverse_kinematics.h"

int main() {
    // Test forward kinematics
    try {
        std::cout << "Creating ForwardKinematics object..." << std::endl;
        s_pipe::ForwardKinematics fk;
        
        std::cout << "Testing computeDeg method..." << std::endl;
        std::vector<double> test_angles = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        Eigen::Matrix4d result = fk.computeDeg(test_angles);
        
        std::cout << "Forward kinematics result:" << std::endl;
        std::cout << result << std::endl;
        
        // Test inverse kinematics
        std::cout << "Creating InverseKinematics object..." << std::endl;
        s_pipe::InverseKinematics ik;
        
        std::cout << "Testing computeDeg method..." << std::endl;
        std::vector<double> ik_result = ik.computeDeg(result, test_angles);
        
        std::cout << "Inverse kinematics result:" << std::endl;
        for (double angle : ik_result) {
            std::cout << angle << " ";
        }
        std::cout << std::endl;
        
        std::cout << "All tests passed!" << std::endl;
        return 0;
    }
    catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
}