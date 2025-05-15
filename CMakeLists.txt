cmake_minimum_required(VERSION 3.10)
project(s_pipe_kinematics)



# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find required packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(Eigen3 REQUIRED)

# Print system info for debugging
message(STATUS "CMAKE_SYSTEM_NAME: ${CMAKE_SYSTEM_NAME}")
message(STATUS "CMAKE_CXX_COMPILER_ID: ${CMAKE_CXX_COMPILER_ID}")
message(STATUS "CMAKE_BUILD_TYPE: ${CMAKE_BUILD_TYPE}")
message(STATUS "Eigen3 include dir: ${EIGEN3_INCLUDE_DIR}")
message(STATUS "Eigen3 version: ${EIGEN3_VERSION_STRING}")

# Define service interface files
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/ForwardKinematics.srv"
  "srv/InverseKinematics.srv"
  DEPENDENCIES geometry_msgs std_msgs
)

# Set include directories for all targets
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIR}
)

# Get the typesupport target
rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")

# Forward Kinematics Service Node - directly include all source files
add_executable(fk_service_node 
  src/fk_service_node.cpp
  src/forward_kinematics.cpp
  src/inverse_kinematics.cpp
)

target_link_libraries(fk_service_node
  Eigen3::Eigen
  ${cpp_typesupport_target}
)

ament_target_dependencies(fk_service_node
  rclcpp
  std_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_eigen
)

# Inverse Kinematics Service Node - directly include all source files
add_executable(ik_service_node 
  src/ik_service_node.cpp
  src/forward_kinematics.cpp
  src/inverse_kinematics.cpp
)

target_link_libraries(ik_service_node
  Eigen3::Eigen
  ${cpp_typesupport_target}
)

ament_target_dependencies(ik_service_node
  rclcpp
  std_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_eigen
)

# Test clients (in test_example directory)
add_executable(test_fk_client test_example/test_fk_client.cpp)
target_link_libraries(test_fk_client 
  ${cpp_typesupport_target}
)
ament_target_dependencies(test_fk_client
  rclcpp
  std_msgs
  geometry_msgs
)

add_executable(test_ik_client test_example/test_ik_client.cpp)
target_link_libraries(test_ik_client 
  ${cpp_typesupport_target}
)
ament_target_dependencies(test_ik_client
  rclcpp
  std_msgs
  geometry_msgs
)

# Test kinematics implementation directly
add_executable(test_kinematics 
  test_example/test_kinematics.cpp
  src/forward_kinematics.cpp
  src/inverse_kinematics.cpp
)

target_link_libraries(test_kinematics
  Eigen3::Eigen
)

# Install targets
install(TARGETS
  fk_service_node
  ik_service_node
  test_fk_client
  test_ik_client
  test_kinematics
  DESTINATION lib/${PROJECT_NAME}
)

# Install includes
install(DIRECTORY include/
  DESTINATION include
)

# Install launch files
install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()