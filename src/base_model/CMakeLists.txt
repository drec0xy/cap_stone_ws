cmake_minimum_required(VERSION 3.5)
project(base_model)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)

# Link dependencies
target_link_libraries(sample_node ${rclcpp_LIBRARIES})

# Install executable
install(
  DIRECTORY src launch rviz config
  DESTINATION share/${PROJECT_NAME}
)
install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)



ament_package()