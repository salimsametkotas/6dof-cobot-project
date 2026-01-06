cmake_minimum_required(VERSION 3.8)
project(cobot_kinematics_plugin)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED) 
find_package(pluginlib REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_kdl REQUIRED)

include_directories(include)

add_library(${PROJECT_NAME} src/cobot_kinematics_plugin.cpp)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  moveit_core
  moveit_ros_planning
  pluginlib
  tf2
  tf2_kdl
)

install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Headerları Yükle
install(
  DIRECTORY include/
  DESTINATION include
)

install(
  FILES cobot_kinematics_plugin_description.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  moveit_core
  moveit_ros_planning
  pluginlib
  rclcpp
)

ament_package()
