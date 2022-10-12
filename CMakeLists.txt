cmake_minimum_required(VERSION 3.5)
project(cslam_experiments)

install(DIRECTORY 
   config
   data
   launch
   DESTINATION share/${PROJECT_NAME}/
)

find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)

find_package(cslam REQUIRED)
find_package(cslam_common_interfaces REQUIRED)
find_package(cslam_loop_detection_interfaces REQUIRED)

find_package(launch_testing_ament_cmake REQUIRED)

find_package(ament_cmake_python REQUIRED)

ament_package()
