cmake_minimum_required(VERSION 3.8)
project(gazebo_ros_motors)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(gazebo_plugins REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(angles REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(urdf REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(gazebo_ros_dc_motor SHARED src/gazebo_ros_dc_motor.cpp include/gazebo_ros_motors/backlash.h src/backlash.cpp)
target_compile_features(gazebo_ros_dc_motor PUBLIC cxx_std_17)
target_include_directories(gazebo_ros_dc_motor PRIVATE include)
ament_target_dependencies(gazebo_ros_dc_motor gazebo_dev gazebo_ros rclcpp std_msgs geometry_msgs nav_msgs sensor_msgs)

add_library(gazebo_ros_bldc_motor SHARED src/gazebo_ros_bldc_motor.cpp src/bldcmotor_foc.cpp)
target_compile_features(gazebo_ros_bldc_motor PUBLIC cxx_std_17)
target_include_directories(gazebo_ros_bldc_motor PRIVATE include)
target_link_libraries(gazebo_ros_bldc_motor)
ament_target_dependencies(gazebo_ros_bldc_motor gazebo_dev gazebo_ros rclcpp std_msgs geometry_msgs nav_msgs sensor_msgs)

add_library(gazebo_ros_link_torque SHARED src/gazebo_ros_link_torque.cpp)
target_compile_features(gazebo_ros_link_torque PUBLIC cxx_std_17)
target_include_directories(gazebo_ros_link_torque PRIVATE include)
ament_target_dependencies(gazebo_ros_link_torque gazebo_dev gazebo_ros rclcpp std_msgs geometry_msgs nav_msgs sensor_msgs)


install(TARGETS
        gazebo_ros_dc_motor
        gazebo_ros_bldc_motor
        gazebo_ros_link_torque
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
        )

install(
        DIRECTORY params xacro
        DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
