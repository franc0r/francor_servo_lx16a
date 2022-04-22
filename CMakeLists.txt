cmake_minimum_required(VERSION 3.8)
project(francor_servo_lx16a)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(francor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

set(dependencies rclcpp
                 std_msgs
                 sensor_msgs
                 geometry_msgs
                 std_srvs
                 francor_msgs
                 tf2
                 tf2_ros
                 )

include_directories(include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(francor_servo_lx16a_node src/main.cpp
                                        src/ServoLx16a_node.cpp
                                        )
ament_target_dependencies(francor_servo_lx16a_node ${dependencies} )
target_link_libraries(francor_servo_lx16a_node tinyxml2)

add_executable(francor_servo_lx16a_joy_test_node src/test_servo_main.cpp
                                                 src/TestServoJoy_node.cpp)
ament_target_dependencies(francor_servo_lx16a_joy_test_node ${dependencies})

install(TARGETS francor_servo_lx16a_node francor_servo_lx16a_joy_test_node
  DESTINATION lib/${PROJECT_NAME})

# install(DIRECTORY images
#   DESTINATION share/${PROJECT_NAME}
#   FILES_MATCHING PATTERN "*.png" PATTERN "*.svg")

# install(DIRECTORY launch
#   DESTINATION share/${PROJECT_NAME})


ament_package()
