#include "francor_servo_lx16a/TestServoJoy_node.h"



int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TestServoJoy_node>();
  node->init();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}