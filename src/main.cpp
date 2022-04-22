#include "francor_servo_lx16a/ServoLx16a_node.h"



int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ServoLx16a_node>();
  node->init();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}