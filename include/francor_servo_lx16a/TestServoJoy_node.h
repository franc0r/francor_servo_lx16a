#ifndef TESTSERVOJOY_NODE_H_
#define TESTSERVOJOY_NODE_H_



// #include <ros/ros.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <francor_msgs/msg/servo_lx16a.hpp>

class TestServoJoy_node : public rclcpp::Node
{

public:
  TestServoJoy_node();
  virtual ~TestServoJoy_node();

  /**
     *
     * @brief
     *
     * @return  void
     */
  void init(double duration = 0.02);

private: //functions


  void loop_callback();

  //void subCallback(const ROS_PACK::MESSAGE& msg);
  void subJoy_callback(const sensor_msgs::msg::Joy::SharedPtr msg);

private: //dataelements
  // ros::NodeHandle _nh;

  // ros::Publisher _pubServoSpeed1;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr _pubServoSpeed1;
  // ros::Publisher _pubServoSpeed2;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr _pubServoSpeed2;
  // ros::Publisher _pubServoSpeed3;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr _pubServoSpeed3;

  // ros::Publisher _pubServoPos1;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr _pubServoPos1;
  // ros::Publisher _pubServoPos2;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr _pubServoPos2;
  // ros::Publisher _pubServoPos3;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr _pubServoPos3;

  // ros::Subscriber _subJoy;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr _subJoy;

  double _servo1 = 0.0;
  double _servo2 = 0.0;
  double _servo3 = 0.0;

  // ros::Timer _loopTimer;
  rclcpp::TimerBase::SharedPtr _loopTimer;
};


#endif  //TESTSERVOJOY_NODE_H_
