
#include "francor_servo_lx16a/TestServoJoy_node.h"
#include <std_msgs/msg/detail/float64__struct.hpp>

TestServoJoy_node::TestServoJoy_node() : rclcpp::Node("francor_servo_lx16a_joy_test_node")
{
  //rosParam
  // ros::NodeHandle privNh("~");
  // std::string string_val;
  // double double_val;
  // int int_val;
  // bool bool_val;


  // privNh.param(         "string_val" ,    string_val,   std::string("string"));
  // privNh.param<double>( "double_val" ,    double_val,   100.0);
  // privNh.param<int>(    "int_val"    ,    int_val   ,   1.0);
  // privNh.param<bool>(   "bool_val"   ,    bool_val  ,   true);
  //init publisher
  //_pub = _nh.advertise<std_msgs::Bool>("pub_name", 1);
  // _pubServoPos1 = _nh.advertise<std_msgs::Float64>("servo_lx16a/pan/pos", 1);
  // _pubServoPos2 = _nh.advertise<std_msgs::Float64>("servo_lx16a/tilt/pos", 1);


  // _pubServoSpeed1 = _nh.advertise<std_msgs::Float64>("servo_lx16a/arm_pan/speed", 1);
  _pubServoSpeed1 = this->create_publisher<std_msgs::msg::Float64>("servo_lx16a/arm_pan/speed", 10);
  // _pubServoSpeed2 = _nh.advertise<std_msgs::Float64>("servo_lx16a/arm_tilt/speed", 1);
  _pubServoSpeed2 = this->create_publisher<std_msgs::msg::Float64>("servo_lx16a/arm_tilt/speed", 10);
  // _pubServoSpeed3 = _nh.advertise<std_msgs::Float64>("servo_lx16a/arm_roll/speed", 1);
  _pubServoSpeed3 = this->create_publisher<std_msgs::msg::Float64>("servo_lx16a/arm_roll/speed", 10);

  // _pubServoPos1 = _nh.advertise<std_msgs::Float64>("servo_lx16a/arm_pan/pos", 1);
  _pubServoPos1 = this->create_publisher<std_msgs::msg::Float64>("servo_lx16a/arm_pan/pos", 10);
  // _pubServoPos2 = _nh.advertise<std_msgs::Float64>("servo_lx16a/arm_tilt/pos", 1);
  _pubServoPos2 = this->create_publisher<std_msgs::msg::Float64>("servo_lx16a/arm_tilt/pos", 10);
  // _pubServoPos3 = _nh.advertise<std_msgs::Float64>("servo_lx16a/arm_roll/pos", 1);
  _pubServoPos3 = this->create_publisher<std_msgs::msg::Float64>("servo_lx16a/arm_roll/pos", 10);

  //inti subscriber
  // _subJoy = _nh.subscribe("/joy", 1, &TestServoJoy_node::subJoy_callback, this);
  _subJoy = this->create_subscription<sensor_msgs::msg::Joy>("joy", 10, std::bind(&TestServoJoy_node::subJoy_callback, this, std::placeholders::_1));

} 

TestServoJoy_node::~TestServoJoy_node()
{
}

void TestServoJoy_node::init(double duration)
{
  //create timer
  // _loopTimer = _nh.createTimer(ros::Duration(duration), &TestServoJoy_node::loop_callback, this);
  _loopTimer = this->create_wall_timer(std::chrono::duration<double>(duration), std::bind(&TestServoJoy_node::loop_callback, this));
}


void TestServoJoy_node::loop_callback()
{
  //do loop stuff here!!!
  std_msgs::msg::Float64 msg;
  msg.data = _servo1;// * 2.0944;
  _pubServoSpeed1->publish(msg);
  msg.data = _servo2;// * 2.0944;
  _pubServoSpeed2->publish(msg);
  msg.data = _servo3;
  _pubServoSpeed3->publish(msg);

}

void TestServoJoy_node::subJoy_callback(const sensor_msgs::msg::Joy::SharedPtr msg) 
{
  if(msg->buttons[0])
  {
    std_msgs::msg::Float64 msg;
    msg.data = 0.0;
    _pubServoPos1->publish(msg);
    _pubServoPos2->publish(msg);
    _pubServoPos3->publish(msg);
  }
  //servo 1 : axis 0
  //servo 2 : axis 3
  _servo1 = msg->axes[3];
  _servo2 = msg->axes[4];
  _servo3 = msg->axes[0];

}

