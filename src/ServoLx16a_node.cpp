
#include "francor_servo_lx16a/ServoLx16a_node.h"
#include <chrono>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>

ServoLx16a_node::ServoLx16a_node() : rclcpp::Node("francor_servo_lx16a_node"),
  _tf_broadcaster(this)
{
  //rosParam
  // ros::NodeHandle privNh("~");
  std::string servo_xml_cfg;
  std::string serial_device;
  double      rate_pos_req;
  double      rate_speed_req;
  double      rate_error_req;
  double      rate_temp_req;
  double      rate_vin_req;
  // int int_val;
  // bool bool_val;


  // privNh.param(         "servo_xml_cfg" ,     servo_xml_cfg,   std::string("invalid"));
  this->declare_parameter<std::string>( "servo_xml_cfg", std::string("invalid"));
  servo_xml_cfg = this->get_parameter("servo_xml_cfg").as_string();
  // privNh.param(         "serial_device" ,     serial_device,   std::string("/dev/ttyUSB0"));
  this->declare_parameter<std::string>( "serial_device", std::string("/dev/ttyUSB0"));
  serial_device = this->get_parameter("serial_device").as_string();
  // privNh.param<double>( "rate_pos_req" ,      rate_pos_req,     15.0);
  this->declare_parameter<double>( "rate_pos_req", 15.0);
  rate_pos_req = this->get_parameter("rate_pos_req").as_double();
  // privNh.param<double>( "rate_speed_req" ,    rate_speed_req,   0.1);
  this->declare_parameter<double>( "rate_speed_req", 0.1);
  rate_speed_req = this->get_parameter("rate_speed_req").as_double();
  // privNh.param<double>( "rate_error_req" ,    rate_error_req,   0.5);
  this->declare_parameter<double>( "rate_error_req", 0.5);
  rate_error_req = this->get_parameter("rate_error_req").as_double();
  // privNh.param<double>( "rate_temp_req" ,     rate_temp_req,    0.5);
  this->declare_parameter<double>( "rate_temp_req", 0.5);
  rate_temp_req = this->get_parameter("rate_temp_req").as_double();
  // privNh.param<double>( "rate_vin_req" ,      rate_vin_req,     0.5);
  this->declare_parameter<double>( "rate_vin_req", 0.5);
  rate_vin_req = this->get_parameter("rate_vin_req").as_double();
  
  
  // privNh.param<int>(    "int_val"    ,    int_val   ,   1.0);
  // privNh.param<bool>(   "bool_val"   ,    bool_val  ,   true);
  //init publisher
  // _pub = _nh.advertise<std_msgs::Bool>("pub_name", 1);

  _servo_handler = std::make_unique<francor::servo::SerialServoHandler>(serial_device, servo_xml_cfg);
  _servo_handler->set_rate_pos_request(rate_pos_req);
  _servo_handler->set_rate_speed_request(rate_speed_req);
  _servo_handler->set_rate_error_request(rate_error_req);
  _servo_handler->set_rate_temp_request(rate_temp_req);
  _servo_handler->set_rate_vin_request(rate_vin_req);


}

ServoLx16a_node::~ServoLx16a_node()
{ }

void ServoLx16a_node::init(double duration)
{
  //create timer
  // _loopTimer = _nh.createTimer(ros::Duration(duration), &ServoLx16a_node::loop_callback, this);

  _loopTimer = this->create_wall_timer(std::chrono::duration<double>(duration), std::bind(&ServoLx16a_node::loop_callback, this));
  // this->run();

    //init servo
  if(!_servo_handler->init())
  {
    // ROS_ERROR("Error at init servo handler... will exit..");
    RCLCPP_ERROR(this->get_logger(), "Error at init servo handler... will exit..");
    ::exit(EXIT_FAILURE);
  }
  _servo_handler->attach_servo_status_update_callback(std::bind(&ServoLx16a_node::servo_status_update_callback, this, std::placeholders::_1, std::placeholders::_2));

  for(auto& e : _servo_handler->get_servos())
  {
    //attach pubs for servos
    //  _pubs_servo_status.insert(std::make_pair(e.first, _nh.advertise<francor_msgs::ServoLx16a>("servo_lx16a/" + e.second.get_param().name + "/status", 1)));
    _pubs_servo_status.insert(std::make_pair(e.first, this->create_publisher<francor_msgs::msg::ServoLx16a>("servo_lx16a/" + e.second.get_param().name + "/status", 10)));
    //create servo subs objs
    _servo_subs.insert(std::make_pair(e.first, std::make_unique<ServoSub>(this, e.second)));
  }
  std::cout << "#######################################" << std::endl;
  for(auto& e : _servo_subs)
  {
    std::cout << e.second->getServo().get_param() << std::endl;
  }  
  std::cout << "#######################################" << std::endl;
}

void ServoLx16a_node::loop_callback()
{
  _servo_handler->spin_once();
}


void ServoLx16a_node::servo_status_update_callback(const uint8_t id, const francor::servo::Status_Lx16a& status) 
{
  //pub status
  _pubs_servo_status.at(id)->publish(this->servo_status_to_ros_status(id, status));

  //pub tf
  geometry_msgs::msg::TransformStamped tf;
  tf.header.stamp = this->get_clock()->now();
  tf.header.frame_id = _servo_handler->get_servo(id).get_param().base_frame;
  tf.child_frame_id = _servo_handler->get_servo(id).get_param().name;
  tf.transform.translation.x = 0.0;
  tf.transform.translation.y = 0.0;
  tf.transform.translation.z = 0.0;
  tf2::Quaternion q;
  q.setRPY(0, status.pos, 0);
  tf.transform.rotation.x = q.x();
  tf.transform.rotation.y = q.y();
  tf.transform.rotation.z = q.z();
  tf.transform.rotation.w = q.w();
  
  _tf_broadcaster.sendTransform(tf);
}

