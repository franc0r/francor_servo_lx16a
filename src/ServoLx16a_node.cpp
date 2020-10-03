
#include "ServoLx16a_node.h"

ServoLx16a_node::ServoLx16a_node()
{
  //rosParam
  ros::NodeHandle privNh("~");
  std::string servo_xml_cfg;
  std::string serial_device;
  double      rate_pos_req;
  double      rate_speed_req;
  double      rate_error_req;
  double      rate_temp_req;
  double      rate_vin_req;
  // int int_val;
  // bool bool_val;


  privNh.param(         "servo_xml_cfg" ,     servo_xml_cfg,   std::string("invalid"));
  privNh.param(         "serial_device" ,     serial_device,   std::string("/dev/ttyUSB0"));
  privNh.param<double>( "rate_pos_req" ,      rate_pos_req,     15.0);
  privNh.param<double>( "rate_speed_req" ,    rate_speed_req,   0.1);
  privNh.param<double>( "rate_error_req" ,    rate_error_req,   0.5);
  privNh.param<double>( "rate_temp_req" ,     rate_temp_req,    0.5);
  privNh.param<double>( "rate_vin_req" ,      rate_vin_req,     0.5);
  
  
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


  //inti subscriber
  // _sub_joy = _nh.subscribe("joy", 1, &ServoLx16a_node::subJoy_callback, this);


}

ServoLx16a_node::~ServoLx16a_node()
{ }

void ServoLx16a_node::start(double duration)
{
  //create timer
  _loopTimer = _nh.createTimer(ros::Duration(duration), &ServoLx16a_node::loop_callback, this);
  this->run();
}

void ServoLx16a_node::run()
{
  //init servo
  if(!_servo_handler->init())
  {
    ROS_ERROR("Error at init servo handler... will exit..");
    ::exit(EXIT_FAILURE);
  }
  _servo_handler->attach_servo_status_update_callback(std::bind(&ServoLx16a_node::servo_status_update_callback, this, std::placeholders::_1, std::placeholders::_2));

  for(auto& e : _servo_handler->get_servos())
  {
    //attach pubs for servos
     _pubs_servo_status.insert(std::make_pair(e.first, _nh.advertise<francor_msgs::ServoLx16a>("servo_lx16a/" + e.second.get_param().name + "/status", 1)));
    //create servo subs objs
    _servo_subs.insert(std::make_pair(e.first, std::make_unique<ServoSub>(_nh, e.second)));
  }
  std::cout << "#######################################" << std::endl;
  for(auto& e : _servo_subs)
  {
    std::cout << e.second->getServo().get_param() << std::endl;
  }  
  std::cout << "#######################################" << std::endl;

  ros::spin();
}

void ServoLx16a_node::loop_callback(const ros::TimerEvent& e)
{
  _servo_handler->spin_once();
}


void ServoLx16a_node::servo_status_update_callback(const uint8_t id, const francor::servo::Status_Lx16a& status) 
{
  //pub status
  _pubs_servo_status.at(id).publish(this->servo_status_to_ros_status(id, status));

  //pub tf
  geometry_msgs::TransformStamped tf;
  tf.header.stamp = ros::Time::now();
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

// ------------- main ---------------
int main(int argc, char *argv[])
{
  ros::init(argc, argv, "ServoLx16a_node_node");
  ros::NodeHandle nh("~");

  ServoLx16a_node node;
  node.start();
}
