#ifndef SERVOLX16A_NODE_H_
#define SERVOLX16A_NODE_H_


#include <francor_msgs/msg/detail/servo_lx16a__struct.hpp>
#include <iostream>
#include <memory>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/time.hpp>
#include <vector>
#include <map>

// #include <ros/ros.h>
#include <rclcpp/rclcpp.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

// #include <sensor_msgs/Joy.h>
#include <std_msgs/msg/float64.hpp>

#include <francor_msgs/msg/servo_lx16a.hpp>

// #include <std_msgs/Bool.h>
//dyn reconfig
// #include <dynamic_reconfigure/server.h>
//#include <ServoLx16a_node/ServoLx16a_nodeConfig.h>


#include "francor_servo_lx16a/lx16a_servo/src/SerialServoHandler.h"



class ServoSub{
public:
  ServoSub(rclcpp::Node* parent, francor::servo::Servo_Lx16a& servo) :
    _parent(parent),
    _servo(servo)
  {
    _topic = std::string("servo_lx16a/" + servo.get_param().name + "/pos");
    std::cout << "_topic: " << _topic << std::endl;
    // _sub_pos = nh.subscribe( _topic, 1, &ServoSub::subPos_callback, this);
    _sub_pos = _parent->create_subscription<std_msgs::msg::Float64>(_topic, 10, std::bind(&ServoSub::subPos_callback, this, std::placeholders::_1));
    // _sub_speed = nh.subscribe( std::string("servo_lx16a/" + servo.get_param().name + "/speed"), 1, &ServoSub::subSpeed_callback, this);
    _sub_speed = _parent->create_subscription<std_msgs::msg::Float64>("servo_lx16a/" + servo.get_param().name + "/speed", 10, std::bind(&ServoSub::subSpeed_callback, this, std::placeholders::_1));
  }
  ~ServoSub() = default;

  void subPos_callback(const std_msgs::msg::Float64::SharedPtr msg)
  {
    // ROS_INFO_STREAM("ID: " << _servo.get_param().id << ", Name: " << _servo.get_param().name << ", topic: " <<_topic << "   --> Got MSG: " << msg.data);
    auto now = _parent->get_clock()->now();
    if((now - _t_last_pos).seconds() < 0.04)
    {
      return;
    }
    _t_last_pos = now;
    _servo.set_pos(msg->data);
  }

  void subSpeed_callback(const std_msgs::msg::Float64::SharedPtr msg)
  {
    auto now = _parent->get_clock()->now();
    if((now - _t_last_spd).seconds() < 0.04)
    {
      return;
    }
    _t_last_spd = now;
    _servo.set_speed(msg->data);
  }

  const francor::servo::Servo_Lx16a& getServo() const
  {
    return _servo;
  }

private:
  rclcpp::Node*_parent;
  // ros::Time _t_last_pos;
  rclcpp::Time _t_last_pos;
  // ros::Time _t_last_spd;
  rclcpp::Time _t_last_spd;
  francor::servo::Servo_Lx16a& _servo;
  std::string _topic;
  // ros::Subscriber _sub_pos;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr _sub_pos;
  // ros::Subscriber _sub_speed;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr _sub_speed;
};


class ServoLx16a_node : public rclcpp::Node
{

public:
  ServoLx16a_node();
  virtual ~ServoLx16a_node();

  /**
     *
     * @brief
     *
     * @return  void
     */
  void init(double duration = 0.01);

private: //functions

  void loop_callback();

  //void subCallback(const ROS_PACK::MESSAGE& msg);

  // void subJoy_callback(const sensor_msgs::Joy& msg);

  void servo_status_update_callback(const uint8_t id, const francor::servo::Status_Lx16a& status);


  inline francor_msgs::msg::ServoLx16a servo_status_to_ros_status(const uint8_t id, const francor::servo::Status_Lx16a& status)
  {
    francor_msgs::msg::ServoLx16a status_ros;
    status_ros.id    = id;
    status_ros.pos   = status.pos.radian();
    status_ros.speed = status.speed;
    status_ros.err   = status.error_state;
    status_ros.temp  = status.temp;
    status_ros.vin   = status.v_in;

    return status_ros;
  }

  //void dynreconfig_callback(ServoLx16a_node::ServoLx16a_nodeConfig &config, uint32_t level);
private: //dataelements
  // ros::Publisher _pub;
  // ros::Subscriber _sub_joy;

  tf2_ros::TransformBroadcaster _tf_broadcaster;

  std::map<uint8_t, rclcpp::Publisher<francor_msgs::msg::ServoLx16a>::SharedPtr> _pubs_servo_status;

  std::map<uint8_t, std::unique_ptr<ServoSub>> _servo_subs;

  // dynamic_reconfigure::Server<rona_frontier::ExplorationConfig> _drServer;

  std::unique_ptr<francor::servo::SerialServoHandler> _servo_handler;


  // ros::Timer _loopTimer;
  rclcpp::TimerBase::SharedPtr _loopTimer;
};


#endif  //SERVOLX16A_NODE_H_
