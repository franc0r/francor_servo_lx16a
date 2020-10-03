#ifndef SERVOLX16A_NODE_H_
#define SERVOLX16A_NODE_H_


#include <iostream>
#include <memory>
#include <vector>
#include <map>

#include <ros/ros.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

#include <sensor_msgs/Joy.h>
#include <std_msgs/Float64.h>

#include <francor_msgs/ServoLx16a.h>

// #include <std_msgs/Bool.h>
//dyn reconfig
// #include <dynamic_reconfigure/server.h>
//#include <ServoLx16a_node/ServoLx16a_nodeConfig.h>


#include "lx16a_servo/src/SerialServoHandler.h"



class ServoSub{
public:
  ServoSub(ros::NodeHandle& nh, francor::servo::Servo_Lx16a& servo) :
    _servo(servo)
  {
    _topic = std::string("servo_lx16a/" + servo.get_param().name + "/pos");
    std::cout << "_topic: " << _topic << std::endl;
    _sub_pos = nh.subscribe( _topic, 1, &ServoSub::subPos_callback, this);
    _sub_speed = nh.subscribe( std::string("servo_lx16a/" + servo.get_param().name + "/speed"), 1, &ServoSub::subSpeed_callback, this);
  }
  ~ServoSub() = default;

  void subPos_callback(const std_msgs::Float64& msg)
  {
    // ROS_INFO_STREAM("ID: " << _servo.get_param().id << ", Name: " << _servo.get_param().name << ", topic: " <<_topic << "   --> Got MSG: " << msg.data);
    if((ros::Time::now() - _t_last_pos).toSec() < 0.04)
    {
      return;
    }
    _t_last_pos = ros::Time::now();
    _servo.set_pos(msg.data);
  }

  void subSpeed_callback(const std_msgs::Float64& msg)
  {
    if((ros::Time::now() - _t_last_spd).toSec() < 0.04)
    {
      return;
    }
    _t_last_spd = ros::Time::now();
    _servo.set_speed(msg.data);
  }

  const francor::servo::Servo_Lx16a& getServo() const
  {
    return _servo;
  }

private:
  ros::Time _t_last_pos;
  ros::Time _t_last_spd;
  francor::servo::Servo_Lx16a& _servo;
  std::string _topic;
  ros::Subscriber _sub_pos;
  ros::Subscriber _sub_speed;
};


class ServoLx16a_node
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
  void start(double duration = 0.01);

private: //functions
  /**
     *
     * @brief this function containts the main working loop
     *
     * @param[in,out]  void
     *
     * @return 		   void
     */
  void run();

  void loop_callback(const ros::TimerEvent& e);

  //void subCallback(const ROS_PACK::MESSAGE& msg);

  // void subJoy_callback(const sensor_msgs::Joy& msg);

  void servo_status_update_callback(const uint8_t id, const francor::servo::Status_Lx16a& status);


  inline francor_msgs::ServoLx16a servo_status_to_ros_status(const uint8_t id, const francor::servo::Status_Lx16a& status)
  {
    francor_msgs::ServoLx16a status_ros;
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
  ros::NodeHandle _nh;

  // ros::Publisher _pub;
  // ros::Subscriber _sub_joy;

  tf2_ros::TransformBroadcaster _tf_broadcaster;

  std::map<uint8_t, ros::Publisher> _pubs_servo_status;

  std::map<uint8_t, std::unique_ptr<ServoSub>> _servo_subs;

  // dynamic_reconfigure::Server<rona_frontier::ExplorationConfig> _drServer;

  std::unique_ptr<francor::servo::SerialServoHandler> _servo_handler;


  ros::Timer _loopTimer;
};


#endif  //SERVOLX16A_NODE_H_
