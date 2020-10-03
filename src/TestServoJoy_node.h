#ifndef TESTSERVOJOY_NODE_H_
#define TESTSERVOJOY_NODE_H_



#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/Joy.h>
#include <francor_msgs/ServoLx16a.h>

class TestServoJoy_node
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
  void start(double duration = 0.02);

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
  void subJoy_callback(const sensor_msgs::Joy& msg);

private: //dataelements
  ros::NodeHandle _nh;

  ros::Publisher _pubServoSpeed1;
  ros::Publisher _pubServoSpeed2;
  ros::Publisher _pubServoSpeed3;

  ros::Publisher _pubServoPos1;
  ros::Publisher _pubServoPos2;
  ros::Publisher _pubServoPos3;

  ros::Subscriber _subJoy;

  double _servo1 = 0.0;
  double _servo2 = 0.0;
  double _servo3 = 0.0;

  ros::Timer _loopTimer;
};


#endif  //TESTSERVOJOY_NODE_H_
