import os

# from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

  config = os.path.join(
        get_package_share_directory('francor_servo_lx16a'),
        'param',
        'default.yaml'
        )

  servo_lx16a = Node(package='francor_servo_lx16a',
                     namespace='',
                     executable='francor_servo_lx16a_node',
                     name='francor_servo_lx16a_node',
                     output='screen',
                     parameters=[config],
                    #  parameters=[{
                    #    "servo_xml_cfg"  : "/home/m1ch1/workspace/ros2/dev_ws/src/francor_servo_lx16a/config/servo_default.xml",
                    #    "serial_device"  : "/dev/ttyUSB0",
                    #    "rate_pos_req"   : 15.0,
                    #    "rate_speed_req" : 0.1,
                    #    "rate_error_req" : 0.5,
                    #    "rate_temp_req"  : 0.5,
                    #    "rate_vin_req"   : 0.5
                    #  }],
                     remappings=[
                       #pub
                      #  ('/old_topic', '/new_topic'),
                     ]
                     )

  return LaunchDescription([
    servo_lx16a,
  ])