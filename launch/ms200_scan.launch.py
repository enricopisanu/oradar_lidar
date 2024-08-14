from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

import ament_index_python
import os

RECORD_BAG = True

'''
parameters=[
        {'device_model': 'MS200'},
        {'frame_id': 'laser_frame'},
        {'scan_topic': 'MS200/scan'},
        {'port_name': '/dev/ttyAMA1 '},
        {'baudrate': 230400},
        {'angle_min': 0.0},
        {'angle_max': 360.0},
        {'range_min': 0.05},
        {'range_max': 20.0},
        {'clockwise': False},
        {'motor_speed': 10}
      ]
'''
def get_package_path(package_name: str) -> str:
  package_path = ament_index_python.get_package_prefix(package_name)
  return os.path.abspath(os.path.join(package_path, '..', '..'))


def generate_launch_description():
  # LiDAR publisher node
  ordlidar_node = Node(
      package='oradar_lidar',
      executable='oradar_scan',
      name='MS200',
      output='screen',
      parameters=[
        {'device_model': 'MS200'},
        {'frame_id': 'laser_frame'},
        {'scan_topic': '/scan'},
        {'port_name': '/dev/ttyAMA1 '},
        {'baudrate': 230400},
        {'angle_min': 0.0},
        {'angle_max': 360.0},
        {'range_min': 0.05},
        {'range_max': 20.0},
        {'clockwise': False},
        {'motor_speed': 10}
      ]
  )

  # base_link to laser_frame tf node
  base_link_to_laser_tf_node = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    name='base_link_to_base_laser',
    arguments=['0','0','0.18','0','0','0','base_link','laser_frame']
  )

  ld = LaunchDescription()
  
  ld.add_action(DeclareLaunchArgument('bagfile', default_value='my_bag_file',
                                        description='Name of the bag file to record'))
  
  bag_file_path = PathJoinSubstitution([
    get_package_path('oradar_lidar'),
    'bags', 
    LaunchConfiguration('bagfile') 
  ])

  record_bag = ExecuteProcess(
    cmd=[
          'ros2', 'bag', 'record', '-a',
          '-o', bag_file_path
        ],
    output='screen',
    log_cmd=True
  )



  ld.add_action(ordlidar_node)
  ld.add_action(base_link_to_laser_tf_node)
  ld.add_action(record_bag)

  return ld

