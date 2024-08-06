# ORADAR ROS package

ORADAR ROS - Oradar MS200 for ROS2 Humble

## How to run 

1. Install ROS2 Humble：

   https://docs.ros.org/en/humble/Installation.html

2. Clone the repo
   ```shell
   git@github.com:enricopisanu/oradar_lidar.git
   ```


3. Build

   ```shell
   cd ~/oradar_lidar
   colcon build
   echo -e 'source /opt/ros/humble/setup.bash\nsource ~/oradar_lidar/install/setup.bash' >> ~/.bashrc
   source ~/.bashrc
   ```

4. ROS Parameters

   | Name      | Type | Description                                                         |
   | ----------- | -------- | ------------------------------------------------------------ |
   | frame_id    | string   | Frame related to the lidar, defaulted to laser_frame                       |
   | scan_topic  | string   | Topic to publish the data, laser_frame      |
   | port_name   | string   | Port of the lidar: /dev/ttyUSB0                      |
   | baudrate    | int      | 230400                             |
   | angle_min   | double   | Field of view [0, 360], 0 |
   | angle_max   | double   | Field of view [0, 360], 360 |
   | range_min   | double   | Range: 0.05                               |
   | range_max   | double   | Range: 20                               |
   | clockwise    | bool    |Rotation |
   | motor_speed | int      | Frequency of the lidar 5~15Hz, 10Hz             |

   

5. Launch files

   Use the first file to use launch the lidar, use the second to visualize with RVIZ.

   ```shell
   ros2 launch oradar_lidar ms200_scan.launch.py
   
   ros2 launch oradar_lidar ms200_scan_view.launch.py
   ```

   
