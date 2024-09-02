#!/usr/bin/python3

import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
   package_name = "kelo_tulip"
   example_launch = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory(package_name), 'launch'),
         '/example.launch.py'])
      )

   return LaunchDescription([
      example_launch,
      Node(
         package='joy_linux',
         executable='joy_linux_node',
         name='joy_linux_node',
         output='screen'
      ),
   ])
