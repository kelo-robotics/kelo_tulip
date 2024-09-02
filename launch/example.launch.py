#!/usr/bin/python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
   package_name = "kelo_tulip"
   robot_name = os.environ.get('ROBOT_NAME', 'example')   
   config_path = os.path.join(get_package_share_directory(package_name), "config", robot_name + ".yaml")

   return LaunchDescription([
      Node(
         package='kelo_tulip',
         executable='platform_driver',
         name='platform_driver',
         output='screen',
         parameters=[config_path]
      ),
   ])
