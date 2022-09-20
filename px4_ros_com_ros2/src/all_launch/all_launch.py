#!/usr/bin/env python
import os.path as osp

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription, logging
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import \
    PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import ThisLaunchFileDir

import numpy as np

def generate_launch_description():

    ## Realsense
    realsense_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([osp.join(
                get_package_share_directory('realsense2_camera'),'launch'),
                '/rs_launch.py']),
            launch_arguments = {
                'config_file': '"/root/px4_ros_com_ros2/src/all_launch/config_d455.yaml"',
                }.items()
            )

    ## static transform between drone1 and realsense camera frame
    static_tf_node = Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=['0','0','0.1', '0', '0','0.0',   '/vicon/drone4', '/camera_link']
            )

    trajectory_follower = Node(
  	package="dasc_robot",
	executable="trajectory_follower"
	)

    gatekeeper_node = Node(
        package="gatekeeper",
        executable="gatekeeper",
        parameters=[
            {"octomap/m_res": 0.1},
            {"octomap/m_zmin": -0.25},
            {"octomap/m_zmax": 3.0}, 
            {"voxel/leaf_size": 0.1},
            {"gatekeeper/safety_radius": 0.2}
        ]
        )

    return LaunchDescription([
        realsense_launch,
        static_tf_node,
	# trajectory_follower,
	gatekeeper_node
        ]
    )
