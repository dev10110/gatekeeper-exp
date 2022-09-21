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

    ## gatekeeper's octomapper
    mapper_node = Node(
        package="gatekeeper",
        executable="mapper",
        parameters=[
           {"map/m_res": 0.05},
           {"map/m_zmin": -20.0},
           {"map/m_zmax": 20.0},
           {"map/leaf_size": 0.025},
           {"map/keep_every_n": 1},
           {"map/free_x_min": -2.0},
           {"map/free_y_min": -1.0},
           {"map/free_z_min": 0.0},
           {"map/free_x_max": 0.0},
           {"map/free_y_max": 1.0},
           {"map/free_z_max": 2.0},
           {"map/publish_radius": 1.5},
           {"map/publish_occupied_every_ms": 1000},
           #{"map/publish_unsafe_every_ms": 250},
        ],
        #prefix = ['gdb -ex run  run --args']
        )

    return LaunchDescription([
	mapper_node
        ]
    )

