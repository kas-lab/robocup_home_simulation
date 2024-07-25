# Copyright 2024 KAS-lab
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
import os
from os import environ, pathsep

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    robot_name = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Run with gui (true/false)')

    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    # pkg_world = get_package_share_directory('aws_robomaker_small_house_world')
    # world_path = os.path.join(pkg_world, 'worlds', 'small_house.world')
    pkg_world = get_package_share_directory('plasys_house_world')
    world_path = os.path.join(pkg_world, 'worlds', 'plasys_house', 'plasys_house.world')

    small_house_world_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={
           'gz_args': '-v 4 -r ' + world_path
        }.items(),
    )

    pkg_robocup_simulation = get_package_share_directory('robocup_home_simulation')
    albert_simulation_path = os.path.join(
        pkg_robocup_simulation, 'launch', 'albert_gazebo.launch.py')
    albert_simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(albert_simulation_path),
    )

    return LaunchDescription([
        small_house_world_launch,
        albert_simulation_launch
    ])
