from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    world_path = PathJoinSubstitution([
                FindPackageShare('robocup_home_simulation'),
                'worlds',
                'robocup_world.sdf'])

    gz_launch_file = os.path.join(get_package_share_directory("mirte_gazebo"),
                                  "launch",
                                  "mirte_simulation_fortress.launch.py")

    gz_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(gz_launch_file),
        launch_arguments={
            'world': world_path,
        }.items(),
    )

    return LaunchDescription(
        [gz_launch]
    )
