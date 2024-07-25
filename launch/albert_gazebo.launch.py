import os
import xacro # type: ignore
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

from launch.event_handlers import OnProcessExit

from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
def generate_launch_description():

    # world = os.path.join(
    #     get_package_share_directory('albert_gazebo'),
    #     'worlds',
    #     'empty_world.world'
    # )
    urdf_prefix = get_package_share_directory("albert_description")
    xacro_file = os.path.join(urdf_prefix, "urdf", "albert_gazebo.xacro")
    urdf_file = os.path.join(get_package_share_directory('albert_description'), 'urdf', 'albert.urdf')
    use_sim_time = LaunchConfiguration("use_sim_time", default=True)
    # Parse and process the XACRO file
    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    robot_description_xml = doc.toxml()
    robot_description = {'robot_description': robot_description_xml}

    # Robot State Publisher
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[ {'use_sim_time': True}, robot_description]
    )

    # Spawn Albert in Gazebo
    ignition_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description',
            '-x', '0',
            '-y', '0',
            '-z', '0.1',
        ],
        output='screen',
    )

    load_joint_state_broadcaster = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "active",
            "joint_state_broadcaster",
        ],
        output="screen",
    )

    load_joint_trajectory_controller = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "active",
            "joint_trajectory_controller",
        ],
        output="screen",
    )
    load_boxer_velocity_controller = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "active",
            "boxer_velocity_controller",
        ],
        output="screen",
    )

    load_boxer_lift_controller = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "active",
            "boxer_lift_controller",
        ],
        output="screen",
    )
    # Launch RViz2 with a specified configuration
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', PathJoinSubstitution([
            FindPackageShare('albert_description'),
            'rviz',
            'albert_navigation.rviz'
        ])]
    )

    # Launch the joint_state_publisher_gui
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        arguments=[urdf_file],
        output=['screen'],
    )

    bridge_params = os.path.join(get_package_share_directory('albert_gazebo'), "config", "bridge.yaml")

    start_gazebo_ros_bridge_cmd = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "--ros-args",
            "-p",
            f"config_file:={bridge_params}",
        ],
        output="screen",
    )

    start_gazebo_ros_image_bridge_cmd = Node(
        package='ros_gz_image',
        executable='image_bridge',
        arguments=['/camera/image_raw'],
        output='screen',
    )

    control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('boxer_control'), 'launch', 'control_launch.py')
        )
    )

    nodes = [
        # Launch gazebo environment
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(
        #         [
        #             os.path.join(
        #                 get_package_share_directory("ros_ign_gazebo"),
        #                 "launch",
        #                 "ign_gazebo.launch.py",
        #             )
        #         ]
        #     ),
        #     launch_arguments=[("gz_args", [f" -r -v 4 {world} "])],
        # ),
        node_robot_state_publisher,
        ignition_spawn_entity,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=ignition_spawn_entity,
                on_exit=[load_joint_state_broadcaster],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_broadcaster,
                on_exit=[load_joint_trajectory_controller],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_trajectory_controller,
                on_exit=[load_boxer_velocity_controller],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_boxer_velocity_controller,
                on_exit=[load_boxer_lift_controller],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_boxer_lift_controller,
                on_exit=[rviz_node],
            )
        ),

        start_gazebo_ros_bridge_cmd,
        start_gazebo_ros_image_bridge_cmd,

        joint_state_publisher_gui,

        # rviz_node,
        DeclareLaunchArgument(
            "use_sim_time", default_value=use_sim_time, description="If true, use simulated clock"
        ),
    ]

    return LaunchDescription(nodes)
