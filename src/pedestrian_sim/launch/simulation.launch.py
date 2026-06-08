"""
simulation.launch.py

Full teaching environment in one command:
  Gazebo (simple 10×10 m room) + TurtleBot4 robot + simulated pedestrians.

Usage
-----
    ros2 launch pedestrian_sim simulation.launch.py

Optional arguments
------------------
    model           standard | lite     TB4 model (default: standard)
    num_people      int                 Pedestrians (default: 2)
    ped_speed       float               Walking speed m/s (default: 0.5)
    boundary_radius float               Room boundary m (default: 3.5)

Architecture note
-----------------
This file intentionally bypasses turtlebot4_gz.launch.py and calls
gz_sim.launch.py directly so it can extend GZ_SIM_RESOURCE_PATH to
include the custom simple_room.sdf world before Gazebo starts.
The world uses <world name="warehouse"> internally so the ros-gz bridge
topic paths (/world/warehouse/model/turtlebot4/...) remain correct.
"""
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():

    # ── Package paths ─────────────────────────────────────────────────────────
    pkg_tb4        = get_package_share_directory('turtlebot4_gz_bringup')
    pkg_tb4_desc   = get_package_share_directory('turtlebot4_description')
    pkg_ir_desc    = get_package_share_directory('irobot_create_description')
    pkg_ir_gz      = get_package_share_directory('irobot_create_gz_bringup')
    pkg_tb4_gui    = get_package_share_directory('turtlebot4_gz_gui_plugins')
    pkg_ir_plugins = get_package_share_directory('irobot_create_gz_plugins')
    pkg_ros_gz     = get_package_share_directory('ros_gz_sim')
    pkg_ped_sim    = get_package_share_directory('pedestrian_sim')

    # ── Arguments ─────────────────────────────────────────────────────────────
    model_arg = DeclareLaunchArgument(
        'model', default_value='standard',
        choices=['standard', 'lite'],
        description='TurtleBot4 model variant')

    num_people_arg = DeclareLaunchArgument(
        'num_people', default_value='2',
        description='Number of simulated pedestrians')

    ped_speed_arg = DeclareLaunchArgument(
        'ped_speed', default_value='0.5',
        description='Pedestrian walking speed (m/s)')

    boundary_arg = DeclareLaunchArgument(
        'boundary_radius', default_value='3.5',
        description='Soft boundary from origin that turns pedestrians back (m)')

    # ── Environment variables ─────────────────────────────────────────────────
    # Set BEFORE Gazebo starts.  Our worlds/ directory goes first so
    # simple_room.sdf is found before the TB4 warehouse.sdf.
    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=':'.join([
            os.path.join(pkg_ped_sim, 'worlds'),         # simple_room.sdf lives here
            os.path.join(pkg_tb4, 'worlds'),
            os.path.join(pkg_ir_gz, 'worlds'),
            str(Path(pkg_tb4_desc).parent.resolve()),    # turtlebot4_description parent
            str(Path(pkg_ir_desc).parent.resolve()),     # irobot_create_description parent
        ])
    )

    gz_gui_path = SetEnvironmentVariable(
        name='GZ_GUI_PLUGIN_PATH',
        value=':'.join([
            os.path.join(pkg_tb4_gui, 'lib'),
            os.path.join(pkg_ir_plugins, 'lib'),
        ])
    )

    # ── Gazebo ────────────────────────────────────────────────────────────────
    # Launch gz-sim directly so our resource path (above) is already in effect.
    # simple_room.sdf has <world name="warehouse"> so all bridge paths match.
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments=[
            ('gz_args', [
                'simple_room.sdf',
                ' -r',
                ' -v 3',
                ' --gui-config ',
                os.path.join(pkg_tb4, 'gui', 'standard', 'gui.config'),
            ]),
        ]
    )

    # Clock bridge (sim time → ROS)
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        output='screen',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
    )

    # ── TurtleBot4 robot (spawn + all bridges + nodes) ─────────────────────────
    # turtlebot4_spawn.launch.py handles robot description, spawning into Gazebo,
    # the LiDAR bridge, cmd_vel bridge, odometry, TF, HMI, and ros2_control.
    # The 'world' argument it uses internally defaults to 'warehouse', which
    # matches our SDF's <world name="warehouse">.
    robot_spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_tb4, 'launch', 'turtlebot4_spawn.launch.py')
        ),
        launch_arguments=[
            ('model',         LaunchConfiguration('model')),
            ('namespace',     ''),
            ('use_sim_time',  'true'),
            ('x', '0.0'), ('y', '0.0'), ('z', '0.0'), ('yaw', '0.0'),
        ]
    )

    # ── Pedestrian simulator ───────────────────────────────────────────────────
    # Delayed 25 s: Gazebo + robot must be fully up before spawning models.
    # The world_name must match the SDF's <world name="..."> tag.
    pedestrian_sim = TimerAction(
        period=25.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_ped_sim, 'launch', 'pedestrian_sim.launch.py')
                ),
                launch_arguments=[
                    ('world_name',      'warehouse'),   # matches <world name="warehouse">
                    ('num_people',      LaunchConfiguration('num_people')),
                    ('speed',           LaunchConfiguration('ped_speed')),
                    ('boundary_radius', LaunchConfiguration('boundary_radius')),
                ]
            )
        ]
    )

    return LaunchDescription([
        # Arguments
        model_arg, num_people_arg, ped_speed_arg, boundary_arg,
        # Environment (must be before gazebo action)
        gz_resource_path, gz_gui_path,
        # Simulation stack
        gazebo, clock_bridge, robot_spawn,
        # Pedestrians (delayed)
        pedestrian_sim,
    ])
