from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pedestrian_sim',
            executable='pedestrian_sim_node',
            name='pedestrian_sim',
            output='screen',
            parameters=[{
                # ── Must match the Gazebo world name ─────────────────────────
                # Check with:  gz service -l | grep create
                # Typically 'depot' for the default TB4 world.
                'world_name':      'warehouse',

                # ── Pedestrian behaviour ──────────────────────────────────────
                'num_people':      2,      # number of people to spawn
                'speed':           0.5,    # walking speed (m/s)
                'turn_noise_std':  0.4,    # heading noise per step (rad)
                'update_hz':       5.0,    # pose update rate (Hz)
                'boundary_radius': 3.5,    # soft boundary from origin (m)
            }],
        ),
    ])
