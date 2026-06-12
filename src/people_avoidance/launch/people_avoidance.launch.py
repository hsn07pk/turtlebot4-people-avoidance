from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='people_avoidance',
            executable='people_avoidance_node',
            name='people_avoidance',
            output='screen',
            parameters=[{
                # ── Topics ──────────────────────────────────────────────────
                'scan_topic':            '/scan',
                'cmd_vel_topic':         '/cmd_vel',
                'odom_topic':            '/odom',
                # ── Frames ──────────────────────────────────────────────────
                'laser_frame':           'rplidar_link',
                'odom_frame':            'odom',
                # ── Kalman filter ────────────────────────────────────────────
                'dt':                    0.13,   # s; matches the A1 ~7.7 Hz scan rate
                'max_misses':            5,      # frames before a track is deleted
                # ── Leg detection (research-backed: Leigh leg_tracker / Arras /
                #    anthropometry — see PRESETS.md; A1 measured at 0.5°) ───────
                'distance_threshold':    0.13,   # segmentation gap (m); Leigh 0.13
                'leg_radius':            0.06,   # single-leg radius (m); calf ~6 cm
                'max_leg_width':         0.65,   # leg-pair separation (m); lecture d_max
                # RPLidar A1 is mounted yaw +90° vs base_link on the TB4
                # (measured from TF). Rotate detections so navigation/avoidance
                # are aligned with the robot's true forward direction.
                'laser_yaw_offset':      1.5708,
                # ── Controller ───────────────────────────────────────────────
                'max_linear_speed':      0.2,    # m/s
                'max_angular_speed':     1.0,    # rad/s
                'obstacle_radius_scale': 2.0,    # uncertainty inflation factor k
            }],
        ),
    ])
