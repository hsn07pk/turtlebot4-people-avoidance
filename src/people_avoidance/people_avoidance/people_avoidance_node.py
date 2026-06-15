"""ROS 2 node for people avoidance: subscribes to /scan and /odom, runs leg
detection → Kalman tracking → avoidance control, and publishes /cmd_vel."""
from __future__ import annotations

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

# Real TurtleBot4 /scan and /odom use BEST_EFFORT (SensorData) QoS; a RELIABLE subscriber receives nothing.
_SENSOR_QOS = QoSProfile(depth=10,
                         reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST)

from .leg_detection import detect_legs, LegMeasurement
from .tracking import KalmanTracker
from .controller import compute_velocity


def _rotate_measurements(measurements, yaw):
    """Rotate laser-frame measurements into base_link by the lidar mount yaw.
    The TurtleBot4 RPLidar is mounted +90° vs base_link; without this the robot avoids 90° off."""
    if abs(yaw) < 1e-6:
        return measurements
    c, s = math.cos(yaw), math.sin(yaw)
    out = []
    for m in measurements:
        x = c * m.x - s * m.y
        y = s * m.x + c * m.y
        # rotate the 2×2 covariance: R' = Rot R Rotᵀ
        rxx = c*c*m.Rxx - 2*c*s*m.Rxy + s*s*m.Ryy
        ryy = s*s*m.Rxx + 2*c*s*m.Rxy + c*c*m.Ryy
        rxy = c*s*m.Rxx + (c*c - s*s)*m.Rxy - c*s*m.Ryy
        out.append(LegMeasurement(x=x, y=y, Rxx=rxx, Rxy=rxy, Ryy=ryy))
    return out


class PeopleAvoidanceNode(Node):

    def __init__(self) -> None:
        super().__init__('people_avoidance_node')

        self.declare_parameter('scan_topic',            '/scan')
        self.declare_parameter('cmd_vel_topic',         '/cmd_vel')
        self.declare_parameter('odom_topic',            '/odom')
        self.declare_parameter('laser_frame',           'rplidar_link')
        self.declare_parameter('odom_frame',            'odom')
        self.declare_parameter('dt',                    0.1)
        self.declare_parameter('max_misses',            5)
        self.declare_parameter('distance_threshold',    0.1)
        self.declare_parameter('leg_radius',            0.10)
        self.declare_parameter('max_leg_width',         0.25)
        self.declare_parameter('laser_yaw_offset',      0.0)   # lidar mount yaw (rad)
        self.declare_parameter('max_linear_speed',      0.2)
        self.declare_parameter('max_angular_speed',     1.0)
        self.declare_parameter('obstacle_radius_scale', 2.0)

        p = self._params()

        self.tracker = KalmanTracker(
            dt=p['dt'],
            max_misses=p['max_misses'],
        )

        self._robot_x:     float = 0.0
        self._robot_y:     float = 0.0
        self._robot_theta: float = 0.0

        self.create_subscription(LaserScan, p['scan_topic'], self._scan_cb, _SENSOR_QOS)
        self.create_subscription(Odometry,  p['odom_topic'], self._odom_cb, _SENSOR_QOS)

        self._cmd_pub = self.create_publisher(Twist, p['cmd_vel_topic'], 10)

        self.get_logger().info(
            f"PeopleAvoidanceNode ready — "
            f"listening on '{p['scan_topic']}', publishing to '{p['cmd_vel_topic']}'"
        )

    def _odom_cb(self, msg: Odometry) -> None:
        """Cache the latest robot pose from odometry."""
        self._robot_x = msg.pose.pose.position.x
        self._robot_y = msg.pose.pose.position.y
        self._robot_theta = _yaw_from_quaternion(
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w,
        )

    def _scan_cb(self, scan: LaserScan) -> None:
        """Main pipeline callback — fires on every incoming LaserScan."""
        p = self._params()

        measurements = detect_legs(
            scan,
            distance_threshold=p['distance_threshold'],
            leg_radius=p['leg_radius'],
            max_leg_width=p['max_leg_width'],
        )
        measurements = _rotate_measurements(measurements, p['laser_yaw_offset'])

        self.tracker.update(measurements)
        tracks = self.tracker.get_tracks()

        cmd = compute_velocity(
            tracks,
            robot_x=self._robot_x,
            robot_y=self._robot_y,
            robot_theta=self._robot_theta,
            max_linear_speed=p['max_linear_speed'],
            max_angular_speed=p['max_angular_speed'],
            obstacle_radius_scale=p['obstacle_radius_scale'],
        )

        self._cmd_pub.publish(cmd)

        self.get_logger().debug(
            f"{len(measurements)} detections  {len(tracks)} tracks  "
            f"→  v={cmd.linear.x:.2f} m/s  ω={cmd.angular.z:.2f} rad/s"
        )

    def _params(self) -> dict:
        return {
            'scan_topic':            self.get_parameter('scan_topic').value,
            'cmd_vel_topic':         self.get_parameter('cmd_vel_topic').value,
            'odom_topic':            self.get_parameter('odom_topic').value,
            'laser_frame':           self.get_parameter('laser_frame').value,
            'odom_frame':            self.get_parameter('odom_frame').value,
            'dt':                    self.get_parameter('dt').value,
            'max_misses':            self.get_parameter('max_misses').value,
            'distance_threshold':    self.get_parameter('distance_threshold').value,
            'leg_radius':            self.get_parameter('leg_radius').value,
            'max_leg_width':         self.get_parameter('max_leg_width').value,
            'laser_yaw_offset':      self.get_parameter('laser_yaw_offset').value,
            'max_linear_speed':      self.get_parameter('max_linear_speed').value,
            'max_angular_speed':     self.get_parameter('max_angular_speed').value,
            'obstacle_radius_scale': self.get_parameter('obstacle_radius_scale').value,
        }


def _yaw_from_quaternion(qx: float, qy: float, qz: float, qw: float) -> float:
    """Extract yaw (rotation about Z) from a unit quaternion."""
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PeopleAvoidanceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
