"""
people_avoidance_node.py — ROS 2 node that wires the pipeline together.

Data flow (triggered by each incoming /scan message):

    /scan  ──► detect_legs()      ──► List[LegMeasurement]
                                           │
                                    tracker.update()
                                           │
                                     List[Track]
                                           │
                                  compute_velocity()  ◄──  robot pose (/odom)
                                           │
                                        /cmd_vel

The node wires the stages; the stage bodies are stubs until students fill them in.
With all stubs in place the node publishes a zero Twist on /cmd_vel — safe to run
from day one without any implementation.

Robot pose source
-----------------
Pose is read from nav_msgs/Odometry on the `odom_topic` parameter (default: /odom).
Odometry provides the robot pose in the odom frame, which is also the frame in
which tracks are maintained — so no additional transform is required for the
controller.

For deployment beyond the simulator (e.g. with SLAM), replace the odometry
subscription with a TF lookup:

    import tf2_ros
    self.tf_buffer   = tf2_ros.Buffer()
    self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
    # In _scan_cb:
    t = self.tf_buffer.lookup_transform(odom_frame, base_frame, rclpy.time.Time())
    robot_x     = t.transform.translation.x
    robot_y     = t.transform.translation.y
    robot_theta = _yaw_from_quaternion(t.transform.rotation.{x,y,z,w})

ROS 2 Parameters (tune at launch; no code changes required)
-----------------------------------------------------------
scan_topic              str    /scan          LaserScan input topic
cmd_vel_topic           str    /cmd_vel       Twist output topic
odom_topic              str    /odom          Odometry pose topic
laser_frame             str    base_scan      Laser sensor frame id (reference only)
odom_frame              str    odom           Odometry/world frame id (reference only)
dt                      float  0.1            KF time step (s); match to scan rate
max_misses              int    5              Frames before a track is deleted
distance_threshold      float  0.1            Segmentation gap (m)
leg_radius              float  0.10           Expected single-leg radius (m)
max_leg_width           float  0.25           Max leg-pair separation (m)
max_linear_speed        float  0.2            Forward speed cap (m/s)
max_angular_speed       float  1.0            Rotation rate cap (rad/s)
obstacle_radius_scale   float  2.0            Uncertainty inflation factor k
"""
from __future__ import annotations

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

# The TurtleBot4 publishes /scan and /odom with BEST_EFFORT (SensorData) QoS.
# A default (RELIABLE) subscriber is QoS-incompatible and receives NOTHING from
# the real robot — the node would spin forever publishing zeros.  Subscribe
# with BEST_EFFORT so the pipeline actually receives sensor data on hardware.
_SENSOR_QOS = QoSProfile(depth=10,
                         reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST)

from .leg_detection import detect_legs, LegMeasurement
from .tracking import KalmanTracker
from .controller import compute_velocity


def _rotate_measurements(measurements, yaw):
    """Rotate laser-frame LegMeasurements into base_link by the lidar mount yaw.
    The RPLidar A1 on the TurtleBot4 is mounted yaw +90° vs base_link, so the
    detector/tracker/controller (which assume robot forward = +x) need the
    measurements rotated, else the robot navigates and avoids 90° off."""
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

        # ── Declare all tunable parameters ───────────────────────────────────
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

        # ── Kalman tracker (shared state across scans) ────────────────────────
        self.tracker = KalmanTracker(
            dt=p['dt'],
            max_misses=p['max_misses'],
        )

        # ── Latest robot pose — updated from /odom, consumed on each /scan ───
        self._robot_x:     float = 0.0
        self._robot_y:     float = 0.0
        self._robot_theta: float = 0.0

        # ── Subscriptions ─────────────────────────────────────────────────────
        self.create_subscription(LaserScan, p['scan_topic'], self._scan_cb, _SENSOR_QOS)
        self.create_subscription(Odometry,  p['odom_topic'], self._odom_cb, _SENSOR_QOS)

        # ── Publisher ─────────────────────────────────────────────────────────
        self._cmd_pub = self.create_publisher(Twist, p['cmd_vel_topic'], 10)

        self.get_logger().info(
            f"PeopleAvoidanceNode ready — "
            f"listening on '{p['scan_topic']}', publishing to '{p['cmd_vel_topic']}'"
        )

    # ── Callbacks ─────────────────────────────────────────────────────────────

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

        # ── Stage 1: leg detection ────────────────────────────────────────────
        measurements = detect_legs(
            scan,
            distance_threshold=p['distance_threshold'],
            leg_radius=p['leg_radius'],
            max_leg_width=p['max_leg_width'],
        )
        # Rotate laser-frame detections into base_link (lidar mount offset).
        measurements = _rotate_measurements(measurements, p['laser_yaw_offset'])

        # ── Stage 2: Kalman tracking ──────────────────────────────────────────
        self.tracker.update(measurements)
        tracks = self.tracker.get_tracks()

        # ── Stage 3: avoidance control ────────────────────────────────────────
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

    # ── Helper ────────────────────────────────────────────────────────────────

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


# ---------------------------------------------------------------------------
# Utility
# ---------------------------------------------------------------------------

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
