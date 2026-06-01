#!/usr/bin/env python3
"""
Waypoint Commander for Clearpath A300
======================================
Drives the robot to a sequence of waypoints defined in odom frame (for sim
testing) or converted from GPS coordinates (for real deployment).

Topics used:
  Publish : /a300_00000/cmd_vel          (geometry_msgs/Twist)
  Subscribe: /a300_00000/platform/odom/filtered  (nav_msgs/Odometry)

Run:
  ros2 run clearpath_waypoint_nav waypoint_commander
  ros2 run clearpath_waypoint_nav waypoint_commander --ros-args \
      -p use_gps:=false -p goal_tolerance:=0.3
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import Twist, TwistStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String


# ── State machine states ────────────────────────────────────────────────────
class State:
    IDLE = 'IDLE'
    ROTATING = 'ROTATING'
    DRIVING = 'DRIVING'
    ACTION = 'ACTION'
    COMPLETE = 'COMPLETE'


class WaypointCommander(Node):
    """Navigate to a list of (x, y) waypoints using proportional control."""

    def __init__(self):
        super().__init__('waypoint_commander')

        # ── Parameters ─────────────────────────────────────────────────────
        self.declare_parameter('robot_namespace', 'a300_00008')
        self.declare_parameter('goal_tolerance', 0.3)       # metres
        self.declare_parameter('heading_tolerance', 0.05)   # radians (~3°)
        self.declare_parameter('max_linear_speed', 1.0)     # m/s
        self.declare_parameter('max_angular_speed', 0.8)    # rad/s
        self.declare_parameter('linear_kp', 0.4)
        self.declare_parameter('angular_kp', 1.2)
        self.declare_parameter('action_duration', 3.0)      # seconds
        self.declare_parameter('use_gps', False)

        ns              = self.get_parameter('robot_namespace').value
        self.goal_tol   = self.get_parameter('goal_tolerance').value
        self.head_tol   = self.get_parameter('heading_tolerance').value
        self.max_lin    = self.get_parameter('max_linear_speed').value
        self.max_ang    = self.get_parameter('max_angular_speed').value
        self.lin_kp     = self.get_parameter('linear_kp').value
        self.ang_kp     = self.get_parameter('angular_kp').value
        self.action_dur = self.get_parameter('action_duration').value

        # ── Waypoints (x, y) in odom frame ─────────────────────────────────
        # Replace these with GPS-converted coords when running on real robot.
        # Format: (x_metres, y_metres, label)
        self.waypoints = [
            (5.0,  0.0, 'waypoint_1'),
            (5.0,  5.0, 'waypoint_2'),
            (0.0,  5.0, 'waypoint_3'),
            (0.0,  0.0, 'home'),
        ]

        # ── State ───────────────────────────────────────────────────────────
        self.state = State.IDLE
        self.wp_idx = 0
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.action_timer_remaining = 0.0

        # ── Publishers / Subscribers ────────────────────────────────────────
        self.cmd_pub = self.create_publisher(
            TwistStamped, f'/{ns}/cmd_vel', 10)

        self.status_pub = self.create_publisher(
            String, '/waypoint_commander/status', 10)

        self.odom_sub = self.create_subscription(
            Odometry,
            f'/{ns}/platform/odom/filtered',
            self._odom_cb,
            10,
        )

        # ── Control loop at 10 Hz ───────────────────────────────────────────
        self.timer = self.create_timer(0.1, self._control_loop)

        self.get_logger().info(
            f'WaypointCommander ready — {len(self.waypoints)} waypoints loaded'
        )
        self.get_logger().info('Waiting for first odometry message…')

    # ────────────────────────────────────────────────────────────────────────
    # Callbacks
    # ────────────────────────────────────────────────────────────────────────

    def _odom_cb(self, msg: Odometry):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.current_yaw = self._quat_to_yaw(q.x, q.y, q.z, q.w)

        # Kick off navigation on first odom message
        if self.state == State.IDLE and self.wp_idx < len(self.waypoints):
            self.state = State.ROTATING
            self.get_logger().info('Odometry received — starting navigation')

    def _control_loop(self):
        if self.state == State.IDLE:
            return

        if self.state == State.COMPLETE:
            self._stop()
            self._publish_status('COMPLETE', 'All waypoints finished')
            return

        if self.state == State.ACTION:
            self._run_action()
            return

        if self.wp_idx >= len(self.waypoints):
            self.state = State.COMPLETE
            return

        tx, ty, label = self.waypoints[self.wp_idx]
        dx = tx - self.current_x
        dy = ty - self.current_y
        distance = math.hypot(dx, dy)

        # ── Reached waypoint? ───────────────────────────────────────────────
        if distance < self.goal_tol:
            self._stop()
            self.get_logger().info(
                f'✓ Reached {label} (wp {self.wp_idx + 1}/{len(self.waypoints)})'
            )
            self._publish_status('REACHED', label)
            self.state = State.ACTION
            self.action_timer_remaining = self.action_dur
            return

        desired_heading = math.atan2(dy, dx)
        heading_error   = self._norm_angle(desired_heading - self.current_yaw)

        # ── Rotate in place first if heading error is large ─────────────────
        if self.state == State.ROTATING:
            if abs(heading_error) < self.head_tol:
                self.state = State.DRIVING
                self.get_logger().info(f'Heading aligned → driving to {label}')
            else:
                msg = TwistStamped()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = 'base_link'
                msg.twist.angular.z = self._clamp(
                    self.ang_kp * heading_error, -self.max_ang, self.max_ang)
                self.cmd_pub.publish(msg)
            return

        # ── Drive toward waypoint ───────────────────────────────────────────
        if self.state == State.DRIVING:
            # Re-enter rotate if heading drifts significantly
            if abs(heading_error) > 0.4:
                self.state = State.ROTATING
                return

            msg = TwistStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'base_link'
            msg.twist.linear.x  = self._clamp(
                self.lin_kp * distance, 0.05, self.max_lin)
            msg.twist.angular.z = self._clamp(
                self.ang_kp * heading_error, -self.max_ang, self.max_ang)
            self.cmd_pub.publish(msg)

    # ────────────────────────────────────────────────────────────────────────
    # Action executor  ← DEFINE YOUR CUSTOM ACTIONS HERE
    # ────────────────────────────────────────────────────────────────────────

    def _run_action(self):
        """
        Called every control loop tick while in ACTION state.
        Replace / extend this method with your real action logic.

        Ideas:
          - Call a ROS 2 service
          - Publish to an arm/actuator topic
          - Trigger a camera capture
          - Send a nav2 action goal
        """
        _, _, label = self.waypoints[self.wp_idx]

        if self.action_timer_remaining > 0:
            self.action_timer_remaining -= 0.1   # subtract one tick (0.1 s)
            if int(self.action_timer_remaining * 10) % 10 == 0:
                self.get_logger().info(
                    f'  Action @ {label}: {self.action_timer_remaining:.1f}s remaining'
                )
        else:
            self.get_logger().info(f'  Action @ {label} complete')
            self._publish_status('ACTION_DONE', label)
            self.wp_idx += 1
            if self.wp_idx < len(self.waypoints):
                self.state = State.ROTATING
                self.get_logger().info(
                    f'→ Next: {self.waypoints[self.wp_idx][2]}'
                )
            else:
                self.state = State.COMPLETE

    # ────────────────────────────────────────────────────────────────────────
    # GPS helper  (used when use_gps:=true)
    # ────────────────────────────────────────────────────────────────────────

    @staticmethod
    def gps_to_local(lat, lon, origin_lat, origin_lon):
        """
        Convert GPS coordinates to local Cartesian offsets (metres).
        Uses equirectangular approximation — accurate within ~1 km.
        For longer distances use pyproj UTM conversion instead.
        """
        R = 6_371_000  # Earth radius in metres
        dlat = math.radians(lat - origin_lat)
        dlon = math.radians(lon - origin_lon)
        x = dlon * math.cos(math.radians(origin_lat)) * R
        y = dlat * R
        return x, y

    # ────────────────────────────────────────────────────────────────────────
    # Utilities
    # ────────────────────────────────────────────────────────────────────────

    def _stop(self):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        # linear.x and angular.z default to 0.0, which stops the robot
        self.cmd_pub.publish(msg)

    def _publish_status(self, event: str, detail: str):
        msg = String()
        msg.data = f'{event}:{detail}'
        self.status_pub.publish(msg)

    @staticmethod
    def _quat_to_yaw(qx, qy, qz, qw) -> float:
        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def _norm_angle(a: float) -> float:
        while a >  math.pi: a -= 2 * math.pi
        while a < -math.pi: a += 2 * math.pi
        return a

    @staticmethod
    def _clamp(val, lo, hi):
        return max(lo, min(hi, val))


# ────────────────────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = WaypointCommander()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted — stopping robot')
        node._stop()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
