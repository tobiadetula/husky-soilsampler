import rclpy
import math
import time
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose

def get_yaw_from_quaternion(q):
    """Convert quaternion to yaw (Z-axis rotation)."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))

class SimpleNavigator(Node):
    def __init__(self):
        super().__init__('simple_navigator')

        # Control Parameters
        self.declare_parameter('xy_tolerance', 0.15)       # meters
        self.declare_parameter('yaw_tolerance', 0.08)      # radians (~4.5 degrees)
        self.declare_parameter('max_linear_speed', 0.5)    # m/s
        self.declare_parameter('max_angular_speed', 0.5)   # rad/s
        
        self.xy_tol = self.get_parameter('xy_tolerance').value
        self.yaw_tol = self.get_parameter('yaw_tolerance').value
        self.max_v = self.get_parameter('max_linear_speed').value
        self.max_w = self.get_parameter('max_angular_speed').value

        # State tracking
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.odom_received = False

        # Subscriptions and Publishers
        self.cmd_pub = self.create_publisher(TwistStamped, '/a300_00008/cmd_vel', 10)
        
        # Explicitly added the namespace to ensure it hears the robot's movement
        self.odom_sub = self.create_subscription(
            Odometry, '/odometry/gps', self.odom_callback, 10)

        # Action Server
        self.cb_group = ReentrantCallbackGroup()
        self._action_server = ActionServer(
            self,
            NavigateToPose,
            '/a300_00008/navigate_to_pose',
            self.execute_callback,
            callback_group=self.cb_group
        )

        self.get_logger().info('Simple Navigator is ready and waiting for Map X/Y goals.')

    def odom_callback(self, msg):
        """Constantly update robot's current position from the global GPS-fused odometry."""
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        self.current_yaw = get_yaw_from_quaternion(msg.pose.pose.orientation)
        self.odom_received = True

    def create_velocity_command(self, linear_x=0.0, angular_z=0.0):
        """Helper to generate a properly formatted TwistStamped message."""
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'a300_00008/base_link' 
        cmd.twist.linear.x = float(linear_x)
        cmd.twist.angular.z = float(angular_z)
        return cmd
    
    def normalize_angle(self, angle):
        """Keep angle between -pi and pi."""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def stop_robot(self):   
        msg = self.create_velocity_command(0.0, 0.0)
        self.cmd_pub.publish(msg)

    def execute_callback(self, goal_handle):
        self.get_logger().info('Received new navigation goal.')

        if not self.odom_received:
            self.get_logger().error('No odometry received yet! Aborting.')
            goal_handle.abort()
            return NavigateToPose.Result()

        target_x = goal_handle.request.pose.pose.position.x
        target_y = goal_handle.request.pose.pose.position.y
        target_yaw = get_yaw_from_quaternion(goal_handle.request.pose.pose.orientation)

        rate = self.create_rate(10) # 10 Hz control loop
        feedback = NavigateToPose.Feedback()
        NAV_TIMEOUT_S = 120.0
        start_time = time.time()
        
        def timed_out():
            if time.time() - start_time > NAV_TIMEOUT_S:
                self.get_logger().error('Navigation timeout — aborting waypoint.')
                self.stop_robot()
                goal_handle.abort()
                return True
            return False
        
        # --- Phase 1: Rotate to face target ---
        self.get_logger().info('Phase 1: Aligning to target...')
        loop_count = 0
        while rclpy.ok():
            if timed_out():
                return NavigateToPose.Result()
                
            dx = target_x - self.current_x
            dy = target_y - self.current_y
            angle_to_target = math.atan2(dy, dx)
            yaw_error = self.normalize_angle(angle_to_target - self.current_yaw)
            
            if abs(yaw_error) < self.yaw_tol:
                self.get_logger().info('Phase 1 Complete: Aligned successfully.')
                break

            angular_vel = max(min(yaw_error * 1.5, self.max_w), -self.max_w)
            
            # Print debug data twice a second (every 5th loop of 10Hz)
            if loop_count % 5 == 0:
                self.get_logger().info(
                    f"[Phase 1 Debug] Tgt_Yaw: {angle_to_target:.2f} | Cur_Yaw: {self.current_yaw:.2f} | "
                    f"Err: {yaw_error:.2f} (Tol: {self.yaw_tol}) | Cmd_W: {angular_vel:.2f}"
                )
            loop_count += 1

            cmd = self.create_velocity_command(linear_x=0.0, angular_z=angular_vel)
            self.cmd_pub.publish(cmd)
            rate.sleep()

        self.stop_robot()
        time.sleep(0.5)

        # --- Phase 2: Drive to target ---
        self.get_logger().info('Phase 2: Driving to target...')
        loop_count = 0
        while rclpy.ok():
            if timed_out():
                return NavigateToPose.Result()
                
            dx = target_x - self.current_x
            dy = target_y - self.current_y
            distance = math.sqrt(dx**2 + dy**2)
            angle_to_target = math.atan2(dy, dx)
            yaw_error = self.normalize_angle(angle_to_target - self.current_yaw)

            if distance < self.xy_tol:
                self.get_logger().info('Phase 2 Complete: Arrived at X/Y target.')
                break

            linear_vel = max(min(distance * 0.8, self.max_v), 0.1)
            angular_vel = max(min(yaw_error * 1.5, self.max_w), -self.max_w)
            
            if loop_count % 5 == 0:
                self.get_logger().info(
                    f"[Phase 2 Debug] Dist: {distance:.2f}m (Tol: {self.xy_tol}) | "
                    f"Cur_XY: ({self.current_x:.2f}, {self.current_y:.2f}) | "
                    f"Cmd_V: {linear_vel:.2f}"
                )
            loop_count += 1

            cmd = self.create_velocity_command(linear_x=linear_vel, angular_z=angular_vel)
            self.cmd_pub.publish(cmd)
            
            # Send feedback
            feedback.current_pose.pose.position.x = float(self.current_x)
            feedback.current_pose.pose.position.y = float(self.current_y)
            feedback.distance_remaining = float(distance)
            goal_handle.publish_feedback(feedback)
            rate.sleep()

        self.stop_robot()
        time.sleep(0.5)

        # --- Phase 3: Rotate to final desired heading ---
        self.get_logger().info('Phase 3: Rotating to final heading...')
        loop_count = 0
        while rclpy.ok():
            if timed_out():
                return NavigateToPose.Result()
                
            yaw_error = self.normalize_angle(target_yaw - self.current_yaw)
            if abs(yaw_error) < self.yaw_tol:
                self.get_logger().info('Phase 3 Complete: Final heading achieved.')
                break

            angular_vel = max(min(yaw_error * 1.5, self.max_w), -self.max_w)
            
            if loop_count % 5 == 0:
                self.get_logger().info(
                    f"[Phase 3 Debug] Final_Tgt: {target_yaw:.2f} | Cur_Yaw: {self.current_yaw:.2f} | "
                    f"Err: {yaw_error:.2f} (Tol: {self.yaw_tol}) | Cmd_W: {angular_vel:.2f}"
                )
            loop_count += 1

            cmd = self.create_velocity_command(linear_x=0.0, angular_z=angular_vel)
            self.cmd_pub.publish(cmd)
            rate.sleep()

        self.stop_robot()

        self.get_logger().info('Waypoint reached successfully.')
        goal_handle.succeed()
        
        return NavigateToPose.Result()

def main(args=None):
    rclpy.init(args=args)
    node = SimpleNavigator()
    executor = MultiThreadedExecutor()
    rclpy.spin(node, executor=executor)
    rclpy.shutdown()

if __name__ == '__main__':
    main()