#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from actuator_interfaces.msg import ActuatorCommand
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped
from clearpath_platform_msgs.msg import Lights, RGB
import struct
import threading

# Joystick event format: time (4B), value (2B), type (1B), number (1B)
JS_EVENT_FORMAT = 'IhBB'
JS_EVENT_SIZE = struct.calcsize(JS_EVENT_FORMAT)

JS_EVENT_BUTTON = 0x01
JS_EVENT_AXIS   = 0x02

# --- CONFIGURE YOUR BUTTONS HERE ---
# Map these to your specific gamepad's button numbers
FORWARD_BUTTON    = 4   # L1 / Left Bumper
REVERSE_BUTTON    = 5   # R1 / Right Bumper
KILL_BUTTON       = 2   # L2 (Hold for kill switch) - Note: This is a "hold" button, not a toggle.
HOME_ACTUATOR     = 5   # R2 (Hold to move to home position) - Note: This is a "hold" button, not a toggle.
HOME_ROBOT        = 9   # Share (Move robot to home position) - Note: This is a "hold" button, not a toggle.
STOP_BUTTON       = 8   # Select / Share
TARE_BUTTON       = 0   # A / Cross (Tares the load cell)
RESET_SAFE_BUTTON = 1   # B / Circle (Resets the safety trip)
ZERO_POS_BUTTON   = 3   # Y / Triangle (Zeros the hall effect position)

class JoystickMotorController(Node):
    def __init__(self):
        super().__init__('joystick_motor_controller')
        
        # Publisher to the Pico's namespace
        self.publisher = self.create_publisher(ActuatorCommand, '/microros/actuator_control', 10)
        
        # New Husky Publishers
        self.estop_pub  = self.create_publisher(Bool, '/a300_00008/platform/emergency_stop', 10)
        self.lights_pub = self.create_publisher(Lights, '/a300_00008/platform/cmd_lights', 10)
        self.home_pub   = self.create_publisher(PoseStamped, '/goal_pose', 10) # Nav2 Standard
        self.kill_pub   = self.create_publisher(Bool, '/mission/kill_switch', 10) # For mission commander awareness of kill switch state
        # "Snoop" Subscriber: Listens to other publishers (like Foxglove) so we 
        # don't accidentally override the target_position and trigger auto-mode.
        self.subscription = self.create_subscription(
            ActuatorCommand,
            '/microros/actuator_control',
            self.snoop_callback,
            10
        )
        
        # State tracking
        self.kill_switch_active = False
        self.blink_state = False
        self.last_known_target = 0
        self.current_direction = 0

        self.last_logged_direction = None
        self._lock = threading.Lock()
        # Timer for blinking lights (2Hz)
        self.create_timer(0.5, self.update_blinking)
        
        # Read joystick in a background thread
        self.js_thread = threading.Thread(target=self.read_joystick, daemon=True)
        self.js_thread.start()
        self.get_logger().info('Joystick motor controller started. Ready for input.')

    def snoop_callback(self, msg):
        """Keep track of the target position set by Foxglove/CLI"""
        self.last_known_target = msg.target_position

    def publish_command(self, direction=None,target_pos=None, tare=False, zero_pos=False, reset_safety=False, auto_mode=False):
        """Builds and publishes the unified custom message"""
        if direction is not None:
            self.current_direction = direction
        if target_pos is not None:
            self.last_known_target = target_pos

        msg = ActuatorCommand()
        msg.motor_direction = self.current_direction
        msg.target_position = self.last_known_target # Maintain the current target to stay in manual mode
        msg.tare_scale = tare
        msg.zero_position = zero_pos
        msg.reset_safety = reset_safety
        msg.use_auto_mode = auto_mode

        self.publisher.publish(msg)

        # Logging
        labels = {1: 'FORWARD', -1: 'REVERSE', 0: 'STOP'}
        dir_label = labels.get(self.current_direction, self.current_direction)

        action_log = [a for a, f in [("TARE", tare), ("ZERO_POS", zero_pos), ("RESET_SAFETY", reset_safety), ("AUTO_MODE", auto_mode)] if f]

        dir_changed = self.current_direction != self.last_logged_direction
        if dir_changed:
            self.get_logger().info(f'Motor: {dir_label}')
            self.last_logged_direction = self.current_direction
        if action_log:
            self.get_logger().info(f'Actions: {", ".join(action_log)}')
            
    def update_blinking(self):
        """Toggles lights red if kill switch is held."""
        if not self.kill_switch_active:
            return
            
        self.blink_state = not self.blink_state
        msg = Lights()
        # Husky usually has 4 light zones; set all to Red
        for _ in range(4):
            color = RGB(red=255, green=0, blue=0) if self.blink_state else RGB(red=0, green=0, blue=0)
            msg.lights.append(color)
        self.lights_pub.publish(msg)

    def trigger_robot_home(self):
        """Engages Husky Nav2 return-to-home and Homes the actuator."""
        # 1. Move Actuator to zero
        self.publish_command(direction=0, target_pos=0)
        
        # 2. Send Husky to Origin (0,0,0)
        goal = PoseStamped()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = "map"
        goal.pose.position.x = 0.0
        goal.pose.position.y = 0.0
        goal.pose.orientation.w = 1.0
        self.home_pub.publish(goal)
        self.get_logger().info('Homing both Robot and Actuator...')
        
    def read_joystick(self):
        while rclpy.ok():
            try:
                self.get_logger().info('Waiting for joystick at /dev/input/js0...')
                with open('/dev/input/js0', 'rb') as js:
                    self.get_logger().info('Joystick connected! Listening for events...')
                    while True:
                        event = js.read(JS_EVENT_SIZE)
                        if not event:
                            self.get_logger().warn('Joystick disconnected. Waiting for reconnection...')
                            break
                        _, value, ev_type, number = struct.unpack(JS_EVENT_FORMAT, event)

                        # Ignore initialization events
                        if ev_type & 0x80:
                            continue
                        # 1. Handle KILL SWITCH (L2 Hold)
                        
                        if ev_type == JS_EVENT_AXIS and number == KILL_BUTTON:
                            self.kill_switch_active = (value > 0)
                            # Engage E-Stop Topic
                            self.estop_pub.publish(Bool(data=self.kill_switch_active))
                            self.kill_pub.publish(Bool(data=self.kill_switch_active)) # Also publish to mission commander topic for awareness
                            if self.kill_switch_active:
                                self.get_logger().warn('KILL SWITCH ENGAGED — E-Stop active')
                                # Force actuator to stop and go to zero
                                self.publish_command(direction=0)
                            else:
                                self.get_logger().info('Kill switch released')
                        # 2. Handle ACTUATOR HOME (R2 Hold)
                        
                        elif ev_type == JS_EVENT_AXIS and number == HOME_ACTUATOR:
                            if value > 0: # R2 pressed
                                self.get_logger().info('Homing actuator to zero position...')   
                                self.publish_command(target_pos=0,auto_mode=True)  # Move to zero position in auto mode

                        # 3. Handle ROBOT HOME (D-Pad Up Hold)
                        elif ev_type == JS_EVENT_BUTTON and number == HOME_ROBOT:
                            if value == 1: # D-Pad UP pressed
                                self.get_logger().info('Triggering robot home sequence...') 
                                self.trigger_robot_home()

                        # 4. Filter existing manual controls if Kill Switch is active
                        if self.kill_switch_active:
                            continue  # Ignore all other inputs when kill switch is active           
                        if ev_type == JS_EVENT_BUTTON and value == 1:  # Button pressed
                            if number == FORWARD_BUTTON:
                                self.publish_command(direction=1)
                            elif number == REVERSE_BUTTON:
                                self.publish_command(direction=-1)
                            elif number == STOP_BUTTON:
                                self.publish_command(direction=0)
                            elif number == TARE_BUTTON:
                                self.publish_command(tare=True)
                            elif number == ZERO_POS_BUTTON:
                                self.publish_command(zero_pos=True)
                            elif number == RESET_SAFE_BUTTON:
                                self.publish_command(reset_safety=True)

                        elif ev_type == JS_EVENT_BUTTON and value == 0:  # Button released
                            if number in (FORWARD_BUTTON, REVERSE_BUTTON):
                                self.publish_command(direction=0)  # Stop on release

            except FileNotFoundError:
                self.get_logger().error('Joystick not found at /dev/input/js0')
                time.sleep(1.0)  # Wait before retrying
            except PermissionError:
                self.get_logger().error('Permission denied — try: sudo chmod a+r /dev/input/js0')
                time.sleep(1.0)  # Wait before retrying
            except OSError as e:
                self.get_logger().warn(f'Joystick error: {e} — retrying...')
                self.publish_command(direction=0)  # Safety stop on unexpected disconnect
                time.sleep(1.0)
def main():
    rclpy.init()
    node = JoystickMotorController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Make sure motor stops on exit
        node.publish_command(direction=0)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()