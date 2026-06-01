import time
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from clearpath_soil_interfaces.action import Sample
from clearpath_platform_msgs.msg import Lights, RGB

class MockSampleServer(Node):
    def __init__(self):
        super().__init__('mock_sample_server')
        self._action_server = ActionServer(
            self, Sample, 'take_soil_sample', self.execute_callback)
        self.light_pub = self.create_publisher(Lights, '/a300_00008/platform/cmd_lights', 10)
        self.get_logger().info('Mock Action Server ready. Waiting for Commander...')

    def execute_callback(self, goal_handle):
        self.get_logger().info('--- Waypoint Reached! Starting Mock Action ---')
        
        self.get_logger().info('Waiting 2 seconds...')
        time.sleep(2.0)
        
        self.get_logger().info('Blinking lights!')
        self.set_lights(255, 255, 0)
        time.sleep(0.5)
        self.set_lights(0, 0, 0)
        time.sleep(0.5)
        self.set_lights(255, 255, 0)
        time.sleep(0.5)
        self.set_lights(0, 0, 0)
        
        goal_handle.succeed()
        
        result = Sample.Result()
        result.success = True
        result.vwc = 0.0
        result.temperature = 0.0
        result.ec = 0.0
        result.message = "Mock sample complete."
        
        self.get_logger().info('Mock Action complete. Sending robot to next waypoint.\n')
        return result

    def set_lights(self, r, g, b):
        msg = Lights()
        color = RGB(red=float(r), green=float(g), blue=float(b))
        msg.lights = [color, color, color, color]
        self.light_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = MockSampleServer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()