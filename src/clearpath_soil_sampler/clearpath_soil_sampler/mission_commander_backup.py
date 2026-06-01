import rclpy
import math
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

# Import the service used to translate GPS to Map coordinates
from robot_localization.srv import FromLL
from geographic_msgs.msg import GeoPoint

from clearpath_soil_interfaces.action import Sample

class MissionCommander(Node):
    def __init__(self):
        super().__init__('mission_commander')
        
        # 1. Declare the runtime parameter!
        self.declare_parameter('use_gps_mode', False)
        self.use_gps = self.get_parameter('use_gps_mode').value
        
        # Action client to talk to our hardware server
        self._action_client = ActionClient(self, Sample, 'take_soil_sample')
        
        # Nav2 Commander for the A300
        self.navigator = BasicNavigator(namespace='a300_00008')
        
        # 2. Setup a client to ask robot_localization to translate GPS to XY
        if self.use_gps:
            self.get_logger().info("Operating in GPS Mode! Waiting for coordinate translator...")
            self.ll_client = self.create_client(FromLL, '/a300_00008/fromLL')
            self.ll_client.wait_for_service()
        else:
            self.get_logger().info("Operating in Cartesian (Local) Mode!")

    def generate_grid(self, start_x, start_y, width, height, spacing):
        """Generates a simple lawnmower (boustrophedon) path in Cartesian coordinates."""
        waypoints = []
        rows = int(height / spacing)
        cols = int(width / spacing)
        
        for r in range(rows):
            y = start_y + (r * spacing)
            
            # Sweep left-to-right on even rows, right-to-left on odd rows
            x_range = range(cols) if r % 2 == 0 else reversed(range(cols))
            
            for c in x_range:
                x = start_x + (c * spacing)
                # Calculate yaw so robot faces the direction of travel
                yaw = 0.0 if r % 2 == 0 else 3.14159
                waypoints.append((x, y, yaw))
                
        return waypoints
    
    def translate_gps_to_map(self, lat, lon):
        """Asks the navsat_transform node to convert Lat/Lon into local X/Y meters"""
        req = FromLL.Request()
        req.ll_point = GeoPoint(latitude=float(lat), longitude=float(lon), altitude=0.0)
        
        # Call the service synchronously
        future = self.ll_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        
        response = future.result()
        # Returns the (X, Y) coordinates in the 'map' frame
        return response.map_point.x, response.map_point.y

    def create_pose(self, x, y, yaw):
        """Helper to create a PoseStamped message for Nav2."""
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.navigator.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        return pose

    def trigger_sample_action(self):
        """Calls the hardware action server and waits for the result."""
        self.get_logger().info('Waiting for hardware action server...')
        self._action_client.wait_for_server()

        goal_msg = Sample.Goal()
        goal_msg.target_depth = 0.2 # Insert probe 20cm
        goal_msg.dwell_time = 5.0   # Wait 5 seconds for reading

        self.get_logger().info('Sending sample command to hardware...')
        send_goal_future = self._action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        
        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Hardware rejected the sample request!')
            return None

        # Wait for hardware to finish inserting, dwelling, and retracting
        get_result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, get_result_future)
        
        return get_result_future.result().result

    def run_mission(self):
        self.get_logger().info("Waiting for Nav2 to become active...")
        self.navigator.waitUntilNav2Active(localizer='bt_navigator') 
               
        # Generate a 10m x 10m grid with 5m spacing
        grid = self.generate_grid(start_x=0.0, start_y=0.0, width=10.0, height=10.0, spacing=5.0)
        self.get_logger().info(f"Grid generated with {len(grid)} waypoints.")

        for idx, (x, y, yaw) in enumerate(grid):
            self.get_logger().info(f"--- Navigating to Waypoint {idx+1}/{len(grid)}: (X:{x}, Y:{y}) ---")
            
            goal_pose = self.create_pose(x, y, yaw)
            self.navigator.goToPose(goal_pose)

            # Wait while robot drives
            while not self.navigator.isTaskComplete():
                pass # You can print feedback here if desired

            result = self.navigator.getResult()
            if result == TaskResult.SUCCEEDED:
                self.get_logger().info('Arrived at waypoint! Initiating sample sequence...')
                
                # Robot is stopped. Trigger hardware.
                sample_data = self.trigger_sample_action()
                
                if sample_data and sample_data.success:
                    self.get_logger().info(f"SUCCESS - VWC: {sample_data.vwc}%, Temp: {sample_data.temperature}C, EC: {sample_data.ec}")
                    # In reality, you would write this to a CSV or a rosbag here
                else:
                    self.get_logger().error("Sampling failed at this waypoint.")
            else:
                self.get_logger().error('Nav2 failed to reach waypoint. Skipping to next.')

        self.get_logger().info("Mission Complete!")

def main(args=None):
    rclpy.init(args=args)
    node = MissionCommander()
    node.run_mission()
    rclpy.shutdown()

if __name__ == '__main__':
    main()