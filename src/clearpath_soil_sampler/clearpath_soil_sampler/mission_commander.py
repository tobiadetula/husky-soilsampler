import rclpy
import math
import csv
import os
import time
import json  # <-- Added for GeoJSON
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import NavSatFix
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Bool, ColorRGBA, String  # <-- Added String for GeoJSON
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Path
from robot_localization.srv import FromLL
from geographic_msgs.msg import GeoPoint

from clearpath_soil_interfaces.action import Sample

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def haversine_distance(lat1, lon1, lat2, lon2):
    R = 6371000.0
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlambda = math.radians(lon2 - lon1)
    a = math.sin(dphi / 2) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlambda / 2) ** 2
    return R * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

def bearing(lat1, lon1, lat2, lon2):
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dlambda = math.radians(lon2 - lon1)
    x = math.sin(dlambda) * math.cos(phi2)
    y = math.cos(phi1) * math.sin(phi2) - math.sin(phi1) * math.cos(phi2) * math.cos(dlambda)
    return math.atan2(x, y)

def offset_gps(lat, lon, distance_m, bearing_rad):
    R = 6371000.0
    lat2 = math.asin(
        math.sin(math.radians(lat)) * math.cos(distance_m / R)
        + math.cos(math.radians(lat)) * math.sin(distance_m / R) * math.cos(bearing_rad)
    )
    lon2 = math.radians(lon) + math.atan2(
        math.sin(bearing_rad) * math.sin(distance_m / R) * math.cos(math.radians(lat)),
        math.cos(distance_m / R) - math.sin(math.radians(lat)) * math.sin(lat2),
    )
    return math.degrees(lat2), math.degrees(lon2)

# ---------------------------------------------------------------------------
# MissionCommander
# ---------------------------------------------------------------------------

class MissionCommander(Node):
    def __init__(self):
        super().__init__('mission_commander')

        self.cb_group = ReentrantCallbackGroup()

        # ── Parameters ──────────────────────────────────────────────────────
        self.declare_parameter('use_gps_mode', True)
        self.declare_parameter('field_polygon_gps', [0.0, 0.0, 0.0, 0.0])
        self.declare_parameter('cartesian_start_x', 0.0)
        self.declare_parameter('cartesian_start_y', 0.0)
        self.declare_parameter('cartesian_width',   10.0)
        self.declare_parameter('cartesian_height',  10.0)
        self.declare_parameter('row_spacing',   5.0)
        self.declare_parameter('point_spacing', 5.0)
        self.declare_parameter('sample_depth',  0.2)
        self.declare_parameter('dwell_time',    5.0)
        self.declare_parameter('output_csv',    '')

        self.use_gps    = self.get_parameter('use_gps_mode').value
        self.row_sp     = self.get_parameter('row_spacing').value
        self.point_sp   = self.get_parameter('point_spacing').value
        self.depth      = self.get_parameter('sample_depth').value
        self.dwell      = self.get_parameter('dwell_time').value
        self.csv_path   = self.get_parameter('output_csv').value

        # ── Publishers ───────────────────────────────────────────────────────
        self.marker_pub = self.create_publisher(MarkerArray, '/a300_00008/mission/waypoints', 10)
        self.path_pub = self.create_publisher(Path, '/a300_00008/mission/planned_path', 10)
        self.geojson_pub = self.create_publisher(String, '/a300_00008/mission/planned_geojson', 10)
        
        self._waypoint_markers = MarkerArray()
        self._planned_path = Path()
        self._planned_geojson = String()
        self.create_timer(2.0, self._republish_visuals, callback_group=self.cb_group)
        
        self.current_wp_pub = self.create_publisher(NavSatFix, '/a300_00008/mission/current_waypoint_gps', 10)

        # ── Action Clients ─────────────────────────────────────────────
        self._action_client = ActionClient(self, Sample, 'take_soil_sample', callback_group=self.cb_group)
        self._nav_client = ActionClient(self, NavigateToPose, '/a300_00008/navigate_to_pose', callback_group=self.cb_group)

        # ── GPS translation service ──────────────────────────────────────────
        if self.use_gps:
            self.get_logger().info("GPS mode — waiting for navsat_transform (fromLL)...")
            self.ll_client = self.create_client(FromLL, '/a300_00008/fromLL', callback_group=self.cb_group)
            self.ll_client.wait_for_service()
            self.get_logger().info("navsat_transform ready.")
        else:
            self.get_logger().info("Cartesian (IMU/odometry) mode.")

        # ── State Tracking ───────────────────────────────────────────────────
        self._killed = False
        self.create_subscription(Bool, '/mission/kill_switch', self._kill_cb, 10, callback_group=self.cb_group)
        self._active_nav_handle = None
        self._results = []
        
        self.timer = self.create_timer(1.0, self.run_mission, callback_group=self.cb_group)

    def _kill_cb(self, msg):
        if msg.data and not self._killed:
            self._killed = True
            self.get_logger().warn('KILL SWITCH — cancelling active navigation!')
            if self._active_nav_handle is not None:
                self._active_nav_handle.cancel_goal_async()
        elif not msg.data:
            self._killed = False
            self.get_logger().info('Kill switch released — mission can resume.')
            
    def _republish_visuals(self):
        if self._waypoint_markers.markers:
            self.marker_pub.publish(self._waypoint_markers)
        if self._planned_path.poses:
            self.path_pub.publish(self._planned_path)
        if self._planned_geojson.data:
            self.geojson_pub.publish(self._planned_geojson)

    def generate_gps_grid(self, polygon_latlon: list[tuple]) -> list[tuple]:
        if len(polygon_latlon) < 2:
            raise ValueError("Need at least 2 polygon corners.")
        p0 = polygon_latlon[0]
        p1 = polygon_latlon[1]
        sweep_bearing  = bearing(p0[0], p0[1], p1[0], p1[1])
        perp_bearing   = sweep_bearing + math.pi / 2.0
        width_m  = haversine_distance(p0[0], p0[1], p1[0], p1[1])
        if len(polygon_latlon) >= 3:
            p2 = polygon_latlon[2]
            height_m = haversine_distance(p1[0], p1[1], p2[0], p2[1])
        else:
            height_m = self.row_sp
        cols = max(1, int(width_m  / self.point_sp))
        rows = max(1, int(height_m / self.row_sp))
        waypoints = []
        for r in range(rows):
            row_origin = offset_gps(p0[0], p0[1], r * self.row_sp, perp_bearing)
            forward    = sweep_bearing if r % 2 == 0 else sweep_bearing + math.pi
            col_range = range(cols) if r % 2 == 0 else reversed(range(cols))
            for c in col_range:
                pt = offset_gps(row_origin[0], row_origin[1], c * self.point_sp, sweep_bearing)
                waypoints.append((pt[0], pt[1], forward))
        return waypoints

    def generate_cartesian_grid(self) -> list[tuple]:
        sx  = self.get_parameter('cartesian_start_x').value
        sy  = self.get_parameter('cartesian_start_y').value
        w   = self.get_parameter('cartesian_width').value
        h   = self.get_parameter('cartesian_height').value
        cols = max(1, int(w / self.point_sp))
        rows = max(1, int(h / self.row_sp))
        waypoints = []
        for r in range(rows):
            y = sy + r * self.row_sp
            col_range = range(cols) if r % 2 == 0 else reversed(range(cols))
            for c in col_range:
                x   = sx + c * self.point_sp
                yaw = 0.0 if r % 2 == 0 else math.pi
                waypoints.append((x, y, yaw))
        return waypoints

    def translate_gps_to_map(self, lat: float, lon: float) -> tuple[float, float]:
        req = FromLL.Request()
        req.ll_point = GeoPoint(latitude=float(lat), longitude=float(lon), altitude=0.0)
        future = self.ll_client.call_async(req)
        while not future.done():
            time.sleep(0.05)
        resp = future.result()
        return resp.map_point.x, resp.map_point.y

    def publish_waypoints_gps(self, waypoints_gps: list[tuple]):
        marker_array = MarkerArray()
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'map'

        for idx, (lat, lon, yaw) in enumerate(waypoints_gps):
            mx, my = self.translate_gps_to_map(lat, lon)
            
            # 1. Build the Marker (Sphere)
            m = Marker()
            m.header = path_msg.header
            m.ns = 'waypoints'
            m.id = idx
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.scale.x = m.scale.y = m.scale.z = 0.4
            if idx == 0:
                m.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)
            elif idx == len(waypoints_gps) - 1:
                m.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)
            else:
                m.color = ColorRGBA(r=0.2, g=0.6, b=1.0, a=0.9)
            
            m.pose.position.x = mx
            m.pose.position.y = my
            m.pose.position.z = 0.1
            m.pose.orientation.w = 1.0
            marker_array.markers.append(m)

            # 2. Build the Path point
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = mx
            pose.pose.position.y = my
            pose.pose.orientation.z = math.sin(yaw / 2.0)
            pose.pose.orientation.w = math.cos(yaw / 2.0)
            path_msg.poses.append(pose)

        self._waypoint_markers = marker_array
        self._planned_path = path_msg
        self.marker_pub.publish(marker_array)
        self.path_pub.publish(path_msg)

        # 3. Build GeoJSON for Foxglove Map
        # GeoJSON expects [longitude, latitude] order
        coords = [[lon, lat] for lat, lon, yaw in waypoints_gps]
        
        geojson_dict = {
            "type": "FeatureCollection",
            "features": [
                {
                    "type": "Feature",
                    "geometry": {
                        "type": "LineString",
                        "coordinates": coords
                    },
                    "properties": {"stroke": "#00aaff", "stroke-width": 4} # Blue Line
                },
                {
                    "type": "Feature",
                    "geometry": {
                        "type": "MultiPoint",
                        "coordinates": coords
                    },
                    "properties": {"marker-color": "#ff0000", "marker-size": "small"} # Red Dots
                }
            ]
        }
        
        geojson_msg = String()
        geojson_msg.data = json.dumps(geojson_dict)
        self._planned_geojson = geojson_msg
        self.geojson_pub.publish(geojson_msg)

    def publish_waypoints_cartesian(self, waypoints_xy: list[tuple]):
        marker_array = MarkerArray()
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'map'

        for idx, (x, y, yaw) in enumerate(waypoints_xy):
            # 1. Build the Marker
            m = Marker()
            m.header = path_msg.header
            m.ns = 'waypoints'
            m.id = idx
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.scale.x = m.scale.y = m.scale.z = 0.4
            if idx == 0:
                m.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)
            elif idx == len(waypoints_xy) - 1:
                m.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)
            else:
                m.color = ColorRGBA(r=0.2, g=0.6, b=1.0, a=0.9)
            
            m.pose.position.x = float(x)
            m.pose.position.y = float(y)
            m.pose.position.z = 0.1
            m.pose.orientation.w = 1.0
            marker_array.markers.append(m)

            # 2. Build the Path point
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.orientation.z = math.sin(yaw / 2.0)
            pose.pose.orientation.w = math.cos(yaw / 2.0)
            path_msg.poses.append(pose)

        self._waypoint_markers = marker_array
        self._planned_path = path_msg
        self.marker_pub.publish(marker_array)
        self.path_pub.publish(path_msg)

    def create_pose(self, x: float, y: float, yaw: float) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp    = self.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        return pose

    def trigger_sample_action(self):
        self._action_client.wait_for_server()
        goal_msg = Sample.Goal()
        goal_msg.target_depth = float(self.depth)
        goal_msg.dwell_time   = float(self.dwell)

        send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.sample_feedback_callback)
        while not send_goal_future.done():
            time.sleep(0.05)
            
        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Hardware rejected the sample request!')
            return None

        get_result_future = goal_handle.get_result_async()
        while not get_result_future.done():
            time.sleep(0.1)
        return get_result_future.result().result

    def sample_feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        if "ERROR" in feedback.current_state:
            self.get_logger().warn(f"⚠️ HARDWARE FEEDBACK: {feedback.current_state} at depth {feedback.current_depth} ticks!")

    def save_results_csv(self):
        if not self.csv_path or not self._results:
            return
        os.makedirs(os.path.dirname(self.csv_path) or '.', exist_ok=True)
        with open(self.csv_path, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=self._results[0].keys())
            writer.writeheader()
            writer.writerows(self._results)
        self.get_logger().info(f"Results saved to {self.csv_path}")

    def run_mission(self):
        self.timer.cancel()

        if self.use_gps:
            raw = self.get_parameter('field_polygon_gps').value
            polygon = [(raw[i], raw[i+1]) for i in range(0, len(raw), 2)]
            gps_waypoints = self.generate_gps_grid(polygon)
            self.publish_waypoints_gps(gps_waypoints)
            nav_waypoints = []
            for lat, lon, yaw in gps_waypoints:
                x, y = self.translate_gps_to_map(lat, lon)
                nav_waypoints.append((x, y, yaw, lat, lon))
        else:
            xy_waypoints = self.generate_cartesian_grid()
            self.publish_waypoints_cartesian(xy_waypoints)
            nav_waypoints = [(x, y, yaw, None, None) for x, y, yaw in xy_waypoints]

        self.get_logger().info(f"Starting mission: {len(nav_waypoints)} waypoints.")

        for idx, (x, y, yaw, lat, lon) in enumerate(nav_waypoints):
            self.get_logger().info(f"── Waypoint {idx+1}/{len(nav_waypoints)}: X:{x:.2f} Y:{y:.2f} ──")

            self._nav_client.wait_for_server()
            goal_msg = NavigateToPose.Goal()
            goal_msg.pose = self.create_pose(x, y, yaw)
            
            if lat is not None:
                fix = NavSatFix()
                fix.header.stamp = self.get_clock().now().to_msg()
                fix.header.frame_id = 'map'
                fix.latitude = lat
                fix.longitude = lon
                self.current_wp_pub.publish(fix)

            send_goal_future = self._nav_client.send_goal_async(goal_msg)
            while not send_goal_future.done():
                time.sleep(0.05)
                
            nav_goal_handle = send_goal_future.result()
            if not nav_goal_handle.accepted:
                self.get_logger().error('Navigator rejected the goal!')
                continue
            
            self._active_nav_handle = nav_goal_handle

            nav_result_future = nav_goal_handle.get_result_async()
            while not nav_result_future.done():
                if self._killed:
                    self.get_logger().warn('Kill switch active — aborting mission loop.')
                    return
                time.sleep(0.1)

            self._active_nav_handle = None

            if nav_result_future.result().status == 4: 
                self.get_logger().info('Arrived. Starting sample sequence...')
                sample_data = self.trigger_sample_action()

                if sample_data and sample_data.success:
                    self.get_logger().info(f"✓ VWC:{sample_data.vwc:.1f}% Temp:{sample_data.temperature:.1f}°C")
                    self._results.append({
                        'waypoint': idx + 1,
                        'map_x': x, 'map_y': y,
                        'vwc': sample_data.vwc,
                        'vwc_stddev': sample_data.vwc_stddev,
                    })
                else:
                    self.get_logger().error("Sampling failed at this waypoint.")
            else:
                self.get_logger().error(f"Failed to reach waypoint {idx+1}. Skipping.")

        self.get_logger().info("═══ Mission Complete! ═══")
        self.save_results_csv()

def main(args=None):
    rclpy.init(args=args)
    node = MissionCommander()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()