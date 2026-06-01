import rclpy
import math
import csv
import os
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import NavSatFix
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

from robot_localization.srv import FromLL
from geographic_msgs.msg import GeoPoint

from clearpath_soil_interfaces.action import Sample


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def haversine_distance(lat1, lon1, lat2, lon2):
    """Returns distance in metres between two GPS points."""
    R = 6371000.0
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlambda = math.radians(lon2 - lon1)
    a = math.sin(dphi / 2) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlambda / 2) ** 2
    return R * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))


def bearing(lat1, lon1, lat2, lon2):
    """Returns bearing in radians from point 1 → point 2 (North = 0, East = π/2)."""
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dlambda = math.radians(lon2 - lon1)
    x = math.sin(dlambda) * math.cos(phi2)
    y = math.cos(phi1) * math.sin(phi2) - math.sin(phi1) * math.cos(phi2) * math.cos(dlambda)
    return math.atan2(x, y)


def offset_gps(lat, lon, distance_m, bearing_rad):
    """Returns a new GPS point offset from (lat, lon) by distance_m along bearing."""
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

        # ── Parameters ──────────────────────────────────────────────────────
        self.declare_parameter('use_gps_mode', True)

        # GPS polygon corners (lat/lon pairs as a flat list)
        # e.g. [lat0, lon0, lat1, lon1, lat2, lon2, lat3, lon3]
        self.declare_parameter('field_polygon_gps', [0.0, 0.0, 0.0, 0.0])

        # Cartesian grid (used when use_gps_mode = false)
        self.declare_parameter('cartesian_start_x', 0.0)
        self.declare_parameter('cartesian_start_y', 0.0)
        self.declare_parameter('cartesian_width',   10.0)
        self.declare_parameter('cartesian_height',  10.0)

        # Shared
        self.declare_parameter('row_spacing',   5.0)   # metres between rows
        self.declare_parameter('point_spacing', 5.0)   # metres between points along row
        self.declare_parameter('sample_depth',  0.2)
        self.declare_parameter('dwell_time',    5.0)
        self.declare_parameter('output_csv',    '')    # path to write results CSV

        self.use_gps    = self.get_parameter('use_gps_mode').value
        self.row_sp     = self.get_parameter('row_spacing').value
        self.point_sp   = self.get_parameter('point_spacing').value
        self.depth      = self.get_parameter('sample_depth').value
        self.dwell      = self.get_parameter('dwell_time').value
        self.csv_path   = self.get_parameter('output_csv').value

        # ── Publishers ───────────────────────────────────────────────────────
        # MarkerArray so every waypoint is visible in RViz/Foxglove
        self.marker_pub = self.create_publisher(
            MarkerArray, '/a300_00008/mission/waypoints', 10)

        # NavSatFix array republished one-by-one for GPS visualizers
        self.gps_pub = self.create_publisher(
            NavSatFix, '/a300_00008/mission/waypoint_gps', 10)

        # ── Nav2 / action client ─────────────────────────────────────────────
        self._action_client = ActionClient(self, Sample, 'take_soil_sample')
        self.navigator = BasicNavigator(namespace='a300_00008')

        # ── GPS translation service ──────────────────────────────────────────
        if self.use_gps:
            self.get_logger().info("GPS mode — waiting for navsat_transform (fromLL)...")
            self.ll_client = self.create_client(FromLL, '/a300_00008/fromLL')
            self.ll_client.wait_for_service()
            self.get_logger().info("navsat_transform ready.")
        else:
            self.get_logger().info("Cartesian (IMU/odometry) mode.")

        # Results log
        self._results: list[dict] = []

    # ── Grid generation ──────────────────────────────────────────────────────

    def generate_gps_grid(self, polygon_latlon: list[tuple]) -> list[tuple]:
        """
        Accepts a list of (lat, lon) corner points defining a convex field polygon.
        Generates a lawnmower grid aligned to the first edge of the polygon.

        Returns a list of (lat, lon, yaw_rad) tuples published to ROS topics.
        """
        if len(polygon_latlon) < 2:
            raise ValueError("Need at least 2 polygon corners.")

        # Use first edge as the sweep direction
        p0 = polygon_latlon[0]
        p1 = polygon_latlon[1]

        sweep_bearing  = bearing(p0[0], p0[1], p1[0], p1[1])
        perp_bearing   = sweep_bearing + math.pi / 2.0

        # Width = length of first edge, Height = perpendicular span
        width_m  = haversine_distance(p0[0], p0[1], p1[0], p1[1])

        if len(polygon_latlon) >= 3:
            p2 = polygon_latlon[2]
            height_m = haversine_distance(p1[0], p1[1], p2[0], p2[1])
        else:
            height_m = self.row_sp  # single strip

        cols = max(1, int(width_m  / self.point_sp))
        rows = max(1, int(height_m / self.row_sp))

        self.get_logger().info(
            f"GPS grid: {width_m:.1f}m × {height_m:.1f}m  →  "
            f"{cols} cols × {rows} rows = {cols * rows} waypoints"
        )

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
        """Lawnmower grid in local X/Y map frame (no GPS required)."""
        sx  = self.get_parameter('cartesian_start_x').value
        sy  = self.get_parameter('cartesian_start_y').value
        w   = self.get_parameter('cartesian_width').value
        h   = self.get_parameter('cartesian_height').value

        cols = max(1, int(w / self.point_sp))
        rows = max(1, int(h / self.row_sp))

        self.get_logger().info(
            f"Cartesian grid: {w:.1f}m × {h:.1f}m  →  "
            f"{cols} cols × {rows} rows = {cols * rows} waypoints"
        )

        waypoints = []
        for r in range(rows):
            y = sy + r * self.row_sp
            col_range = range(cols) if r % 2 == 0 else reversed(range(cols))
            for c in col_range:
                x   = sx + c * self.point_sp
                yaw = 0.0 if r % 2 == 0 else math.pi
                waypoints.append((x, y, yaw))
        return waypoints

    # ── Coordinate translation ───────────────────────────────────────────────

    def translate_gps_to_map(self, lat: float, lon: float) -> tuple[float, float]:
        req = FromLL.Request()
        req.ll_point = GeoPoint(latitude=float(lat), longitude=float(lon), altitude=0.0)
        future = self.ll_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        resp = future.result()
        return resp.map_point.x, resp.map_point.y

    # ── ROS publishing ───────────────────────────────────────────────────────

    def publish_waypoints_gps(self, waypoints_gps: list[tuple]):
        """
        Publishes each GPS waypoint as:
          • A sphere Marker in a MarkerArray (visible in RViz / Foxglove)
          • An individual NavSatFix message
        Also logs a summary table.
        """
        marker_array = MarkerArray()

        self.get_logger().info(
            f"\n{'─'*55}\n"
            f"  {'#':>3}  {'Latitude':>12}  {'Longitude':>13}  {'Yaw°':>6}\n"
            f"{'─'*55}"
        )

        for idx, (lat, lon, yaw) in enumerate(waypoints_gps):
            yaw_deg = math.degrees(yaw) % 360

            self.get_logger().info(
                f"  {idx+1:>3}  {lat:>12.7f}  {lon:>13.7f}  {yaw_deg:>6.1f}°"
            )

            # NavSatFix
            fix = NavSatFix()
            fix.header.stamp    = self.get_clock().now().to_msg()
            fix.header.frame_id = 'map'
            fix.latitude        = lat
            fix.longitude       = lon
            fix.altitude        = 0.0
            self.gps_pub.publish(fix)

            # Sphere marker (map frame — requires GPS→map translation)
            m          = Marker()
            m.header.stamp    = self.get_clock().now().to_msg()
            m.header.frame_id = 'map'
            m.ns       = 'waypoints'
            m.id       = idx
            m.type     = Marker.SPHERE
            m.action   = Marker.ADD
            m.scale.x  = m.scale.y = m.scale.z = 0.4

            # Colour: green for first/last, blue otherwise
            if idx == 0:
                m.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)
            elif idx == len(waypoints_gps) - 1:
                m.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)
            else:
                m.color = ColorRGBA(r=0.2, g=0.6, b=1.0, a=0.9)

            # Translate to map X/Y for the marker position
            mx, my         = self.translate_gps_to_map(lat, lon)
            m.pose.position.x = mx
            m.pose.position.y = my
            m.pose.position.z = 0.1
            m.pose.orientation.w = 1.0

            marker_array.markers.append(m)

        self.get_logger().info(f"{'─'*55}")
        self.marker_pub.publish(marker_array)
        self.get_logger().info(
            f"Published {len(waypoints_gps)} waypoints to "
            f"/a300_00008/mission/waypoints and /a300_00008/mission/waypoint_gps"
        )

    def publish_waypoints_cartesian(self, waypoints_xy: list[tuple]):
        """Publishes cartesian waypoints as a MarkerArray (no GPS needed)."""
        marker_array = MarkerArray()

        self.get_logger().info(
            f"\n{'─'*45}\n"
            f"  {'#':>3}  {'X (m)':>10}  {'Y (m)':>10}  {'Yaw°':>6}\n"
            f"{'─'*45}"
        )

        for idx, (x, y, yaw) in enumerate(waypoints_xy):
            self.get_logger().info(
                f"  {idx+1:>3}  {x:>10.3f}  {y:>10.3f}  {math.degrees(yaw):>6.1f}°"
            )

            m          = Marker()
            m.header.stamp    = self.get_clock().now().to_msg()
            m.header.frame_id = 'map'
            m.ns       = 'waypoints'
            m.id       = idx
            m.type     = Marker.SPHERE
            m.action   = Marker.ADD
            m.scale.x  = m.scale.y = m.scale.z = 0.4

            if idx == 0:
                m.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)
            elif idx == len(waypoints_xy) - 1:
                m.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)
            else:
                m.color = ColorRGBA(r=0.2, g=0.6, b=1.0, a=0.9)

            m.pose.position.x = float(x)
            m.pose.position.y = float(y)
            m.pose.position.z = 0.1
            m.pose.orientation.z = math.sin(yaw / 2.0)
            m.pose.orientation.w = math.cos(yaw / 2.0)

            marker_array.markers.append(m)

        self.get_logger().info(f"{'─'*45}")
        self.marker_pub.publish(marker_array)

    # ── Nav2 helpers ─────────────────────────────────────────────────────────

    def create_pose(self, x: float, y: float, yaw: float) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp    = self.navigator.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        return pose

    def trigger_sample_action(self):
        self.get_logger().info('Waiting for hardware action server...')
        self._action_client.wait_for_server()

        goal_msg = Sample.Goal()
        goal_msg.target_depth = float(self.depth)
        goal_msg.dwell_time   = float(self.dwell)

        send_goal_future = self._action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)

        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Hardware rejected the sample request!')
            return None

        get_result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, get_result_future)
        return get_result_future.result().result

    # ── CSV output ───────────────────────────────────────────────────────────

    def save_results_csv(self):
        if not self.csv_path or not self._results:
            return
        os.makedirs(os.path.dirname(self.csv_path) or '.', exist_ok=True)
        with open(self.csv_path, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=self._results[0].keys())
            writer.writeheader()
            writer.writerows(self._results)
        self.get_logger().info(f"Results saved to {self.csv_path}")

    # ── Main mission loop ────────────────────────────────────────────────────

    def run_mission(self):
        self.get_logger().info("Waiting for Nav2 to become active...")
        self.navigator.waitUntilNav2Active(localizer='bt_navigator')

        # ── Build waypoint list ──────────────────────────────────────────────
        if self.use_gps:
            raw = self.get_parameter('field_polygon_gps').value
            if len(raw) < 4 or len(raw) % 2 != 0:
                self.get_logger().error(
                    "field_polygon_gps must be a flat list of lat/lon pairs, "
                    "e.g. [lat0,lon0,lat1,lon1,...].  Aborting.")
                return

            polygon = [(raw[i], raw[i+1]) for i in range(0, len(raw), 2)]
            gps_waypoints = self.generate_gps_grid(polygon)
            self.publish_waypoints_gps(gps_waypoints)

            # Convert GPS → map X/Y for Nav2
            nav_waypoints = []
            for lat, lon, yaw in gps_waypoints:
                x, y = self.translate_gps_to_map(lat, lon)
                nav_waypoints.append((x, y, yaw, lat, lon))

        else:
            xy_waypoints = self.generate_cartesian_grid()
            self.publish_waypoints_cartesian(xy_waypoints)
            nav_waypoints = [(x, y, yaw, None, None) for x, y, yaw in xy_waypoints]

        self.get_logger().info(
            f"Starting mission: {len(nav_waypoints)} waypoints.")

        # ── Execute ──────────────────────────────────────────────────────────
        for idx, (x, y, yaw, lat, lon) in enumerate(nav_waypoints):
            label = (f"Lat:{lat:.6f} Lon:{lon:.6f}" if lat is not None
                     else f"X:{x:.2f} Y:{y:.2f}")
            self.get_logger().info(
                f"── Waypoint {idx+1}/{len(nav_waypoints)}: {label} ──")

            goal_pose = self.create_pose(x, y, yaw)
            self.navigator.goToPose(goal_pose)

            while not self.navigator.isTaskComplete():
                pass

            nav_result = self.navigator.getResult()
            if nav_result == TaskResult.SUCCEEDED:
                self.get_logger().info('Arrived. Starting sample sequence...')
                sample_data = self.trigger_sample_action()

                if sample_data and sample_data.success:
                    self.get_logger().info(
                        f"✓ VWC:{sample_data.vwc:.1f}%  "
                        f"Temp:{sample_data.temperature:.1f}°C  "
                        f"EC:{sample_data.ec:.2f}")
                    self._results.append({
                        'waypoint': idx + 1,
                        'latitude': lat, 'longitude': lon,
                        'map_x': x,     'map_y': y,
                        'vwc':  sample_data.vwc,
                        'temperature': sample_data.temperature,
                        'ec':   sample_data.ec,
                    })
                else:
                    self.get_logger().error("Sampling failed at this waypoint.")
            else:
                self.get_logger().error(
                    f"Nav2 failed to reach waypoint {idx+1}. Skipping.")

        self.get_logger().info("═══ Mission Complete! ═══")
        self.save_results_csv()


# ── Entry point ──────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = MissionCommander()
    node.run_mission()
    rclpy.shutdown()

if __name__ == '__main__':
    main()