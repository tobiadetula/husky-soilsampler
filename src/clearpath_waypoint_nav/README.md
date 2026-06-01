# clearpath_waypoint_nav

Offboard waypoint navigation package for the Clearpath A300 in ROS 2 Jazzy.

Drives the robot to a sequence of (x, y) positions using proportional
control on `/cmd_vel`, with a pluggable action hook at each waypoint.

---

## Build

```bash
cd ~/clearpath_ws
colcon build --packages-select clearpath_waypoint_nav
source install/setup.bash
```

---

## Run

### With the simulation running in another terminal:

```bash
# Terminal 1 — start the simulation
ros2 launch clearpath_gz simulation.launch.py

# Terminal 2 — run the waypoint commander
source ~/clearpath_ws/install/setup.bash
ros2 run clearpath_waypoint_nav waypoint_commander
```

### Or use the launch file:

```bash
ros2 launch clearpath_waypoint_nav waypoint_nav.launch.py
```

### Override parameters at runtime:

```bash
ros2 run clearpath_waypoint_nav waypoint_commander --ros-args \
    -p max_linear_speed:=0.3 \
    -p goal_tolerance:=0.5 \
    -p action_duration:=5.0
```

---

## Monitor status

```bash
# Watch waypoint events (REACHED, ACTION_DONE, COMPLETE)
ros2 topic echo /waypoint_commander/status

# Watch odometry
ros2 topic echo /a300_00000/platform/odom/filtered

# Check cmd_vel being published
ros2 topic echo /a300_00000/cmd_vel
```

---

## Edit waypoints

Open `clearpath_waypoint_nav/waypoint_commander.py` and edit `self.waypoints`:

```python
self.waypoints = [
    (3.0,  0.0, 'waypoint_1'),   # (x_metres, y_metres, label)
    (3.0,  3.0, 'waypoint_2'),
    (0.0,  0.0, 'home'),
]
```

Rebuild after editing:
```bash
colcon build --packages-select clearpath_waypoint_nav
source install/setup.bash
```

---

## Using real GPS coordinates

Use the built-in helper to convert lat/lon to local offsets:

```python
from clearpath_waypoint_nav.waypoint_commander import WaypointCommander

origin_lat, origin_lon = 55.3959, 10.3883   # your start point (e.g. Odense)

waypoints_gps = [
    (55.3960, 10.3885),
    (55.3962, 10.3880),
]

self.waypoints = [
    (*WaypointCommander.gps_to_local(lat, lon, origin_lat, origin_lon), f'wp_{i}')
    for i, (lat, lon) in enumerate(waypoints_gps)
]
```

---

## Adding your custom action

Edit `_run_action()` in `waypoint_commander.py`:

```python
def _run_action(self):
    _, _, label = self.waypoints[self.wp_idx]
    # --- ADD YOUR ACTION HERE ---
    # e.g. call a ROS 2 service, publish to an actuator, trigger a capture
    # ----------------------------
    ...
```

---

## Package structure

```
clearpath_waypoint_nav/
├── clearpath_waypoint_nav/
│   ├── __init__.py
│   └── waypoint_commander.py    ← main node
├── config/
│   └── waypoints.yaml           ← tuning parameters
├── launch/
│   └── waypoint_nav.launch.py
├── package.xml
├── setup.cfg
├── setup.py
└── README.md
```



```
sudo apt install ros-jazzy-nav2-simple-commander ros-jazzy-nav2-msgs ros-jazzy-robot-localization
```