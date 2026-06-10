## Waypoint Stack Launch

Build and source the package, then launch the complete waypoint subsystem:

```bash
colcon build --packages-select tree_waypoint_planner
source install/setup.bash
ros2 launch tree_waypoint_planner tree_waypoint_stack.launch.py
```

For a visual test using the planner's hard-coded trees without the boundary filter or tree memory:

```bash
ros2 launch tree_waypoint_planner tree_waypoint_stack.launch.py \
  use_boundary_filter:=false \
  use_memory:=false \
  use_hardcoded_trees:=true
```

For live detections published on `/trees`, enable memory and disable hard-coded trees:

```bash
ros2 launch tree_waypoint_planner tree_waypoint_stack.launch.py \
  use_memory:=true \
  use_hardcoded_trees:=false
```

The boundary filter requires an occupancy grid on `/map`. Disable it with
`use_boundary_filter:=false` when no map is available.

`auto_send` defaults to `false` for safety. Set `auto_send:=true` only when the
Nav2 action server is ready and automatic waypoint execution is intended.

## Controller start/stop interface

controller publishes start and stop messages. Each node listens to its
controller topics, enables or disables its normal data processing, and publishes
its current status.

| Node | Start topic | Stop topic | Status topic |
| --- | --- | --- | --- |
| `fake_boundary_filter_node` | `/boundary_filter_start` | `/boundary_filter_stop` | `/boundary_filter_status` |
| `tree_memory_node` | `/tree_memory_start` | `/tree_memory_stop` | `/tree_memory_status` |
| `tree_waypoint_planner_node` | `/tree_waypoint_start` | `/tree_waypoint_stop` | `/tree_waypoint_status` |
| `nav2_waypoint_sender_node` | `/nav2_sender_start` | `/nav2_sender_stop` | `/nav2_waypoint_status` |

All four nodes provide a `start_active` parameter with a default of `true`.
When stopped, the nodes continue listening for controller messages and publishing
status where practical, but do not publish their normal output data. The Nav2
sender does not send new goals while stopped. `auto_send` remains `false` by
default for safety.

Build the package:

```bash
cd /home/rmitaiil/Macademia-Project-PAR/Node4-Tree_Waypoint
colcon build --packages-select tree_waypoint_planner
source install/setup.bash
```

Run the stack initially stopped:

```bash
ros2 launch tree_waypoint_planner tree_waypoint_stack.launch.py \
  use_hardcoded_trees:=true \
  use_memory:=false \
  use_boundary_filter:=false \
  frame_id:=map \
  auto_send:=false \
  start_active:=false
```

Echo planner and Nav2 sender status:

```bash
ros2 topic echo /tree_waypoint_status
ros2 topic echo /nav2_waypoint_status
```

Start and stop the planner:

```bash
ros2 topic pub --once /tree_waypoint_start std_msgs/msg/Empty "{}"
ros2 topic pub --once /tree_waypoint_stop std_msgs/msg/Empty "{}"
```

Start and stop the Nav2 sender:

```bash
ros2 topic pub --once /nav2_sender_start std_msgs/msg/Empty "{}"
ros2 topic pub --once /nav2_sender_stop std_msgs/msg/Empty "{}"
```

## ROS2 Topics

The `tree_waypoint_planner` package uses ROS2 topics to connect the tree detection, waypoint planning, RViz visualisation, and future Nav2 handoff parts of the system.

### Published Topics

The main waypoint planner node publishes:

* `/tree_markers` (`visualization_msgs/msg/MarkerArray`)
  RViz markers showing the detected or hard-coded tree positions as green cylinders.

* `/tree_waypoint_markers` (`visualization_msgs/msg/MarkerArray`)
  RViz markers showing the generated approach waypoints as orange arrows.

* `/tree_waypoints` (`geometry_msgs/msg/PoseArray`)
  A list of all generated approach waypoints, with one waypoint created for each tree.

* `/selected_tree_waypoint` (`geometry_msgs/msg/PoseStamped`)
  The currently selected target waypoint. This is the single waypoint intended to be sent to Nav2.

* `/selected_tree_waypoint_marker` (`visualization_msgs/msg/Marker`)
  RViz marker showing the currently selected waypoint more clearly.

* `/tree_waypoint_status` (`std_msgs/msg/String`)
  Planner state updates such as detected tree count, generated waypoint count, selected index, visited-index events, and completion.

The Nav2 sender node publishes:

* `/nav2_waypoint_status` (`std_msgs/msg/String`)
  A simple status topic showing whether the Nav2 sender is waiting, has received a waypoint, has sent a goal, or has completed/failed a goal.

* `/reached_tree_waypoint` (`std_msgs/msg/Bool`)
  Publishes `True` when Nav2 successfully reaches the selected tree waypoint. This can later be used to trigger the tree spiral behaviour.

### Subscribed Topics

The waypoint planner node subscribes to:

* `/detected_trees` (`geometry_msgs/msg/PoseArray`)
  Tree positions from the detection system. For testing, this can also come from the fake tree publisher node.

* `/mark_tree_visited` (`std_msgs/msg/Int32`)
  Marks a nonnegative waypoint index as visited.

* `/reset_visited_trees` (`std_msgs/msg/Empty`)
  Clears all visited waypoint indices.

The Nav2 sender node subscribes to:

* `/selected_tree_waypoint` (`geometry_msgs/msg/PoseStamped`)
  The selected waypoint from the planner, which can be forwarded to Nav2 when `auto_send` is enabled.

## Tree Memory Node

`tree_memory_node` sits between the raw tree mapper and the waypoint planner. It remembers tree detections over time, merges detections within a configurable distance, and smooths repeated observations so the planner receives a stable tree list instead of noisy raw positions.

### Topics

* Subscribes to `/trees` (`geometry_msgs/msg/PoseArray`)
  Raw tree detections, including the output topic currently used by Joshua's tree mapper.

* Publishes `/detected_trees` (`geometry_msgs/msg/PoseArray`)
  Smoothed, remembered tree positions for `tree_waypoint_planner_node`.

* Publishes `/tracked_tree_markers` (`visualization_msgs/msg/MarkerArray`)
  Green-blue cylinder markers for the tracked tree positions in RViz.

The node deletes obsolete marker IDs if the published tracked-tree count decreases, preventing stale cylinders from remaining in RViz.

### Parameters

| Parameter | Default | Description |
| --- | --- | --- |
| `frame_id` | `odom` | Frame used for output poses and markers. |
| `merge_distance` | `0.4` | Maximum Euclidean distance for merging a detection into a tracked tree. |
| `smoothing_alpha` | `0.5` | Weight applied to each new detection during position smoothing. |
| `min_observations` | `1` | Observations required before publishing a tracked tree. |
| `publish_rate_hz` | `1.0` | Output publication rate. |
| `marker_radius` | `0.18` | RViz cylinder radius in metres. |
| `marker_height` | `0.6` | RViz cylinder height in metres. |

### Run

Build and source the workspace:

```bash
colcon build --packages-select tree_waypoint_planner
source install/setup.bash
```

Start the complete fake-detection test in separate terminals:

```bash
# Terminal 1: publish fake detections on the raw /trees input
ros2 run tree_waypoint_planner fake_tree_publisher_node --ros-args -r /detected_trees:=/trees
```

```bash
# Terminal 2: remember, merge, and smooth the raw detections
ros2 run tree_waypoint_planner tree_memory_node
```

```bash
# Terminal 3: consume the tracked tree output
ros2 run tree_waypoint_planner tree_waypoint_planner_node --ros-args -p use_hardcoded_trees:=false
```

```bash
# Terminal 4: visualise the pipeline
rviz2
```

In RViz, set `Fixed Frame` to `odom` and add:

* `MarkerArray` on `/tracked_tree_markers`
* `MarkerArray` on `/tree_markers`
* `MarkerArray` on `/tree_waypoint_markers`
* `Marker` on `/selected_tree_waypoint_marker`

Verify the tracked output from another terminal:

```bash
ros2 topic echo /detected_trees
```

## Visited Waypoint Tracking

In `nearest` selection mode, the planner selects the nearest unvisited waypoint by default. Visited waypoint arrows are grey in `/tree_waypoint_markers`, while unvisited arrows remain orange and the selected waypoint marker remains blue.

`index` mode can still select visited waypoints and remains useful for testing a specific waypoint. Set `ignore_visited:=true` to restore the previous nearest-waypoint behaviour:

```bash
ros2 run tree_waypoint_planner tree_waypoint_planner_node --ros-args -p ignore_visited:=true
```

Run the planner:

```bash
ros2 run tree_waypoint_planner tree_waypoint_planner_node
```

Mark waypoint index 0 visited:

```bash
ros2 topic pub --once /mark_tree_visited std_msgs/msg/Int32 "{data: 0}"
```

Reset all visited indices:

```bash
ros2 topic pub --once /reset_visited_trees std_msgs/msg/Empty "{}"
```

Echo planner status:

```bash
ros2 topic echo /tree_waypoint_status
```

## Fake Orchard Boundary Filter Node

`fake_boundary_filter_node` filters the full occupancy grid map to a rectangular fake orchard area. It exists so lab clutter outside the test orchard area, such as table legs, chairs, walls, or other random obstacles, can be hidden from Joshua's tree detection without changing the detector or waypoint planner.

### Topics

* Subscribes to `/map` (`nav_msgs/msg/OccupancyGrid`)
  Full occupancy grid map.

* Publishes `/filtered_map` (`nav_msgs/msg/OccupancyGrid`)
  Copy of the latest map where cells outside the configured rectangle are replaced with `outside_value`.

* Publishes `/orchard_boundary_marker` (`visualization_msgs/msg/Marker`)
  Yellow `LINE_STRIP` rectangle showing the active fake orchard boundary in RViz.

Joshua's tree detection should subscribe to `/filtered_map` instead of `/map` when running in the lab fake-orchard setup.

### Parameters

| Parameter | Default | Description |
| --- | --- | --- |
| `input_map_topic` | `/map` | Occupancy grid topic to filter. |
| `output_map_topic` | `/filtered_map` | Filtered occupancy grid output topic. |
| `frame_id` | `map` | Marker frame used before a map has been received. |
| `min_x` | `0.0` | Minimum rectangle x coordinate in map frame metres. |
| `max_x` | `5.0` | Maximum rectangle x coordinate in map frame metres. |
| `min_y` | `-2.0` | Minimum rectangle y coordinate in map frame metres. |
| `max_y` | `2.0` | Maximum rectangle y coordinate in map frame metres. |
| `outside_value` | `0` | Occupancy value written to cells outside the rectangle. |
| `publish_rate_hz` | `1.0` | Rate for publishing the filtered map and boundary marker. |

### Build

```bash
colcon build --packages-select tree_waypoint_planner
source install/setup.bash
```

### Run

```bash
ros2 run tree_waypoint_planner fake_boundary_filter_node --ros-args \
  -p min_x:=0.0 \
  -p max_x:=5.0 \
  -p min_y:=-2.0 \
  -p max_y:=2.0 \
  -p outside_value:=0
```

### Check Topics

```bash
ros2 topic echo /filtered_map --once
ros2 topic list | grep map
```

### RViz

Set `Fixed Frame` to `map` and add:

* `Map` on `/map`
* `Map` on `/filtered_map`
* `Marker` on `/orchard_boundary_marker`


