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

The Nav2 sender node publishes:

* `/nav2_waypoint_status` (`std_msgs/msg/String`)
  A simple status topic showing whether the Nav2 sender is waiting, has received a waypoint, has sent a goal, or has completed/failed a goal.

* `/reached_tree_waypoint` (`std_msgs/msg/Bool`)
  Publishes `True` when Nav2 successfully reaches the selected tree waypoint. This can later be used to trigger the tree spiral behaviour.

### Subscribed Topics

The waypoint planner node subscribes to:

* `/detected_trees` (`geometry_msgs/msg/PoseArray`)
  Tree positions from the detection system. For testing, this can also come from the fake tree publisher node.

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
