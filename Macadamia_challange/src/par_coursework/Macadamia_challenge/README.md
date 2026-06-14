# Macadamia Challenge

ROS 2 Python package for the macadamia orchard waypoint and tree-behaviour demo.

The package combines:

- waypoint generation around detected or hard-coded trees
- occupancy-map tree detection
- optional fake tree publishing for testing
- optional tree memory for smoothing repeated detections
- optional fake orchard boundary map filtering
- Nav2 waypoint sending
- an orchard mission state controller
- a spiral steering behaviour around each selected tree

## Package Layout

```text
macadamia_challenge/
  controll/
    orchard_state_controller.py
  tree_behaviour/
    spiral_controller.py
    spiral_controller_nav2_waypoint.py
  tree_detection/
    tree_mapper_node.py
  waypoint_provider/
    fake_boundary_filter_node.py
    fake_tree_publisher_node.py
    nav2_waypoint_sender_node.py
    tree_memory_node.py
    tree_waypoint_planner_node.py
```

## Build

From the workspace containing `src/par_coursework/Macadamia_challenge`:

```bash
colcon build --packages-select macadamia_challenge
source install/setup.bash
```

## Universal Launch

There is one launch file:

```bash
ros2 launch macadamia_challenge macadamia_challange.launch.py
```

By default, it starts the main mission components:

- `tree_waypoint_planner_node`
- `nav2_waypoint_sender_node`
- `orchard_control_node`
- `spiral_controller` in basic steering mode

Optional test/helper nodes can be enabled with launch arguments:

```bash
ros2 launch macadamia_challenge macadamia_challange.launch.py \
  use_fake_trees:=true \
  use_boundary_filter:=true \
  use_memory:=true
```

To use the occupancy-map tree detector instead of hard-coded trees:

```bash
ros2 launch macadamia_challenge macadamia_challange.launch.py \
  use_tree_mapper:=true \
  use_hardcoded_trees:=false
```

To route detected trees through the memory node first:

```bash
ros2 launch macadamia_challenge macadamia_challange.launch.py \
  use_tree_mapper:=true \
  use_memory:=true \
  use_hardcoded_trees:=false \
  tree_mapper_output_topic:=/trees
```

The spiral behaviour defaults to direct steering on `/cmd_vel`. To use the Nav2 waypoint spiral instead:

```bash
ros2 launch macadamia_challenge macadamia_challange.launch.py \
  spiral_mode:=nav2
```

For a waypoint-stack-only test without the orchard controller or spiral controller:

```bash
ros2 launch macadamia_challenge macadamia_challange.launch.py \
  use_orchard_controller:=false \
  use_spiral_controller:=false \
  use_memory:=true \
  use_boundary_filter:=true \
  use_hardcoded_trees:=false
```

## Launch Arguments

| Argument | Default | Purpose |
| --- | --- | --- |
| `frame_id` | `odom` | Frame used by published poses and markers. |
| `use_fake_trees` | `false` | Launches hard-coded fake tree publisher. |
| `use_tree_mapper` | `false` | Launches occupancy-map tree detector. |
| `use_boundary_filter` | `false` | Launches fake rectangular occupancy-map boundary filter. |
| `use_memory` | `false` | Launches tree memory node for smoothing `/trees` into `/detected_trees`. |
| `use_nav2_sender` | `true` | Launches Nav2 waypoint sender. |
| `use_orchard_controller` | `true` | Launches mission state controller. |
| `use_spiral_controller` | `true` | Launches spiral tree-behaviour controller. |
| `start_active` | `true` | Starts planner/helper nodes active. |
| `nav2_start_active` | `false` | Starts Nav2 sender active. Usually controlled by orchard controller. |
| `auto_start` | `false` | Starts orchard mission automatically. |
| `nav2_auto_send` | `true` | Allows Nav2 sender to forward selected waypoint goals. |
| `use_hardcoded_trees` | `true` | Planner uses built-in tree positions instead of `/detected_trees`. |
| `waypoint_mode` | `towards_centerline` | Waypoint generation mode. |
| `waypoint_offset_x` | `0.0` | X offset used in fixed-offset waypoint mode. |
| `waypoint_offset_y` | `-0.6` | Y offset used in fixed-offset waypoint mode. |
| `centreline_y` | `0.0` | Orchard row centreline used by approach waypoint logic. |
| `approach_distance` | `0.6` | Distance from tree to generated approach waypoint. |
| `min_x`, `max_x`, `min_y`, `max_y` | `0.0`, `5.0`, `-2.5`, `2.5` | Rectangle used by fake boundary filter. |
| `outside_value` | `0` | Occupancy value written outside the fake boundary. |
| `tree_mapper_input_map_topic` | `/map` | Occupancy grid input for tree detection. |
| `tree_mapper_output_topic` | `/detected_trees` | Tree detector output topic. Use `/trees` when routing through tree memory. |
| `tree_mapper_min_radius` | `0.05` | Minimum detected contour radius in metres. |
| `tree_mapper_max_radius` | `0.5` | Maximum detected contour radius in metres. |
| `spiral_min_radius` | `0.25` | Starting spiral radius around a tree. |
| `spiral_max_radius` | `1.4` | Radius where spiral behaviour finishes. |
| `spiral_loop_spacing` | `1.0` | Spiral loop spacing in robot-width units. |
| `spiral_mode` | `steering` | Spiral implementation: `steering` for direct `/cmd_vel`, or `nav2` for Nav2 waypoint batches. |
| `spiral_linear_speed` | `0.125` | Linear speed during spiral steering. |
| `spiral_kp_heading` | `1.5` | Heading proportional gain for spiral steering. |
| `spiral_nav2_action_name` | `navigate_through_poses` | Nav2 action used by the Nav2 spiral. |
| `spiral_nav2_goal_frame` | `map` | Goal frame used by the Nav2 spiral. |
| `spiral_nav2_odom_topic` | `/odometry/filtered` | Odometry topic used by the Nav2 spiral. |
| `spiral_nav2_waypoint_spacing` | `0.35` | Spacing between generated Nav2 spiral waypoints. |
| `spiral_nav2_batch_size` | `8` | Number of Nav2 spiral waypoints sent per batch. |

## Main Nodes

### `tree_waypoint_planner_node`

Creates approach waypoints for each tree and selects the next unvisited waypoint.

Publishes:

- `/tree_markers`
- `/tree_waypoint_markers`
- `/tree_waypoints`
- `/selected_tree_waypoint`
- `/selected_tree_waypoint_marker`
- `/tree_waypoint_status`

Subscribes:

- `/detected_trees`
- `/mark_tree_visited`
- `/reset_visited_trees`
- `/tree_waypoint_start`
- `/tree_waypoint_stop`

### `nav2_waypoint_sender_node`

Sends the selected tree waypoint to Nav2 using `/navigate_to_pose`.

Publishes:

- `/nav2_waypoint_status`
- `/reached_tree_waypoint`

Subscribes:

- `/selected_tree_waypoint`
- `/nav2_sender_start`
- `/nav2_sender_stop`

### `orchard_control_node`

Runs the mission state machine:

1. detect or collect trees
2. plan a tree waypoint
3. navigate to the selected tree
4. start spiral tree behaviour
5. mark the tree visited
6. repeat until all trees are visited
7. return home

Publishes controller topics such as:

- `/tree_memory_start`
- `/tree_waypoint_start`
- `/nav2_sender_start`
- `/start_spiral`
- `/stop_spiral`
- `/mark_tree_visited`
- `/orchard_controller/state`

It subscribes to `/spiral_done` and moves to the next tree only after the spiral controller reports that it has reached its configured maximum radius.

Start the mission manually with:

```bash
ros2 topic pub --once /orchard_controller/start std_msgs/msg/Empty "{}"
```

### `spiral_controller`

Runs a simple steering spiral around the selected tree. It is the default when `spiral_mode:=steering`. It is started by the orchard controller on `/start_spiral` and stopped on `/stop_spiral`.

Publishes:

- `/cmd_vel`
- `/spiral_markers`
- `/spiral_done`

Subscribes:

- `/odometry/filtered`
- `/start_spiral`
- `/stop_spiral`

### `spiral_nav2_controller`

Runs the spiral by sending batches of poses to Nav2 `NavigateThroughPoses`. Enable it with `spiral_mode:=nav2`.

Publishes:

- `/spiral_markers`
- `/spiral_done`

Subscribes:

- `/odometry/filtered`
- `/start_spiral`
- `/stop_spiral`

### `tree_memory_node`

Stores and smooths tree detections. It subscribes to raw tree poses on `/trees`, merges nearby detections, and republishes stable tree poses on `/detected_trees`.

Publishes:

- `/detected_trees`
- `/tracked_tree_markers`
- `/tree_memory_status`

### `tree_mapper_node`

Detects tree-like circular occupied regions from an occupancy grid map. The orchard controller triggers it on `/tree_generator_start` during the detection state.

Subscribes:

- `/map` by default, configurable with `tree_mapper_input_map_topic`
- `/tree_generator_start`

Publishes:

- `/detected_trees` by default, configurable with `tree_mapper_output_topic`

### `fake_tree_publisher_node`

Publishes hard-coded tree positions for testing.

Publishes:

- `/detected_trees`

### `fake_boundary_filter_node`

Filters an occupancy grid to a rectangular fake orchard area. It is useful for lab testing when objects outside the orchard area should be hidden from a separate detector.

Subscribes:

- `/map`

Publishes:

- `/filtered_map`
- `/orchard_boundary_marker`
- `/boundary_filter_status`

## Important Topics

| Topic | Type | Purpose |
| --- | --- | --- |
| `/trees` | `geometry_msgs/msg/PoseArray` | Raw tree detections, usually for `tree_memory_node`. |
| `/detected_trees` | `geometry_msgs/msg/PoseArray` | Tree list consumed by the waypoint planner. |
| `/selected_tree_waypoint` | `geometry_msgs/msg/PoseStamped` | Current waypoint sent to Nav2. |
| `/reached_tree_waypoint` | `std_msgs/msg/Bool` | Nav2 sender reports successful arrival. |
| `/start_spiral` | `std_msgs/msg/Float32MultiArray` | Starts spiral around `[center_x, center_y, min_radius, max_radius, loop_spacing]`. |
| `/stop_spiral` | `std_msgs/msg/Empty` | Stops spiral motion. |
| `/spiral_done` | `std_msgs/msg/Empty` | Spiral controller reports that tree behaviour has finished. |
| `/cmd_vel` | `geometry_msgs/msg/TwistStamped` | Velocity command from spiral controller. |

