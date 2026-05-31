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
