# 1. Topics: 
* /behavior_server/transition_event
* /behavior_tree_log
* /bond
* /bt_navigator/transition_event
* /cmd_vel
* /cmd_vel_nav
* /cmd_vel_smoothed
* /cmd_vel_teleop
* /collision_monitor/collision_points_marker
* /collision_monitor/transition_event
* /collision_monitor_state
* /controller_server/transition_event
* /curvature_lookahead_point
* /detected_dock_pose
* /diagnostics
* /dock_pose
* /docking_server/transition_event
* /docking_trajectory
* /filtered_dock_pose
* /global_costmap/costmap
* /global_costmap/costmap_raw
* /global_costmap/costmap_raw_updates
* /global_costmap/costmap_updates
* /global_costmap/global_costmap/transition_event
* /global_costmap/obstacle_layer
* /global_costmap/obstacle_layer_raw
* /global_costmap/obstacle_layer_raw_updates
* /global_costmap/obstacle_layer_updates
* /global_costmap/published_footprint
* /global_costmap/static_layer
* /global_costmap/static_layer_raw
* /global_costmap/static_layer_raw_updates
* /global_costmap/static_layer_updates
* /imu/data
* /is_rotating_to_heading
* /joint_states
* /joy
* /joy/set_feedback
* /local_costmap/clearing_endpoints
* /local_costmap/costmap
* /local_costmap/costmap_raw
* /local_costmap/costmap_raw_updates
* /local_costmap/costmap_updates
* /local_costmap/local_costmap/transition_event
* /local_costmap/published_footprint
* /local_costmap/voxel_grid
* /local_costmap/voxel_layer
* /local_costmap/voxel_layer_raw
* /local_costmap/voxel_layer_raw_updates
* /local_costmap/voxel_layer_updates
* /lookahead_collision_arc
* /lookahead_point
* /map
* /map_metadata
* /move_command
* /navigate_to_pose
* /odom
* /odom/unfiltered
* /parameter_events
* /plan
* /plan_smoothed
* /planner_server/transition_event
* /pose
* /position_command
* /preempt_teleop
* /received_global_plan
* /robot_description
* /robot_status
* /rosout
* /scan
* /set_pose
* /slam_toolbox/feedback
* /slam_toolbox/graph_visualization
* /slam_toolbox/scan_visualization
* /slam_toolbox/transition_event
* /slam_toolbox/update
* /smoother_server/transition_event
* /staging_pose
* /start_info
* /tf
* /tf_static
* /velocity_smoother/transition_event
* /waypoint_follower/transition_event

# 2. Communicatie tussen Nodes

De communicatie tussen de nodes verloopt via ROS topics en actions: 
```
Topic	   /move_command	       Server → nav_goal_node	                      Start navigatie
Action	   /navigate_to_pose	   nav_goal_node → Nav2	                          Robot laten navigeren
Topic	   /robot_status	       nav_goal_node → robot_status_node	          Statusupdates
Topic	   /position_command	   camera_detection_node → arm_control_node	      Markerpositie
Topic	   /start_info	           Server → start_info_node	                      Initialisatie
Topic      /robot_status           nav_goal_node → robot_status_node              servo_control_node 
```

| Topic / Action      | Type / Message                       | Richting                          | Inhoud / Payload                                           | Doel                                     |
|--------------------|-------------------------------------|----------------------------------|------------------------------------------------------------|-----------------------------------------|
| /move_command       | std_msgs/String                      | Server → nav_goal_node           | Naam van locatie.                 | Start navigatie                          |
| /navigate_to_pose   | nav2_msgs/Action/NavigateToPose      | nav_goal_node → Nav2             | Pose (x, y, θ) van het doel                                | Laat robot navigeren                     |
| /robot_status       | linorobot2_interfaces/RobotStatus    | nav_goal_node → robot_status_node, servo_control_node | robot_id, status, description, target_node | Statusupdates voor fleet manager en servo |
| /position_command   | std_msgs/String                      | camera_detection_node → arm_control_node | Marker ID, x, y, z, knopstatus                              | Arm positionering en liftknop status    |
| /start_info         | std_msgs/String                      | Server → start_info_node         | Liftbutton, DestinationPoint                                | Initialisatie                            |


# 3. Multi-robot communicatie


## 3.1. Scheiding via ROS namespaces. 

Elke robot draait in zijn eigen namespace, bijvoorbeeld:

```
/robot11/

/robot12/

/robot13/
```

Dit betekent dat alle topics en nodes per robot uniek zijn en niet met elkaar conflicteren.

Voorbeeld: /robot11/move_command en /robot12/move_command zijn volledig onafhankelijk van elkaar.


## 3.2 Eigen topics per robot

Robot13 heeft een eigen set van topic en nodes: arm_control_node, camera_detection_node en het topic /position_command. 


## 3.3 Communicatie via de Fleet Manager / server

De fleet manager communiceert met robots door de namespace in de topicnaam te gebruiken.

```
Voor robot11: /robot11/move_command
Voor robot12: /robot12/move_command
```

De fleet manager kan dus per robot opdrachten sturen, en ontvangt status updates via /robotXX/robot_status.


## 3.4 Inter-robot communicatie

In de huidige implementatie bestaat er geen directe communicatie tussen robots. Robots delen geen topics zoals status of posities rechtstreeks met elkaar. Alle coördinatie verloopt via de fleet manager. Dit is mogelijk doordat er zenoh-bridges lopen van elke robot tot de fleet manager. 

Zo kunnen robots autonoom reageren op elkaar zonder directe communicatie tussen elkaar. 


Voor voorbeeld output vanuit de topics die worden gebruikt in de nodes kun je dit ook lezen: 
- [ROS Netwerkoverzicht
](https://github.com/Avansuss/placeholder/wiki/ROS-Netwerkoverzicht)