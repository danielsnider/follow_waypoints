# follow_waypoints

A package that will buffer move_base goals until instructed to navigate to them in sequence.

Wiki page: [http://wiki.ros.org/follow_waypoints](http://wiki.ros.org/follow_waypoints)

![follow_waypoints](https://github.com/danielsnider/follow_waypoints/blob/master/readme_images/follow_waypoints_rviz.gif "rviz")

## Quick Start

**Install:**

```
  $ sudo apt-get install ros-kinetic-rover-follow-waypoints #(package WIP)
```

**Run:**

```
  $ roslaunch rover_follow_waypoints follow_waypoints.py
```

## Usage

To set waypoints you can either publish a ROS Pose message to the /initialpose topic directly or use RViz’s tool “2D Pose Estimate” to click anywhere. Figure 3 shows the resulting pink arrows representing the current waypoints in RViz. To visualize the waypoints in this way the topic /current_waypoints is published by follow_waypoints.py and must be subscribed to in Rviz as a PoseAarray type.

To initiate waypoint following send a “path ready” message.

```
  $ rostopic pub /path_ready std_msgs/Empty -1
```

### Normal Output

```bash
  $ roslaunch rover_follow_waypoints follow_waypoints.py
  [  INFO ] : State machine starting in initial state 'GET_PATH' with userdata: ['waypoints']
  [INFO] [1497201258.613819, 48756.024000]: Waiting to receive waypoints via Pose msg on topic /initialpose
  [INFO] [1497201258.614702, 48756.024000]: To start following waypoints: 'rostopic pub /path_ready 
  std_msgs/Empty -1'
  [INFO] [1497201258.614718, 48756.024000]: To cancel the goal: 'rostopic pub -1 /move_base/cancel actionlib_msgs/GoalID -- {}'
  [INFO] [1497201273.182585, 48763.755000]: Received new waypoint
  [INFO] [1497201278.287493, 48766.496000]: Received new waypoint
  [INFO] [1497201307.641070, 48783.387000]: Received path ready message
  [  INFO ] : State machine transitioning 'GET_PATH':'success'-->'FOLLOW_PATH'
  [INFO] [1497201307.973766, 48783.591000]: Executing move_base goal to position (x,y): 0.0123248100281, -0.0620594024658
  [INFO] [1497201309.647665, 48784.708000]: Executing move_base goal to position (x,y): -0.0924506187439, -0.0527720451355
  [  INFO ] : State machine transitioning 'FOLLOW_PATH':'success'-->'PATH_COMPLETE'
  [INFO] [1497201310.888334, 48785.509000]: ###############################
  [INFO] [1497201310.888826, 48785.509000]: ##### REACHED FINISH GATE #####
  [INFO] [1497201310.889255, 48785.509000]: ###############################
  [  INFO ] : State machine transitioning 'PATH_COMPLETE':'success'-->'GET_PATH'
  [INFO] [1497201310.891560, 48785.514000]: Waiting to receive waypoints via Pose msg on topic /initialpose
  [INFO] [1497201310.894310, 48785.514000]: To start following waypoints: 'rostopic pub /path_ready std_msgs/Empty -1'
```

## ROS API

### Published Topics

`waypoints` (geometry_msgs/PoseArray)  
  Message containing all waypoints in the queue. This is useful for displaying waypoints in RViz.

### Subscribed Topics

`initialpose` (geometry_msgs/PoseWithCovarianceStamped)  
  Message containing one waypoint. RViz can be configured to publish clicks as this message type.

`path_ready` (std_msgs/Empty)  
  Message that will initiate waypoint following.

### Parameters

`~goal_frame_id` (`string`, default: "`map`")  
  The tf frame for move_base goals.

## Usage in the University Rover Competition (URC)

Waypoint following could find usefulness at URC in the following ways:

- To search a variety of locations, ideally faster than by teleoperation
- To allow for assisted autonomous obstacle avoidance where an obstacle is known to fail detection
- To navigate to multiple goals in the autonomous task with a single command (use in combination with GPS goals)

## Detailed Description

The follow_waypoints package uses actionlib to send the goals to move_base.

The code for follow_waypoints.py is structured as a barebones state machine. For this reason it is easy to add to the script complex behavior controlled by defined transitions (ie. a state machine). For modifying the script to be an easy task, you should learn about the Python state machine library in ROS called smach. The state transitions in the script occur in the order GET_PATH, FOLLOW_PATH, and PATH_COMPLETE and then they repeat.

## Feature Wishlist

- Add an action to take if a move_base goal fails or exceeds a maximum duration or distance from goal

## Known Issues

- Currently the only way to cancel waypoint following is to use CTRL+C on this node and cancel the current move_base goal with `rostopic pub -1 /move_base/cancel actionlib_msgs/GoalID -- {}`. The plan is to listen on a topic for a message to preempt and cancel waypoint following.