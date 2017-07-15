# follow_waypoints [![Build Status](http://build.ros.org/buildStatus/icon?job=Kbin_uX64__follow_waypoints__ubuntu_xenial_amd64__binary)](http://build.ros.org/job/Kbin_uX64__follow_waypoints__ubuntu_xenial_amd64__binary)

A package that will buffer `move_base` goals until instructed to navigate to all waypoints in sequence.

Full documentation on wiki: [http://wiki.ros.org/follow_waypoints](http://wiki.ros.org/follow_waypoints)

![follow_waypoints](https://github.com/danielsnider/follow_waypoints/blob/master/readme_images/follow_waypoints_rviz.png "rviz")

## Installation

```
  $ sudo apt-get install ros-kinetic-follow-waypoints
```

## Usage

To set waypoints you can either publish a ROS `PoseWithCovarianceStamped` message to the `/initialpose` topic directly or use RViz’s tool "2D Pose Estimate" to click anywhere. To visualize the waypoints as pink arrows in RViz, configure RViz to display the topic `/current_waypoints` which is published by `follow_waypoints` and must be subscribed to in Rviz as a `PoseAarray` type.

To initiate waypoint following send a "path ready" message.

```
  $ rostopic pub /path_ready std_msgs/Empty -1
```

