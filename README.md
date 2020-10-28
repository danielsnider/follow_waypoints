# follow_waypoints [![Build Status](http://build.ros.org/buildStatus/icon?job=Kbin_uX64__follow_waypoints__ubuntu_xenial_amd64__binary)](http://build.ros.org/job/Kbin_uX64__follow_waypoints__ubuntu_xenial_amd64__binary)

A package that will buffer `move_base` goals until instructed to navigate to all
waypoints in sequence.

| ![follow_waypoints](https://github.com/danielsnider/follow_waypoints/blob/master/readme_images/follow_waypoints_rviz.png "rviz") | ![follow_waypoints](readme_images/follow_waypoint.gif "rviz") |

This is a modified and improved version of the upstream follow_waypoints
package. Here are some of the changes compared to the upstream version:

* Removal of the smach dependency - State machine is not needed
* Implement simpler design, where new goals are added to a FIFO and they
  get poped out, and passed to move_base. This allows adding additional
  waypoints during the actual navigation run
* Removal of the redundant journey_* logic, reading/writing to files
  of the repo
* Allow for specifying the waypoints as either poses (using the "Pose
  Estimation" button from RViz) or by using the "Publish Point"
  functionality. In case of the latter, a default (0, 0, 0, 1)
  quaternion orientation is assumed. In the future, we could aslo use
  an orientation based on the vector connecting previous and current
  waypoint.
* Add ROS Parameters for most of the settings in the algorithm.
* Reformat code using isort and black.
* Make it `python3`-compatible - see print statements.
* `path_ready`, `path_reset are now one-shot std_srvs::Trigger services.
* `patrol_mode` dynamic-reconfigurable flag to follow the given set of
   waypoints over and over again until a `/path_reset` is given

## Installation

```bash
sudo apt-get install ros-$(rosversion -d)-follow-waypoints
```

Or alternatively`, to get the latest version and features, clone this repo and
compile it as part of your ROS workspace

**Documentation on wiki: [http://wiki.ros.org/follow_waypoints](http://wiki.ros.org/follow_waypoints)**

## Execution

```bash
rosrun follow_waypoints follow_waypoints
# Or alternatively
roslaunch follow_waypoints follow_waypoints
```

### New features (not documented on wiki):

#### Wait_duration parameter.

This sets wait duration in between waypoints. The default value is set to 0.0 sec.

```
rosparam set wait_duration 5.0
```

#### Distance threshold parameter.

Issue the next goal target if the robot reaches within this distance. This has
the effect of smoothing motion and not stopping at each waypoint. The default
value is set to 0.0 distance which disables the feature.

```
rosparam set waypoint_distance_tolerance 0.5
```
