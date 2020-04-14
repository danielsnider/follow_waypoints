# follow_waypoints [![Build Status](http://build.ros.org/buildStatus/icon?job=Kbin_uX64__follow_waypoints__ubuntu_xenial_amd64__binary)](http://build.ros.org/job/Kbin_uX64__follow_waypoints__ubuntu_xenial_amd64__binary)

A package that will buffer `move_base` goals until instructed to navigate to all waypoints in sequence.

![follow_waypoints](readme_images/magni_follow_waypoint.gif "rviz")


```
rosrun follow_waypoints follow_waypoints.py
 
```

to save the path follow the usual

```
rostopic pub /path_ready std_msgs/Empty -1
```

to load a previously save path

```
rostopic pub /start_journey std_msgs/Empty -1
```