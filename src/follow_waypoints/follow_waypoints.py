#!/usr/bin/env python

import threading
import rospy
import actionlib

from smach import State,StateMachine
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseArray
from std_msgs.msg import Empty


class FollowPath(State):
    def __init__(self):
        State.__init__(self, outcomes=['success'], input_keys=['waypoints'])
        self.frame_id = rospy.get_param('~goal_frame_id','map')
        # Get a move_base action client
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo('Connecting to move_base...')
        self.client.wait_for_server()
        rospy.loginfo('Connected to move_base.')

    def execute(self, userdata):
        # Execute waypoints each in sequence
        for waypoint in userdata.waypoints:
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = self.frame_id
            goal.target_pose.pose.position = waypoint.pose.pose.position
            goal.target_pose.pose.orientation = waypoint.pose.pose.orientation
            rospy.loginfo('Executing move_base goal to position (x,y): %s, %s' %
                    (waypoint.pose.pose.position.x, waypoint.pose.pose.position.y))
            rospy.loginfo("To cancel the goal: 'rostopic pub -1 /move_base/cancel actionlib_msgs/GoalID -- {}'")
            self.client.send_goal(goal)
            self.client.wait_for_result()
        return 'success'

def convert_PoseWithCovArray_to_PoseArray(waypoints):
    """Used to publish waypoints as pose array so that you can see them in rviz, etc."""
    poses = PoseArray()
    poses.header.frame_id = 'map'
    poses.poses = [pose.pose.pose for pose in waypoints]
    return poses

class GetPath(State):
    def __init__(self):
        State.__init__(self, outcomes=['success'], input_keys=['waypoints'], output_keys=['waypoints'])
        # Create publsher to publish waypoints as pose array so that you can see them in rviz, etc.
        self.poseArray_publisher = rospy.Publisher('/waypoints', PoseArray, queue_size=1)

    def execute(self, userdata):
        waypoints = []
        self.path_ready = False

        # Start thread to listen for when the path is ready (this function will end then)
        def wait_for_path_ready():
            """thread worker function"""
            data = rospy.wait_for_message('/path_ready', Empty)
            rospy.loginfo('Recieved path ready message')
            self.path_ready = True
        t = threading.Thread(target=wait_for_path_ready)
        t.start()

        # Wait for published waypoints
        topic = "/initialpose"
        rospy.loginfo("Waiting to recieve waypoints via Pose msg on topic %s" % topic)
        rospy.loginfo("To start following waypoints: 'rostopic pub /path_ready std_msgs/Empty -1'")
        while not self.path_ready:
            try:
                pose = rospy.wait_for_message(topic, PoseWithCovarianceStamped, timeout=1)
            except rospy.ROSException as e:
                if 'timeout exceeded' in e.message:
                    continue
                else:
                    raise e
            rospy.loginfo("Recieved new waypoint")
            waypoints.append(pose)
            self.poseArray_publisher.publish(convert_PoseWithCovArray_to_PoseArray(waypoints))
            userdata.waypoints = waypoints

        return 'success'

class PathComplete(State):
    def __init__(self):
        State.__init__(self, outcomes=['success'])

    def execute(self, userdata):
        rospy.loginfo('###############################')
        rospy.loginfo('##### REACHED FINISH GATE #####')
        rospy.loginfo('###############################')
        return 'success'

def main():
    rospy.init_node('follow_waypoints')

    sm = StateMachine(outcomes=['success'])
    sm.userdata.waypoints = []

    with sm:
        StateMachine.add('GET_PATH', GetPath(),
                           transitions={'success':'FOLLOW_PATH'},
                           remapping={'waypoints':'waypoints'})
        StateMachine.add('FOLLOW_PATH', FollowPath(),
                           transitions={'success':'PATH_COMPLETE'},
                           remapping={'waypoints':'waypoints'})
        StateMachine.add('PATH_COMPLETE', PathComplete(),
                           transitions={'success':'GET_PATH'})

    outcome = sm.execute()
