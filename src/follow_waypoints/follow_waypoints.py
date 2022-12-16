#!/usr/bin/env python

import threading
import rospy
import actionlib
from smach import State,StateMachine
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseArray ,PointStamped
from std_msgs.msg import Empty
from tf import TransformListener
import tf
import math
import rospkg
import csv
import time
from geometry_msgs.msg import PoseStamped
import dynamic_reconfigure.client

import yaml 
import numpy as np
import os
import csv

# change Pose to the correct frame 
def changePose(waypoint,target_frame):
    if waypoint.header.frame_id == target_frame:
        # already in correct frame
        return waypoint
    if not hasattr(changePose, 'listener'):
        changePose.listener = tf.TransformListener()
    tmp = PoseStamped()
    tmp.header.frame_id = waypoint.header.frame_id
    tmp.pose = waypoint.pose.pose
    try:
        changePose.listener.waitForTransform(
            target_frame, tmp.header.frame_id, rospy.Time(0), rospy.Duration(3.0))
        pose = changePose.listener.transformPose(target_frame, tmp)
        ret = PoseWithCovarianceStamped()
        ret.header.frame_id = target_frame
        ret.pose.pose = pose.pose
        return ret
    except:
        rospy.loginfo("CAN'T TRANSFORM POSE TO {} FRAME".format(target_frame))
        exit()


#Path for saving and retreiving the pose.csv file 
output_file_path = rospkg.RosPack().get_path('follow_waypoints')+"/saved_path/pose.csv"
journey_file_path = rospkg.RosPack().get_path('follow_waypoints')+"/saved_path/pose.csv"
waypoints = []

class FollowPath(State):
    def __init__(self):
        global journey_file_path
        
        State.__init__(self, outcomes=['success'], input_keys=['waypoints'])
        self.frame_id = rospy.get_param('~goal_frame_id','map')
        self.odom_frame_id = rospy.get_param('~odom_frame_id','odom')
        self.base_frame_id = rospy.get_param('~base_frame_id','base_footprint')
        self.duration = rospy.get_param('~wait_duration', 0.0)
        # Get a move_base action client
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo('Connecting to move_base...')
        self.client.wait_for_server()
        rospy.loginfo('Connected to move_base.')
        rospy.loginfo('Starting a tf listner.')
        self.tf = TransformListener()
        self.listener = tf.TransformListener()

        journey_file_path = rospy.get_param("~journey_file_path", output_file_path)

        self.actual_xy_goal_tolerance = rospy.get_param("~xy_goal_tolerance", 0.3)
        self.actual_yaw_goal_tolerance = rospy.get_param("~yaw_goal_tolerance", 3.14)

        self.last_xy_goal_tolerance = rospy.get_param('/move_base/TebLocalPlannerROS/xy_goal_tolerance')
        self.last_yaw_goal_tolerance = rospy.get_param('/move_base/TebLocalPlannerROS/yaw_goal_tolerance')

        self.clientDR = dynamic_reconfigure.client.Client("move_base/TebLocalPlannerROS", timeout=30, config_callback=self.callbackDR)

        ########################## SAVE WAYPOINTS ############################


        foldes_waypoints = '/ws_socialdroids/src/socialdroids/robot_data/data'
        way_files = []
        pose_files = []

        # Read the waypoints from the folder
        for root, dirs, files in os.walk(foldes_waypoints):
            for file in files:
                path = os.path.join(root, file)
                name, ext = os.path.splitext(file)   

                # If the file is a waypoint, add it to the list
                if name == 'way' and ext == '.yaml':
                    way_files.append(path)

                # If the file is a pose, add it to the list
                if name == 'pose' and ext == '.csv':
                    pose_files.append(path)

        # Read the poses from the folder
        for i in range(0, len(way_files)):
            ways = []  
            path_pose = pose_files[i]
            # Read the waypoints files
            with open(way_files[i], 'r') as f:
                data = yaml.safe_load(f)
                for i in data:
                    # add the waypoints to the list
                    ways.append(i['position'])

            # Create the pose file
            with open(path_pose, 'w', newline='') as csvfile:
                csv_writer = csv.writer(csvfile, delimiter=',', quoting=csv.QUOTE_MINIMAL)

                for i in ways:
                    roll_str = str(i[0]).strip('[]')
                    pitch_str = str(i[1]).strip('[]')
                    yaw_str = str(i[2]).strip('[]')

                    roll, pitch, yaw = float(roll_str), float(pitch_str), float(yaw_str)

                    # calculate the coordinates
                    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
                    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
                    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
                    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

                    csv_writer.writerow([roll_str, pitch_str, 0.0, 0.0, 0.0, qz, qw])

        #######################################

    def callbackDR(self, config):
        rospy.loginfo("Navigation tolerance set to [xy_goal:{xy_goal_tolerance}, yaw_goal:{yaw_goal_tolerance}]".format(**config))

    def execute(self, userdata):

        self.clientDR.update_configuration({
            "xy_goal_tolerance":self.actual_xy_goal_tolerance, 
            "yaw_goal_tolerance":self.actual_yaw_goal_tolerance
            })

        global waypoints
        # Execute waypoints each in sequence
        for waypoint in waypoints:
            # Break if preempted
            if waypoints == []:
                rospy.loginfo('The waypoint queue has been reset.')
                break
            # Otherwise publish next waypoint as goal
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = self.frame_id
            goal.target_pose.pose.position = waypoint.pose.pose.position
            goal.target_pose.pose.orientation = waypoint.pose.pose.orientation
            rospy.loginfo('Executing move_base goal to position (x,y): %s, %s' %
                    (waypoint.pose.pose.position.x, waypoint.pose.pose.position.y))
            rospy.loginfo("To cancel the goal: 'rostopic pub -1 /move_base/cancel actionlib_msgs/GoalID -- {}'")
            self.client.send_goal(goal)

            self.client.wait_for_result()
            rospy.loginfo("Waiting for %f sec..." % self.duration)
            time.sleep(self.duration)

        self.clientDR.update_configuration({
            "xy_goal_tolerance":self.last_xy_goal_tolerance, 
            "yaw_goal_tolerance":self.last_yaw_goal_tolerance
            })

        return 'success'

def convert_PoseWithCovArray_to_PoseArray(waypoints):
    """Used to publish waypoints as pose array so that you can see them in rviz, etc."""
    poses = PoseArray()
    poses.header.frame_id = rospy.get_param('~goal_frame_id','map')
    poses.poses = [pose.pose.pose for pose in waypoints]
    return poses

class GetPath(State):
    def __init__(self):
        State.__init__(self, outcomes=['success'], input_keys=['waypoints'], output_keys=['waypoints'])
        # Subscribe to pose message to get new waypoints
        self.addpose_topic = rospy.get_param('~addpose_topic','/initialpose')
        # Create publsher to publish waypoints as pose array so that you can see them in rviz, etc.
        self.posearray_topic = rospy.get_param('~posearray_topic','/waypoints')
        self.poseArray_publisher = rospy.Publisher(self.posearray_topic, PoseArray, queue_size=1)

        reset_thread = threading.Thread(target=self.wait_for_path_reset)
        reset_thread.start()

    # Start thread to listen for reset messages to clear the waypoint queue
    def wait_for_path_reset(self):
        """thread worker function"""
        global waypoints
        while not rospy.is_shutdown():
            data = rospy.wait_for_message('/path_reset', Empty)
            rospy.loginfo('Recieved path RESET message')
            self.initialize_path_queue()
            rospy.sleep(3) # Wait 3 seconds because `rostopic echo` latches
                            # for three seconds and wait_for_message() in a
                            # loop will see it again.

    def initialize_path_queue(self):
        global waypoints
        waypoints = [] # the waypoint queue
        # publish empty waypoint queue as pose array so that you can see them the change in rviz, etc.
        self.poseArray_publisher.publish(convert_PoseWithCovArray_to_PoseArray(waypoints))

    def wait_for_path_ready(self):
        """thread worker function"""
        data = rospy.wait_for_message('/path_ready', Empty)
        rospy.loginfo('Recieved path READY message')
        self.path_ready = True
        with open(output_file_path, 'w') as file:
            for current_pose in waypoints:
                file.write(str(current_pose.pose.pose.position.x) + ',' + str(current_pose.pose.pose.position.y) + ',' + str(current_pose.pose.pose.position.z) + ',' + str(current_pose.pose.pose.orientation.x) + ',' + str(current_pose.pose.pose.orientation.y) + ',' + str(current_pose.pose.pose.orientation.z) + ',' + str(current_pose.pose.pose.orientation.w)+ '\n')
        rospy.loginfo('poses written to '+ output_file_path)	

    def wait_for_start_journey(self):
        """thread worker function"""
        data_from_start_journey = rospy.wait_for_message('start_journey', Empty)
        rospy.loginfo('Recieved path READY start_journey')
        with open(journey_file_path, 'r') as file:
        # with open(output_file_path, 'r') as file:
            reader = csv.reader(file, delimiter = ',')
            for row in reader:
                print (row)
                current_pose = PoseWithCovarianceStamped() 
                current_pose.pose.pose.position.x     =    float(row[0])
                current_pose.pose.pose.position.y     =    float(row[1])
                current_pose.pose.pose.position.z     =    float(row[2])
                current_pose.pose.pose.orientation.x = float(row[3])
                current_pose.pose.pose.orientation.y = float(row[4])
                current_pose.pose.pose.orientation.z = float(row[5])
                current_pose.pose.pose.orientation.w = float(row[6])
                waypoints.append(current_pose)
                self.poseArray_publisher.publish(convert_PoseWithCovArray_to_PoseArray(waypoints))
        self.start_journey_bool = True

    def execute(self, userdata):
        global waypoints
        self.initialize_path_queue()
        self.path_ready = False

        # Start thread to listen for when the path is ready (this function will end then)
        # Also will save the clicked path to pose.csv file
        ready_thread = threading.Thread(target=self.wait_for_path_ready)
        ready_thread.start()

        self.start_journey_bool = False

        # Start thread to listen start_jorney 
        # for loading the saved poses from follow_waypoints/saved_path/poses.csv            
            
        start_journey_thread = threading.Thread(target=self.wait_for_start_journey)
        start_journey_thread.start()

        topic = self.addpose_topic;
        rospy.loginfo("Waiting to recieve waypoints via Pose msg on topic %s" % topic)
        rospy.loginfo("To start following waypoints: 'rostopic pub /path_ready std_msgs/Empty -1'")
        rospy.loginfo("OR")
        rospy.loginfo("To start following saved waypoints: 'rostopic pub /start_journey std_msgs/Empty -1'")


        # Wait for published waypoints or saved path  loaded
        while (not self.path_ready and not self.start_journey_bool):
            try:
                pose = rospy.wait_for_message(topic, PoseWithCovarianceStamped, timeout=1)
            except rospy.ROSException as e:
                if 'timeout exceeded' in str(e):
                    continue  # no new waypoint within timeout, looping...
                else:
                    raise e
            rospy.loginfo("Recieved new waypoint")
            waypoints.append(changePose(pose, "map"))
            # publish waypoint queue as pose array so that you can see them in rviz, etc.
            self.poseArray_publisher.publish(convert_PoseWithCovArray_to_PoseArray(waypoints))

        # Path is ready! return success and move on to the next state (FOLLOW_PATH)
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