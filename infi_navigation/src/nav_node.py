#! /usr/bin/env python
# coding=utf-8

import rospy
import time
import numpy as np
from std_srvs.srv import Empty
from std_msgs.msg import Float64, String
from sensor_msgs.msg import JointState
from gazebo_msgs.srv import SetModelState, SetModelStateRequest, SetModelConfiguration, SetModelConfigurationRequest
from gazebo_msgs.srv import GetModelState, GetModelStateRequest, GetLinkState, GetLinkStateRequest
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SpawnModel, SpawnModelRequest, SpawnModelResponse
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from nav_msgs.msg import Odometry, Path

import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from random import sample
from math import pow, sqrt


def trunc(f, n):
    # Truncates/pads a float f to n decimal places without rounding
    slen = len('%.*f' % (n, f))
    return float(str(f)[:slen])

class Nav:

    def __init__(self):

        rospy.init_node('nav_node')

        self.link_name_lst = ['OPTIMUS::base_footprint','OPTIMUS::whFL', 'OPTIMUS::whFR','OPTIMUS::whRL','OPTIMUS::whRR']
        self.joint_name_lst = ['j_whFR', 'j_whFL', 'j_whRR','j_whRL']
        self.save_req_sub = rospy.Subscriber('/SaveDatPos',String,self.save_req_callback)
        self.nav_path_sub = rospy.Subscriber("/move_base/NavfnROS/plan", Path, self.path_callback)

        #Gazebo stuff
        self.pause_proxy = rospy.ServiceProxy('/gazebo/pause_physics',Empty)
        self.unpause_proxy = rospy.ServiceProxy('/gazebo/unpause_physics',Empty)
        self.reset_world = rospy.ServiceProxy('/gazebo/reset_world',Empty)

        self.get_model_state_proxy = rospy.ServiceProxy('/gazebo/get_model_state',GetModelState)
        self.get_model_state_req = GetModelStateRequest()
        self.get_model_state_req.model_name = 'OPTIMUS'
        self.get_model_state_req.relative_entity_name = 'world'

        self.get_link_state_proxy = rospy.ServiceProxy('/gazebo/get_link_state',GetLinkState)
        self.get_link_state_req = GetLinkStateRequest()
        self.get_link_state_req.link_name = ''  # TODO: change
        self.get_link_state_req.reference_frame = 'world'

        self.filename = 'pos_file.csv'
        # # make CSV file name from these params
        # with open(self.filename, 'w') as f:
        #     f.write('name,x , y, z, roll, pitch, theta\n')
        #     self.flag = False

        rospy.loginfo("[nav_node]: started")

        self.navigate(self.load_nav_tasks(self.filename))
    def path_callback(self,data):
        sum=0
        print("--------------------------------------------")
        for i,pose in enumerate(data.poses):
            print("{} ; {}".format(pose.pose.position.x, pose.pose.position.y))
            if i != len(data.poses)-1:
                sum += sqrt(pow((data.poses[i + 1].pose.position.x - data.poses[i].pose.position.x), 2)
                            + pow((data.poses[i + 1].pose.position.y - data.poses[i].pose.position.y), 2))
        print("--------------------------------------------")
        print("sum: {}".format(sum))



    def navigate(self, locations):
        # Set up the goal locations. Poses are defined in the map frame.
        # An easy way to find the pose coordinates is to point-and-click
        # Nav Goals in RViz when running in the simulator.
        # Pose coordinates are then displayed in the terminal
        # that was used to launch RViz.
        # Goal state return values
        goal_states = ['PENDING', 'ACTIVE', 'PREEMPTED',
                       'SUCCEEDED', 'ABORTED', 'REJECTED',
                       'PREEMPTING', 'RECALLING', 'RECALLED',
                       'LOST']
        # locations['pile'] = Pose(Point(0.643, 4.720, 0.000), Quaternion(0.000, 0.000, 0.223, 0.975))
        # locations['in_front_pile'] = Pose(Point(-1.994, 4.382, 0.000), Quaternion(0.000, 0.000, -0.670, 0.743))
        # Publisher to manually control the robot (e.g. to stop it)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        # Subscribe to the move_base action server
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        # Wait 60 seconds for the action server to become available
        self.move_base.wait_for_server()  # rospy.Duration(60)
        rospy.loginfo("Connected to move base server")
        # A variable to hold the initial pose of the robot to be set by
        # the user in RViz
        initial_pose = PoseWithCovarianceStamped()

        # Variables to keep track of success rate, running time,
        # and distance traveled
        n_locations = len(locations)
        n_goals = 0
        n_successes = 0
        i = n_locations
        distance_traveled = 0
        start_time = rospy.Time.now()
        running_time = 0
        location = ""
        last_location = ""

        # Get the initial pose from the user
        rospy.loginfo("*** Click the 2D Pose Estimate button in RViz to set the robot's initial pose...")
        rospy.wait_for_message('initialpose', PoseWithCovarianceStamped)
        self.last_location = Pose()

        rospy.Subscriber('initialpose', PoseWithCovarianceStamped, self.update_initial_pose)
        # Make sure we have the initial pose
        while initial_pose.header.stamp == "":
            rospy.sleep(1)

        rospy.loginfo("Starting navigation")

        # Begin the main loop and run through a sequence of locations
        while not rospy.is_shutdown():
            # If we've gone through the current sequence,
            # start with a new random sequence
            if i == n_locations:
                i = 0
                sequence = sample(locations, n_locations)
                # Skip over first location if it is the same as
                # the last location
                if sequence[0] == last_location:
                    i = 1

            # Get the next location in the current sequence
            location = sequence[i]

            # Keep track of the distance traveled.
            # Use updated initial pose if available.
            if initial_pose.header.stamp == "":
                distance = sqrt(pow(locations[location].position.x -
                                    locations[last_location].position.x, 2) +
                                pow(locations[location].position.y -
                                    locations[last_location].position.y, 2))
            else:
                rospy.loginfo("Updating current pose.")
                distance = sqrt(pow(locations[location].position.x -
                                    initial_pose.pose.pose.position.x, 2) +
                                pow(locations[location].position.y -
                                    initial_pose.pose.pose.position.y, 2))
                initial_pose.header.stamp = ""

            # Store the last location for distance calculations
            last_location = location

            # Increment the counters
            i += 1
            n_goals += 1

            # Set up the next goal location
            self.goal = MoveBaseGoal()
            self.goal.target_pose.pose = locations[location]
            self.goal.target_pose.header.frame_id = 'map'
            self.goal.target_pose.header.stamp = rospy.Time.now()

            # Let the user know where the robot is going next
            rospy.loginfo("Going to: " + str(location))

            # Start the robot toward the next location
            self.move_base.send_goal(self.goal)

            # Allow 5 minutes to get there
            finished_within_time = self.move_base.wait_for_result(rospy.Duration(300))

            # Check for success or failure
            if not finished_within_time:
                self.move_base.cancel_goal()
                rospy.loginfo("Timed out achieving goal")
            else:
                state = self.move_base.get_state()
                if state == GoalStatus.SUCCEEDED:
                    rospy.loginfo("Goal succeeded!")
                    n_successes += 1
                    distance_traveled += distance
                    rospy.loginfo("State:" + str(state))
                else:
                    rospy.loginfo("Goal failed with error code: " + str(goal_states[state]))

            # How long have we been running?
            running_time = rospy.Time.now() - start_time
            running_time = running_time.secs / 60.0

            # Print a summary success/failure, distance traveled and time elapsed
            rospy.loginfo("Success so far: " + str(n_successes) + "/" +
                          str(n_goals) + " = " +
                          str(100 * n_successes / n_goals) + "%")
            rospy.loginfo("Running time: " + str(trunc(running_time, 1)) +
                          " min Distance: " + str(trunc(distance_traveled, 1)) + " m")

            # rospy.sleep(self.rest_time)

    def save_req_callback(self, data):
        self.flag = True
        self.write_pos(data.data)

    def write_pos(self,type):
        rospy.wait_for_service('/gazebo/get_model_state')
        model_state = self.get_model_state_proxy(self.get_model_state_req)
        pos = np.array([model_state.pose.position.x, model_state.pose.position.y, model_state.pose.position.z])
        orientation = model_state.pose.orientation
        (roll, pitch, theta) = euler_from_quaternion(
            [orientation.x, orientation.y, orientation.z, orientation.w])
        if self.flag:
            with open(self.filename, 'a') as fd:
                self.flag = False
                fd.write(
                    '%s,%3.3f,%3.3f,%3.3f,%3.3f,%3.3f,%3.3f\n' % (type,pos[0], pos[1], pos[2], roll, pitch, theta))
                rospy.loginfo('%s,%3.3f,%3.3f,%3.3f,%3.3f,%3.3f,%3.3f\n' % (type,pos[0], pos[1], pos[2], roll, pitch, theta))

    def load_nav_tasks(self,filename):
        locations = dict()
        with open(filename, 'r') as fd:
            content_list = [l.strip() for l in fd]
            for i, x in enumerate(content_list):
                q = quaternion_from_euler(x[4], x[5], x[6])
                locations[str(x[0])] = Pose(Point(x[1], x[2], x[3]), Quaternion(q[0], q[1], q[2], q[3]))
        return locations


    def update_initial_pose(self, initial_pose):
        self.initial_pose = initial_pose

    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        self.move_base.cancel_goal()
        rospy.sleep(2)
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1)


if __name__ == '__main__':
    try:
        Nav()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("AMCL navigation test finished.")


