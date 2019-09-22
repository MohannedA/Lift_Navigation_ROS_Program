#!/usr/bin/env python

# This program was written by: Hesham AlKhouja & Mohanned Ahmed


import roslib
import rospy
from geometry_msgs.msg import *
import time 
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from math import radians, degrees
from actionlib_msgs.msg import *
import tf
import math

class FirstFloorLocations():
	LIBRARY = [-0.784406331471, -0.857920377034, -93.2649926275]
	VENDING_MACHINE = [-0.0889150346874, 2.06382619103, -7.16993904583]
	MOSALLA = [-10.112787122, 0.756454743744, -103.69204506]
	STAIRS = [-11.6818215519, 3.60649224993, -0.173685241888] 

def go_to_initial_pose(pose):
    pub = rospy.Publisher('initialpose', PoseWithCovarianceStamped, queue_size=10)
    time.sleep(2)

    p   = PoseWithCovarianceStamped();
    msg = PoseWithCovariance();
    msg.pose = pose 
    msg.covariance = [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891945200942];
    p.pose = msg
    pub.publish(p)

def navigate_to_goal(location_coordinates):

	# Define a client for to send goal requests to the move_base server through a SimpleActionClient.
	ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)

	# Wait for the action server to come up.
	while(not ac.wait_for_server(rospy.Duration.from_sec(5.0))):
		rospy.loginfo("Waiting for the move_base action server to come up")
	

	goal = MoveBaseGoal()

	# Set up the frame parameters.
	goal.target_pose.header.frame_id = "map"
	goal.target_pose.header.stamp = rospy.Time.now()

	# Move towards the goal.
	xGoal = location_coordinates[0]
	yGoal = location_coordinates[1]
	yawGoal = location_coordinates[2]

	quaternionGoal = tf.transformations.quaternion_from_euler(0, 0, math.radians(yawGoal))

	goal.target_pose.pose.position =  Point(xGoal,yGoal,0)
	goal.target_pose.pose.orientation.x = quaternionGoal[0]
	goal.target_pose.pose.orientation.y = quaternionGoal[1]
	goal.target_pose.pose.orientation.z = quaternionGoal[2]
	goal.target_pose.pose.orientation.w = quaternionGoal[3]

	rospy.loginfo("Sending goal location ...")
	ac.send_goal(goal)

	ac.wait_for_result(rospy.Duration(60))

	if(ac.get_state() ==  GoalStatus.SUCCEEDED):
		rospy.loginfo("You have reached the destination")	
		return True

	else:
		rospy.loginfo("The robot failed to reach the destination")
		return False

def choose():
	choice='q'
	rospy.loginfo("|-------------------------------|")
	rospy.loginfo("|PRESSE A KEY:")
	rospy.loginfo("|'0': LIBRARY ")
	rospy.loginfo("|'1': VENDING_MACHINE")
	rospy.loginfo("|'2': MOSALLA")
	rospy.loginfo("|'3': STAIRS")
	rospy.loginfo("|PRESSE ANY KEY OTHER THAN ABOVE TO LOGOUT ...")
	rospy.loginfo("|-------------------------------|")
	rospy.loginfo("|WHERE TO GO?")
	choice = input()
	return choice

def go_to_start_point_in_ground_floor():
	# Set starting point pose. 
	pose = Pose(Point(-0.71207386294, -8.12948439849, 0.000), Quaternion(0.000, 0.000, 0.996125340951, 0.087944898175))
	# Go to starting point pose. 
	go_to_initial_pose(pose)

def navigate_to_ground_lift():
	# Set ground lift coordinates. 
	coordinates = [-6.17, -4.71, 87.5622495297] 
	# Navigate to ground lift.
	navigate_to_goal(coordinates)

def go_inside_first_floor_lift():
	# Set first floor inside lift pose. 
	pose = Pose(Point(-4.74523470553, 4.55686651324, 0.000), Quaternion(0.000, 0.000, -0.767481612832, 0.641070958604))
	# Go to first floor inside lift pose. 
	go_to_initial_pose(pose)

def get_choice_coordinates(choice):
	if choice == 0:
		return FirstFloorLocations.LIBRARY
	elif choice == 1:
		return FirstFloorLocations.VENDING_MACHINE
	elif choice == 2: 
		return FirstFloorLocations.MOSALLA
	elif choice == 3: 
		return FirstFloorLocations.STAIRS
	return None 

if __name__ == '__main__':
    try:
    	# Init ROS node.
    	rospy.init_node('lift_map_navigation', anonymous=False) 
    	# Choose the goal. 
    	choice = choose()
    	goal_coordinates = get_choice_coordinates(choice)
    	if goal_coordinates is not None:
	    	# Go to starting point pose.
	    	rospy.loginfo("Going to starting point ...") 
	    	go_to_start_point_in_ground_floor()
	    	# Navigate to lift of ground 0 location and looks towards the lift.
	        rospy.loginfo("Navigating to ground lift ...")
	        navigate_to_ground_lift()
	        # Wait for 3 seconds.
	        rospy.loginfo("Waiting 3 seconds ...")
	        time.sleep(3)
	        # Change robot location to be inside the lift of the first floor.
	        rospy.loginfo("Going inside first floor lift ...")
	        go_inside_first_floor_lift()
	        # Wait for 1 second.
	        rospy.loginfo("Waiting 1 second ...")
	        time.sleep(1)
	        # Navigate towards the specified goal location of the first floor.
	        rospy.loginfo("Navigating to specified goal ...")
	        navigate_to_goal(goal_coordinates)
	        rospy.loginfo("Done")
        else:
        	rospy.loginfo("You did not choose a goal")

    except Exception, e:
        print "error: ", e