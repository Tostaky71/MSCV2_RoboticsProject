#!/usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from math import radians, degrees
from actionlib_msgs.msg import *
from geometry_msgs.msg import Point
from kobuki_msgs.msg import Sound
import roslib
import sys

class map_navigation():

	def __init__(self): 
		
		# declare the sounds to play at each way point
		roslib.load_manifest("kobuki_testsuite")
		rospy.init_node("test_kobuki_Sound")
		pub = rospy.Publisher("/mobile_base/commands/sound", Sound, queue_size=1)
		rate = rospy.Rate(10.0)
		while not pub.get_num_connections():
    			rate.sleep()
		msg = Sound()

		# declare the coordinates of interest 
		self.x1 =  1.431
		self.y1 = 2.828
		self.x2 = 4.402
		self.y2 = 0.528
		self.x3 = 1.332
		self.y3 = -3.171
		self.x0 = -1.462
		self.y0 = -0.748
		self.goalReached = False
		# initialize
		choice = 0
		pub = rospy.Publisher('/mobile_base/commands/sound', Sound, queue_size=1)		

		if (choice == 0): # go to next way point
			msg.value = Sound.ON
			pub.publish(msg)
			self.goalReached = self.moveToGoal(self.x1, self.y1)
			choice += 1
		
		if (choice == 1): # go to the next way point
			msg.value = Sound.ON
			pub.publish(msg)
			self.goalReached = self.moveToGoal(self.x2, self.y2)
			choice += 1

		if (choice == 2): # go to the next way point
			msg.value = Sound.ON
			pub.publish(msg)
			self.goalReached = self.moveToGoal(self.x3, self.y3)
			choice += 1

		if (choice == 3): # go to the initial coordinates
			msg.value = Sound.ON
			pub.publish(msg)
			self.goalReached = self.moveToGoal(self.x0, self.y0)
			choice += 1

		if (choice == 4): # stop the program when the path is completed
			msg.value = Sound.CLEANINGEND
			pub.publish(msg)			
			self.shutdown()


		if (self.goalReached): # display 'Congratulations' when the path is completed
			rospy.loginfo("Congratulations!")


	def shutdown(self):
        # stop the program
        	rospy.loginfo("Quit program")
        	sys.exit()


	def moveToGoal(self,xGoal,yGoal):

		#define a client to send goal requests to the move_base server through a SimpleActionClient
		ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)

		#wait for the action server to come up
		while(not ac.wait_for_server(rospy.Duration.from_sec(5.0))):
			rospy.loginfo("Waiting for the move_base action server to come up")
		
		goal = MoveBaseGoal()

		#set up the frame parameters
		goal.target_pose.header.frame_id = "map"
		goal.target_pose.header.stamp = rospy.Time.now()

		# moving towards the goal*/

		goal.target_pose.pose.position = Point(xGoal,yGoal,0)
		goal.target_pose.pose.orientation.x = 0.0
		goal.target_pose.pose.orientation.y = 0.0
		goal.target_pose.pose.orientation.z = 0.0
		goal.target_pose.pose.orientation.w = 1.0

		rospy.loginfo("Sending goal location ...")
		ac.send_goal(goal)

		ac.wait_for_result(rospy.Duration(60))

		if(ac.get_state() ==  GoalStatus.SUCCEEDED):
			rospy.loginfo("You have reached the destination")	
			return True
	
		else:
			rospy.loginfo("The robot failed to reach the destination")
			msg.value = Sound.ERROR
			pub.publish(msg)
			return False

if __name__ == '__main__':
    try:
	
	rospy.loginfo("You have reached the destination")
        map_navigation()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("map_navigation node terminated.")
