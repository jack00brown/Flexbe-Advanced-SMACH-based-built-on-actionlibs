#!/usr/bin/env python

import struct
import sys
import copy
import ipdb
import math
import rospy
import rospkg

import geometry_msgs.msg
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
from std_msgs.msg import Header


from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)

import baxter_interface
import rospy
import actionlib
from flexbe_core.proxy import ProxyActionClient
# example import of required action

class BackToStartState(EventState):

    def __init__(self, initialize_robot):
		# See example_state.py for basic explanations.
		super(ExampleActionState, self).__init__(outcomes = ['continiue', 'failed'])

		self._initialize_robot = initialize_robot

		# Create the action client when building the behavior.
		# This will cause the behavior to wait for the client before starting execution
		# and will trigger a timeout error if it is not available.
		# Using the proxy client provides asynchronous access to the result and status
		# and makes sure only one client is used, no matter how often this state is used in a behavior.
		self._topic = 'goback'
		self._client = ProxyActionClient({self._topic: DoDishesAction}) # pass required clients as dict (topic: type)

		# It may happen that the action client fails to send the action goal.
		self._error = False
  
    def execute(self, userdata):
		# While this state is active, check if the action has been finished and evaluate the result.

		# Check if the client failed to send the goal.
		if self._error:
			return 'command_error'

		# Check if the action has been finished
		if self._client.has_result(self._topic):
			result = self._client.get_result(self._topic)
			current_angles=baxter_interface.Class Limb.joint_angle(self, joint)
			# Based on the result, decide which outcome to trigger.
			if starting_joint_angles==current_angles:
				return 'continiue'
			else:
				return 'failed'

# If the action has not yet finished, no outcome will be returned and the state stays active.

    def on_enter(self, userdata):
		# When entering this state, we send the action goal once to let the robot start its work.

		# As documented above, we get the specification of which dishwasher to use as input key.
		# This enables a previous state to make this decision during runtime and provide the ID as its own output key.
		dishwasher_id = userdata.dishwasher
  
  
  
                       # init robot
	            robot_state = baxter_interface.RobotEnable(baxter_interface.CHECK_VERSION)
	            robot_init_state = robot_state.state().enabled	
	            robot_state.enable()

	# init baxter right arm
	            limb = 'right'
	            right_limb = baxter_interface.Limb(limb)
	            right_gripper = baxter_interface.Gripper(limb)
	# start joint angles 
	# use this joint angles as a seed
                        starting_joint_angles = {'right_w0': -0.6699952259595108, # this is joint angles
                             	'right_w1': 1.030009435085784,
                             	'right_w2': 0.4999997247485215,
                             	'right_e0': -0.189968899785275,
                             	'right_e1': 1.9400238130755056,
                             	'right_s0': 0.08000397926829805,
                             	'right_s1': -0.9999781166910306}
                       right_limb = baxter_interface.Limb(limb)
                       right_limb.move_to_joint_positions(starting_joint_angles) #joint_angles
                       
                       
                    
		# Create the goal.
		 #goal = DoDishesGoal()
		# goal.dishwasher_id = dishwasher_id

		# Send the goal.
		self._error = False # make sure to reset the error state since a previous state execution might have failed
		try:
			self._rate.sleep()
		except ROSInterruptException:
			rospy.logwarn('Skipped sleep.')
   
#			self._client.send_goal(self._topic, goal)
#		except Exception as e:
			# Since a state failure not necessarily causes a behavior failure, it is recommended to only print warnings, not errors.
			# Using a linebreak before appending the error log enables the operator to collapse details in the GUI.
#			Logger.logwarn('Failed to send the DoDishes command:\n%s' % str(e))
                       self._error = True

    def on_exit(self, userdata):
		# Make sure that the action is not running when leaving this state.
		# A situation where the action would still be active is for example when the operator manually triggers an outcome.

		if not self._client.has_result(self._topic):
			self._client.cancel(self._topic)
                       Logger.loginfo('Cancelled active action goal.')
























