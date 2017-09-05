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

class InitializeRobotState(EventState):

    def __init__(self,  ):
		"""Constructor"""

        super(ExecuteTrajectoryState, self).__init__(outcomes = ['done', 'failed'])

        #self._client = ProxyActionClient({self._action_topic: FollowJointTrajectoryAction})
        self._starting_joint_angles = starting_joint_angles
        self._starting_joint_angles = {'right_w0': -0.6699952259595108, # this is joint angles
                             	'right_w1': 1.030009435085784,
                             	'right_w2': 0.4999997247485215,
                             	'right_e0': -0.189968899785275,
                             	'right_e1': 1.9400238130755056,
                             	'right_s0': 0.08000397926829805,
                             	'right_s1': -0.9999781166910306}
        self._done = False
        self._failed = False

  
    def execute(self, userdata):
		"""Wait for action result and return outcome accordingly"""

        if self._done:
            return 'done'
        if self._failed:
            return 'failed'

	

# If the action has not yet finished, no outcome will be returned and the state stays active.

    def on_enter(self, userdata):

                    self._done = False
                    self._failed = False
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

                    right_limb.move_to_joint_positions(self._starting_joint_angles) #joint_angles
       
                    try:
                         self._client.send_goal(self._action_topic, goal)
                    except Exception as e:
                      Logger.logwarn("Unable to send follow joint trajectory action goal:\n%s" % str(e))
                      self._failed = True
                       
                       
                    
	
    def on_exit(self, userdata):
		# Make sure that the action is not running when leaving this state.
		# A situation where the action would still be active is for example when the operator manually triggers an outcome.

		if not self._client.has_result(self._topic):
			self._client.cancel(self._topic)
                       Logger.loginfo('Cancelled active action goal.')
























