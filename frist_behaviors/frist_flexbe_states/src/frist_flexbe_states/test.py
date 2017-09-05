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

ns = "ExternalTools/" + 'right' + "/PositionKinematicsNode/IKService"
_iksvc = rospy.ServiceProxy(ns, SolvePositionIK)

def ik_request(pose, verbose):
    hdr = Header(stamp=rospy.Time.now(), frame_id='base')
    ikreq = SolvePositionIKRequest()
    ikreq.pose_stamp.append(PoseStamped(header=hdr, pose=pose))
    try:
        resp = _iksvc(ikreq)
    except (rospy.ServiceException, rospy.ROSException), e:
        rospy.logerr("Service call failed: %s" % (e,))
        return False
    # Check if result valid, and type of seed ultimately used to get solution
    # convert rospy's string representation of uint8[]'s to int's
    resp_seeds = struct.unpack('<%dB' % len(resp.result_type), resp.result_type)
    limb_joints = {}
    if (resp_seeds[0] != resp.RESULT_INVALID):
        seed_str = {
                    ikreq.SEED_USER: 'User Provided Seed',
                    ikreq.SEED_CURRENT: 'Current Joint Angles',
                    ikreq.SEED_NS_MAP: 'Nullspace Setpoints',
                   }.get(resp_seeds[0], 'None')
        if verbose:
            print("IK Solution SUCCESS - Valid Joint Solution Found from Seed Type: {0}".format((seed_str)))
            # Format solution into Limb API-compatible dictionary
            limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
        if verbose:
            print("IK Joint Solution:\n{0}".format(limb_joints))
            print("------------------")
    else:
        rospy.logerr("INVALID POSE - No Valid Joint Solution Found.")
        return False
    return limb_joints


global_object_pose = None
def object_pose_callback(object_pose):
    #print "Hello Python"
    #print "{}".format(object_pose)
    global global_object_pose 
    global_object_pose = copy.deepcopy(object_pose)


def main():
	ipdb.set_trace()
	rospy.init_node("compute_IK")
	# sub 
	rospy.Subscriber("object_pose", geometry_msgs.msg.Transform, object_pose_callback)
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
	delta_x = 0.1012
        delta_y = 0.02523
        delta_z = 0.02
        delta_rx = 0.0
        delta_ry = 0.0081525
        delta_rz = 0.0
        delta_rw = 0.0
	hove = 0.15
	# IK test pose
	test_pose_hove = Pose()
	test_pose_hove.position.x = delta_x + global_object_pose.translation.x
	test_pose_hove.position.y = delta_y + global_object_pose.translation.y
	test_pose_hove.position.z = delta_z + global_object_pose.translation.z + hove 

        test_pose_hove.orientation.x = math.sqrt(1.0 - (delta_ry + global_object_pose.rotation.y)**2)
	test_pose_hove.orientation.y = delta_ry + global_object_pose.rotation.y
	test_pose_hove.orientation.z = 0.0
	test_pose_hove.orientation.w = 6.123233995736766e-17

	# move to start joint angles
	right_limb.move_to_joint_positions(starting_joint_angles) #joint_angles
        # move to hove
	IK_joint_angles_hove = ik_request(test_pose_hove, verbose = True)
	right_limb.move_to_joint_positions(IK_joint_angles_hove) 
	rospy.sleep(2)
	# move down
	test_pose = copy.deepcopy(test_pose_hove)
	test_pose.position.z = test_pose.position.z - hove
	IK_joint_angles = ik_request(test_pose, verbose = True)
	right_limb.move_to_joint_positions(IK_joint_angles) 
	right_gripper.close()
	rospy.sleep(0.5)
	# move up
	IK_joint_angles_hove = ik_request(test_pose_hove, verbose = True)
	right_limb.move_to_joint_positions(IK_joint_angles_hove) 
	# move left
	test_pose_left = copy.deepcopy(test_pose_hove)
	test_pose_left.position.y = test_pose_left.position.y + 0.25
	IK_joint_angles = ik_request(test_pose_left, verbose = True)
	right_limb.move_to_joint_positions(IK_joint_angles) 
	# move down
	test_pose_down = copy.deepcopy(test_pose_left)
	test_pose_down.position.z = test_pose_down.position.z - hove
	IK_joint_angles = ik_request(test_pose_down, verbose = True)
	right_limb.move_to_joint_positions(IK_joint_angles) 
	right_gripper.open()
	rospy.sleep(0.5)
	# move to start
	test_pose_up = copy.deepcopy(test_pose_down)
	test_pose_up.position.z = test_pose_up.position.z + hove
	IK_joint_angles = ik_request(test_pose_up, verbose = True)
	right_limb.move_to_joint_positions(IK_joint_angles) 
	right_limb.move_to_joint_positions(starting_joint_angles) 

	#rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
