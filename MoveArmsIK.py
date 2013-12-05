#!/usr/bin/env python

# Copyright (c) 2013, Rethink Robotics
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
# 3. Neither the name of the Rethink Robotics nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""
Baxter RSDK Joint Position Example: file playback
"""

import argparse
import sys

import roslib
roslib.load_manifest('joint_position')
roslib.load_manifest('inverse_kinematics')
import rospy

import baxter_interface
import iodevices

from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
from std_msgs.msg import Header
from baxter_msgs.srv import SolvePositionIK
from baxter_msgs.srv import SolvePositionIKRequest

def inverse_kinematics(limb, x_p, y_p, z_p, x_o, y_o, z_o, w_o):
	ns = "/sdk/robot/limb/" + limb + "/solve_ik_position"
	rospy.wait_for_service(ns)
	iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
	ikreq = SolvePositionIKRequest()
	hdr = Header(stamp=rospy.Time.now(), frame_id='base')
	poses = {
		'left': PoseStamped(
			header=hdr,
			pose=Pose(
				position=Point(
					x = x_p,
					y = y_p,
					z = z_p,
					# EXAMPLE:
					# x=0.657579481614,
					# y=0.851981417433,
					# z=0.0388352386502,
				),
				orientation=Quaternion(
					x = x_o,
					y = y_o,
					z = z_o,
					w = w_o,
					# EXAMPLE:
					# x=-0.366894936773,
					# y=0.885980397775,
					# z=0.108155782462,
					# w=0.262162481772,
				),
			),
		),
		'right': PoseStamped(
			header=hdr,
			pose=Pose(
				position=Point(
					x = x_p,
					y = y_p,
					z = z_p,
					# EXAMPLE:
					# x=0.656982770038,
					# y=-0.852598021641,
					# z=0.0388609422173,
				),
				orientation=Quaternion(
					x = x_o,
					y = y_o,
					z = z_o,
					w = w_o,
					# EXAMPLE:
					# x=0.367048116303,
					# y=0.885911751787,
					# z=-0.108908281936,
					# w=0.261868353356,
				),
			),
		),
	}

	ikreq.pose_stamp.append(poses[limb])
	try:
		resp = iksvc(ikreq)
	except rospy.ServiceException,e :
		rospy.loginfo("Service call failed: %s" % (e,))
	if (resp.isValid[0]):
		# print("SUCCESS - Valid Joint Solution Found:")
		# Format solution into Limb API-compatible dictionary
		limb_joints = dict(zip(resp.joints[0].names, resp.joints[0].angles))
		return limb_joints
	else:
		# print("INVALID POSE - No Valid Joint Solution Found.")
		return '0'

def moveArmLoc(limb, x_p, y_p, z_p, x_o = 0.694, y_o = 0.717, z_o = -0.041, w_o = 0.0307):
	"""
	Move left or right arm (limb) to specified location
	The last four numbers default to a vertical end effector orientation
	"""
	
	if (inverse_kinematics(limb, x_p, y_p, z_p, x_o, y_o, z_o, w_o) == '0'):
		print("Invalid coordinates specified:")
		print(limb, x_p, y_p, z_p, x_o, y_o, z_o, w_o)
	else:
		if (limb == 'left'):
			left.move_to_joint_positions(inverse_kinematics(limb, x_p, y_p, z_p, x_o, y_o, z_o, w_o))
		if (limb == 'right'):
			right.move_to_joint_positions(inverse_kinematics(limb, x_p, y_p, z_p, x_o, y_o, z_o, w_o))
		
if __name__ == '__main__':
	
	# ============================
	# PARSE COMMAND LINE ARGUMENTS
	# ============================
	
	parser = argparse.ArgumentParser()
	#parser.add_argument("file", help="input file")
	#parser.add_argument("-l", "--loops", type=int, default=1, \
		#help="number of times to loop the input file. 0=infinite.")
	args = parser.parse_args()
	
	# ===========================
	# INITIALIZE AND ENABLE ROBOT
	# ===========================
	
	print("Initializing node... ")
	rospy.init_node("chess_arm_position_node")
	print("Getting robot state... ")
	rs = baxter_interface.RobotEnable()
	print("Enabling robot... ")
	rs.enable()
	
	# ============================
	# CREATE OBJECTS FOR BOTH ARMS
	# ============================
	
	left = baxter_interface.Limb('left')
	right = baxter_interface.Limb('right')
	
	# =====================
	# MOVE TO HOME POSITION
	# =====================
	
	left.move_to_neutral()
	right.move_to_neutral()
	
	# Print left arm end effector position and orientation
	print(left.endpoint_pose())
	
	# =======================
	# TEST FORWARD KINEMATICS
	# =======================
	
	"""
	lcmd_start = {'left_w0': 0.0249271877747, 'left_w1': -0.0563737938904, 'left_w2': 0.12386894848, 'left_e0': 0.08628641922, 'left_e1': 2.45091780104, 'left_s0': -0.616660276025, 'left_s1': -0.807640883899}
	rcmd_start = {'right_s0': 1.01702926121, 'right_s1': 0.185611675122, 'right_w0': 0.359718494348, 'right_w1': -0.0421844716187, 'right_w2': -0.469781615753, 'right_e0': -0.034514567688, 'right_e1': 1.44002446298}
	left.move_to_joint_positions(lcmd_start)
	right.move_to_joint_positions(rcmd_start)
	"""
	
	# =======================
	# TEST INVERSE KINEMATICS
	# =======================
	
	for i in xrange(0, 4):
		
		moveArmLoc('right', 0.322, -0.153, -0.275)
		moveArmLoc('right', 0.366, -0.153, -0.275)
		moveArmLoc('right', 0.415, -0.153, -0.275)
