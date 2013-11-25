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

import getopt
import os
import sys
import errno
import roslib
#roslib.load_manifest('head_control')
#roslib.load_manifest('input_output')
roslib.load_manifest('baxter_interface')
import rospy

import baxter_interface

import cv
import cv_bridge

import sensor_msgs.msg
import baxter_msgs.srv

import dataflow
import copy

from baxter_msgs.msg import (
    CameraSettings,
    CameraControl,)

class DisplayControl(object):
   
	def __init__(self, name):
       
		
        	self._id = name
		self._open_svc = rospy.ServiceProxy('/cameras/open', baxter_msgs.srv.OpenCamera)
        	self._close_svc = rospy.ServiceProxy('/cameras/close', baxter_msgs.srv.CloseCamera)

		

        	self._settings = CameraSettings()
        	self._settings.width = 320
        	self._settings.height = 200
        	self._settings.fps = 20
        	self._open = False

		self._frameMsg = None
		
		

		#rospy.init_node('cameras_example', anonymous = True)

	def openCamera(self):

		if self._id == 'head_camera':
            		self._set_control_value(CameraControl.CAMERA_CONTROL_FLIP, True)
            		self._set_control_value(CameraControl.CAMERA_CONTROL_MIRROR, True)

		ret = self._open_svc(self._id, self._settings)
		if ret.err != 0:
				raise OSError(ret.err, "Failed to open camera")
        	self._open = True

		self._image_sub = rospy.Subscriber('/cameras/'+self._id+'/image', sensor_msgs.msg.Image, self._on_image_update)

		dataflow.wait_for(
            		lambda: not (self._frameMsg) is None,
            		timeout=5.0,
            		timeout_msg="Failed to get current image",
        	)


		#cv.NamedWindow("Img")

	def getFrame(self):

		
		frame = cv_bridge.CvBridge().imgmsg_to_cv(self._frameMsg, "bgr8")
		#cv.WaitKey(1)		
		#cv.ShowImage("Img",frame)
		#cv.WaitKey(1)

		return frame

	def _on_image_update(self, msg):

		self._frameMsg = copy.deepcopy(msg)

	def _set_control_value(self, control, value):
        	lookup = [c for c in self._settings.controls if c.id == control]
        	try:
            		lookup[0].value = value
        	except IndexError:
            		self._settings.controls.append(CameraControl(control, value))
					
	def closeCamera(self):
		ret = self._close_svc(self._id)
		if ret.err != 0 and ret.err != errno.EINVAL:
			raise OSError(ret.err, "Failed to close camera")
		self._open = False
