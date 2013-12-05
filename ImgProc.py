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

import getopt
import os
import sys

import rospy
import cv
import cv_bridge
import time
import threading
from threading import Thread
from numpy import*
import numpy as np
import cv2
import roslib
roslib.load_manifest('head_control')
roslib.load_manifest('input_output')
roslib.load_manifest('baxter_interface')
roslib.load_manifest('joint_position')
import dataflow
import copy
import baxter_interface
import baxter_interface.navigator as NAV
import std_msgs.msg as stdmsg
import baxter_msgs.msg as baxmsg
import sensor_msgs.msg
import baxter_msgs.srv
import DisplayControl


from baxter_msgs.msg import (
    CameraSettings,
    CameraControl,)

def findPiece():
	n=NAV.Navigator('right')
	cameraName='right_hand_camera'

	gui_pub = rospy.Publisher('/sdk/xdisplay', sensor_msgs.msg.Image, latch=True)
	
	cam = baxter_interface.CameraController(cameraName)
	res = (960,600)
	cam.close()
	cam.resolution = res
	cam.open()
	action = None
	rs = baxter_interface.RobotEnable()
	
	capture = DisplayControl.DisplayControl(cameraName)
	largeFrame = cv.CreateImage((1024,600),cv.IPL_DEPTH_8U, 3)
	imcolor = cv.CreateImage((1024,600),cv.IPL_DEPTH_8U, 3)
	hsvC = cv.CreateImage((1024,600),cv.IPL_DEPTH_8U, 3)
	hsvN=np.asarray(hsvC[:,:])
	gray= cv.CreateImage((1024,600),cv.IPL_DEPTH_8U, 1)
	
	capture.openCamera()
	rospy.sleep(2)
	#cv.NamedWindow('Chess', cv.CV_WINDOW_AUTOSIZE)
	#initCamera('head_camera')
	
	#leftX=0
	#leftY=0
	#msg=cv_bridge.CvBridge().cv_to_imgmsg(largeFrame,"bgr8")
	#gui_pub.publish(msg)
	rospy.Rate(1).sleep()

	rs.enable()
	while(n.button0 == False):
		
		frame=capture.getFrame()
		cv.Resize(frame, imcolor, cv.CV_INTER_LINEAR)
		cv.CvtColor(imcolor,hsvC,cv.CV_BGR2HSV)
		hsvA = np.asarray(hsvC[:,:])
		im = np.asarray(imcolor[:,:])
		imOrig=im.copy()
		im=cv2.flip(im,1)
		hsvA=cv2.flip(hsvA,1)

		#Find object contours and get centroid cartesian positions       
		contours,hierarchy= cv2.findContours(hsvA,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)
		tempIm=im.copy()
		for cnt in contours:
			M = cv.moments(cnt)
			print M
			#Identify centroid Cartesian coordinates
			centroid_x = int(M['m10']/M['m00'])
			centroid_y = int(M['m01']/M['m00'])
			#Draw circle about centroids
			cv2.circle(tempIm, (centroid_x, centroid_y), 20, (0,255,255), 10)
			
		#Send centroid location to Baxter arm for repositioning
		
		
		imOrig=cv2.add(imOrig,tempIm)
		h, w = self.imOrig.shape[:2]
		imgO= cv.CreateMat(h, w, cv.CV_8UC3)
		np.asarray(imgO)[:,:] = imOrig

		largeFrame=cv.CloneImage(imcolor)
		cv.CvtColor(largeFrame,hsvC,cv.CV_BGR2HSV)
		
		cv.CvtColor(largeFrame,gray,cv.CV_BGR2GRAY)
		
		#imcolor=cv.QueryFrame(capture)
		msg = cv_bridge.CvBridge().cv_to_imgmsg(imgO, "bgr8")
		gui_pub.publish(msg)
		#cv.ShowImage('Chess',hsvC)
		
	#rs.disable()

def clean_line(line, names):
	
		#convert the line of strings to a float or None
	line = [try_float(x) for x in line.rstrip().split(',')]
		#zip the values with the joint names
	combined = zip(names[1:], line[1:])
		#take out any tuples that have a none value
	cleaned = [x for x in combined if x[1] is not None]
		#convert it to a dictionary with only valid commands
	command = dict(cleaned)
	left_command = dict((key, command[key]) for key in command.keys() \
		if key[:-2] == 'left_')
	right_command = dict((key, command[key]) for key in command.keys() \
		if key[:-2] == 'right_')
	return (command, left_command, right_command, line)
		
def try_float(x):
	try:
		return float(x)
	except ValueError:
		return None

if __name__ == '__main__':
	rospy.init_node('MABL_GUI', anonymous=True)
	findPiece()
