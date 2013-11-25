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

def test_menu():
	#haarFace = cv.Load('haarcascade_frontalface_alt.xml')
	#video_test='/home/mabl/Videos/TestVideo.mp4'
	#capture = cv.CaptureFromFile(video_test)
	n=NAV.Navigator('right')
	cameraName='right_hand_camera'
	#surface= cv.LoadImage('Blank-Face.jpg', cv.CV_LOAD_IMAGE_COLOR)
	#surfaceBlood= cv.LoadImage('Blank-Face_1.jpg', cv.CV_LOAD_IMAGE_COLOR)
	
	gui_pub = rospy.Publisher('/sdk/xdisplay', sensor_msgs.msg.Image, latch=True)
	#storage = cv.CreateMemStorage()
	cam = baxter_interface.CameraController(cameraName)
	res = (960,600)
	cam.close()
	cam.resolution =res
	cam.open()
	action = None
	rs = baxter_interface.RobotEnable()
	
	#cv.Ellipse(surface,rightEye,(majorA,minorA),0,0,360,cv.RGB(255,0,0),1)
	#cv.Ellipse(surface,leftEye,(majorA,minorA),0,0,360,cv.RGB(255,0,0),1)
	
	#tempCamH= DisplayControl.DisplayControl('head_camera')
	#tempCamH.closeCamera()
	#tempCamR = DisplayControl.DisplayControl('right_hand_camera')
	#tempCamH.closeCamera()
	#tempCamL = DisplayControl.DisplayControl('left_hand_camera')
	#tempCamH.closeCamera()		
	capture = DisplayControl.DisplayControl(cameraName)
	largeFrame = cv.CreateImage((1024,600),cv.IPL_DEPTH_8U, 3)
	imcolor = cv.CreateImage((1024,600),cv.IPL_DEPTH_8U, 3)
	hsvC = cv.CreateImage((1024,600),cv.IPL_DEPTH_8U, 3)
	hsvN=np.asarray(hsvC[:,:])
	gray= cv.CreateImage((1024,600),cv.IPL_DEPTH_8U, 1)
	#rightEye=(405,233)
	#leftEye=(660,240)
	#majorA=10
	#minorA=10
	
	#cv.Resize(surfaceBlood, largeFrame, cv.CV_INTER_LINEAR)
	#zombieHungry = cv.CloneImage(largeFrame)
	#cv.Resize(surface, largeFrame, cv.CV_INTER_LINEAR)
	#zombie = cv.CloneImage(largeFrame)
	#cv.Ellipse(largeFrame,rightEye,(majorA,minorA),0,0,360,cv.RGB(255,0,0),-1)
	#cv.Ellipse(largeFrame,leftEye,(majorA,minorA),0,0,360,cv.RGB(255,0,0),-1)
	
	capture.openCamera()
	rospy.sleep(2)
	#cv.NamedWindow('Chess', cv.CV_WINDOW_AUTOSIZE)
	#initCamera('head_camera')
	
	#leftX=0
	#leftY=0
	#msg=cv_bridge.CvBridge().cv_to_imgmsg(largeFrame,"bgr8")
	#gui_pub.publish(msg)
	rospy.Rate(1).sleep()
	#test_playback('zombie.csv')
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

		       
		contours,hierarchy= cv2.findContours(hsvA,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)
		tempIm=im.copy()
		for cnt in contours:
			M = cv.moments(cnt)
			print M
			centroid_x = int(M['m10']/M['m00'])
			centroid_y = int(M['m01']/M['m00'])
			cv2.circle(tempIm, (centroid_x, centroid_y), 20, (0,255,255), 10)
		
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
		
		#test_playback('zombieMove.csv')
		
	#rs.disable()
	
def moveEyes(pub_gui,zombieImage,origImage,leftx,lefty,detectFaces):
	for face in detectFaces:
		zombieImage=cv.CloneImage(origImage)
		if leftx==0:
			leftx=660
			lefty= 240
		else :
			leftx = 660 + int((face[0][0] - 660)*0.1)
			lefty = 240 + int(0.1*(face[0][1]-240))
				#width = face[0][2]
		cv.Ellipse(zombieImage,(leftx,lefty),(10,10),0,0,360,cv.RGB(255,0,0),-1)
		cv.Ellipse(zombieImage,(leftx-255,lefty-6),(10,10),0,0,360,cv.RGB(255,0,0),-1)
				#cv.Rectangle(imcolor,(face[0][0],face[0][1]),(face[0][0]+face[0][2],face[0][1]+face[0][3]),cv.RGB(155, 255, 25),2)
		
		msg = cv_bridge.CvBridge().cv_to_imgmsg(zombieImage, "bgr8")
		pub_gui.publish(msg)
	return leftx,lefty
	rospy.Rate(1).sleep()

def test_playback(name):
	left = baxter_interface.Limb('left')
	right = baxter_interface.Limb('right')
	grip_left = baxter_interface.Gripper('left')
	grip_right = baxter_interface.Gripper('right')
	loops =1
			
	print("Getting robot state... ")
	rs = baxter_interface.RobotEnable()
	print("Enabling robot... ")
	rs.enable()
	rospy.sleep(3)
	path='/home/mabl/git/sdk-examples/baxter/examples/joint_position/src/playback/'
	filename = path+name
	n=NAV.Navigator('right')
	rate = rospy.Rate(100)
	start_time = rospy.get_time()
	print("Playing back: %s" % (filename,))
		
	with open(filename, 'r') as f:
		lines = f.readlines()
	keys = lines[0].rstrip().split(',')
	i = 0
	l = 0
	rospy.Rate(10).sleep()
	while loops < 1 or l < loops:
		l=l+1
		
		md_start, lcmd_start, rcmd_start, raw_start = clean_line(lines[1], keys)
		left.move_to_joint_positions(lcmd_start)
		right.move_to_joint_positions(rcmd_start)
		for values in lines[1:]:
			i = i +1
			loopstr = str(loops) if loops > 0 else "forever"
			sys.stdout.write("\r Record %d of %d, loop %d of %s" \
				% (i, len(lines)-1,l,loopstr))
			sys.stdout.flush()
			cmd, lcmd, rcmd, values = clean_line(values, keys)
			while (rospy.get_time() - start_time) < values[0] :
			
				if rospy.is_shutdown():
					print("\n ROS shutdown")
					#sys.exit(0)
				
				if (n.button2==True)or(n.button1==True)or(n.button0==True):
					print("\n stopped")
					#rs.disable()
					return False
				if len(lcmd):
					left.set_joint_positions(lcmd)
					            
				if len(rcmd):
					right.set_joint_positions(rcmd)
					
				if 'left_gripper' in cmd:
					grip_left.set_position(cmd['left_gripper'])
				if 'right_gripper' in cmd:
					grip_right.set_position(cmd['right_gripper'])
			rate.sleep()
	return True
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
	#pygame.init()
	#Thread(target=test_playback,args=('zombie.csv',)).start()
	#test_playback('zombie.csv')
	test_menu()
