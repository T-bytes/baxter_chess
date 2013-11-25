#!/usr/bin/env python
#-*- coding: utf-8 -*-

"""-------------------------------------------------------------------------
    Control de cursor, click derecho e izquierdo, mediante procesamiento de imÃ¡genes.

    author:		Yeison Cardona
    contact:		yeison.eng@gmail.com 
    first release:	12/11/2012
    last release:	22/11/2012

    This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2.1 of the License, or (at your option) any later version.

    This library is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
    Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public
    License along with this library; if not, write to the Free Software
    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
-------------------------------------------------------------------------"""

import pickle
import getopt
import os
import sys

import rospy
import cv
import cv_bridge
import time
import threading
from threading import Thread
#from numpy import*
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
#import pygame
from baxter_msgs.msg import (
    CameraSettings,
    CameraControl,)

try: 
    import numpy as np
    from numpy import sqrt, arccos, rad2deg
except:
    print "need Numpy for Python!!."
    exit()

try: import cv2
except:
    print "need OpenCV for Python!!."
    exit()

#Chequear dependencia de OpenCV 2.4
if not cv2.__version__ >= "2.4":
    print "OpenCV version to old!!."
    print "need version >= 2.4."
    exit()

#Checkear dependencia de xdotool package   
#try: os.system("xdotool --help")
#except:
 #   print "need xdotool dependence."
  #  exit()

########################################################################
class HandTracking(object):
    
    #----------------------------------------------------------------------
	def __init__(self):
		#self.debugMode = False
		self.debugMode = True
		self.n=NAV.Navigator('right')
		self.cameraName='head_camera'
        #self.camera = cv2.VideoCapture(2)  #Cambiar para usar otra cÃ¡mara
        
        
		self._gui_pub = rospy.Publisher('/sdk/xdisplay', sensor_msgs.msg.Image, latch=True)
        #ResolusiÃ³n a usar
        #self.camera.set(3,640)
        #self.camera.set(4,480)
		self._cam = baxter_interface.CameraController(self.cameraName)
		self._res = (960,600)
		self._cam.close()
		self._cam.resolution = self._res
		self._cam.open()
		self._action = None
		#tempCamH= DisplayControl.DisplayControl('head_camera')
		#tempCamH.closeCamera()
		#tempCamR = DisplayControl.DisplayControl('right_hand_camera')
		#tempCamH.closeCamera()
		#tempCamL = DisplayControl.DisplayControl('left_hand_camera')
		#tempCamH.closeCamera()		
		self._dc = DisplayControl.DisplayControl(self.cameraName)
		self._dc.openCamera()
		rospy.sleep(2);
		
		self.posPre = 0  #Para obtener la posisiÃ³n relativa del Â«mouseÂ»
		self.imcolor = cv.CreateImage((1024,600),cv.IPL_DEPTH_8U, 3)
        #Diccionario para almacenar nuestros datos
		self.Data =	{"angles less 90" : 0,
					"cursor" : (0, 0),
					"hulls" : 0, 
					"defects" : 0,
					"fingers": 0,
					"fingers history": [0],
					"area": 0,
					}
        #Ãšltima actualizaciÃ³n
		self.lastData = self.Data
		#self.path="Python-OpenCV-HandTracking/"
		self.path="test"
        #Cargamos las variables de los filtros
        #Si se modifican durante la ejecuciÃ³n, se actualizarÃ¡n en el fichero
		#try:  self.Vars = pickle.load(open(self.path+".config", "r"))
		try:  self.Vars = pickle.load(open("test.config", "r"))
		except:
			print "Config file (Â«.configÂ») not found."
			exit()
		largeFrame = cv.CreateImage((1024,600),cv.IPL_DEPTH_8U, 3)
        #Ventanas independientes para los filtros
		cv.NamedWindow("Filters")
		cv.CreateTrackbar("erode", "Filters", self.Vars["erode"], 255, self.onChange_erode)
		cv.CreateTrackbar("dilate", "Filters", self.Vars["dilate"], 255, self.onChange_dilate)
		cv.CreateTrackbar("smooth", "Filters", self.Vars["smooth"], 255, self.onChange_smooth)
        
		cv.NamedWindow("HSV Filters")        
		cv.CreateTrackbar("upper", "HSV Filters", self.Vars["upper"], 255, self.onChange_upper)
		cv.CreateTrackbar("filterUpS", "HSV Filters", self.Vars["filterUpS"], 255, self.onChange_fuS)
		cv.CreateTrackbar("filterUpV", "HSV Filters", self.Vars["filterUpV"], 255, self.onChange_fuV)        
		cv.CreateTrackbar("lower", "HSV Filters", self.Vars["lower"], 255, self.onChange_lower)   
		cv.CreateTrackbar("filterDownS", "HSV Filters", self.Vars["filterDownS"], 255, self.onChange_fdS)
		cv.CreateTrackbar("filterDownV", "HSV Filters", self.Vars["filterDownV"], 255, self.onChange_fdV)

        #Agregar texto
		self.addText = lambda image, text, point:cv2.putText(image,text, point, cv2.FONT_HERSHEY_PLAIN, 1.0,(255,255,255))
		rospy.Rate(1).sleep()
		
  
        #Siempre
		#while self.n.button0==False:
		#	self.run()  #Procese la imagen
            #self.interprete()  #Interprete eventos (clicks)
            #self.updateMousePos()  #Mueve el cursor
            #if self.debugMode:
		#rospy.Rate(1).sleep()		

            
    #----------------------------------------------------------------------
	def onChange_fuS(self, value):
		self.Vars["filterUpS"] = value
		pickle.dump(self.Vars, open(self.path+".config", "w"))
		rospy.Rate(1).sleep()
        
    #----------------------------------------------------------------------
	def onChange_fdS(self, value):
		self.Vars["filterDownS"] = value
		pickle.dump(self.Vars, open(self.path+".config", "w"))
		rospy.Rate(1).sleep()
        
    #----------------------------------------------------------------------
	def onChange_fuV(self, value):
		self.Vars["filterUpV"] = value
		pickle.dump(self.Vars, open(self.path+".config", "w"))
		rospy.Rate(1).sleep()
        
    #----------------------------------------------------------------------
	def onChange_fdV(self, value):
		self.Vars["filterDownV"] = value
		pickle.dump(self.Vars, open(self.path+".config", "w"))
		rospy.Rate(1).sleep()
        
    #----------------------------------------------------------------------
	def onChange_upper(self, value):
		self.Vars["upper"] = value
		pickle.dump(self.Vars, open(self.path+".config", "w"))
		rospy.Rate(1).sleep()
        
    #----------------------------------------------------------------------
	def onChange_lower(self, value):
		self.Vars["lower"] = value
		pickle.dump(self.Vars, open(self.path+".config", "w"))
		rospy.Rate(1).sleep()
        
    #----------------------------------------------------------------------
	def onChange_erode(self, value):
		self.Vars["erode"] = value + 1
		pickle.dump(self.Vars, open(self.path+".config", "w"))
		rospy.Rate(1).sleep()
        
    #----------------------------------------------------------------------
	def onChange_dilate(self, value):
		self.Vars["dilate"] = value + 1
		pickle.dump(self.Vars, open(self.path+".config", "w"))
		rospy.Rate(1).sleep()
        
    #----------------------------------------------------------------------
	def onChange_smooth(self, value):
		self.Vars["smooth"] = value + 1
		pickle.dump(self.Vars, open(self.path+".config", "w"))
		rospy.Rate(1).sleep()
        
        
    #----------------------------------------------------------------------
	def run(self):
		#ret, im = self.camera.read()
		in_frame=self._dc.getFrame()
		cv.Resize(in_frame,self.imcolor, cv.CV_INTER_LINEAR)
		im = np.asarray( self.imcolor[:,:])
		im = cv2.flip(im, 1)
		self.imOrig = im.copy()
		self.imNoFilters = im.copy()

        #Aplica smooth
		im = cv2.blur(im, (self.Vars["smooth"], self.Vars["smooth"]))
        
        #Aplica filtro de color de piel
		filter_ = self.filterSkin(im)
 
        #Aplica erode
		filter_ = cv2.erode(filter_,cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(self.Vars["erode"], self.Vars["erode"])))           
        
        #Aplica dilate
		filter_ = cv2.dilate(filter_,cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(self.Vars["dilate"], self.Vars["dilate"])))
      
        
        #Muestra la imagen binaria
		#if self.debugMode: cv2.imshow("Filter Skin", filter_)
        
        #Obtiene contornos
	    #contours = cv2.findContours(filter_,cv2.RETR_LIST,cv2.CHAIN_APPROX_NONE)
		#contours= cv2.findContours(filter_,cv2.RETR_LIST,cv2.CHAIN_APPROX_NONE)
		if self.debugMode:
			h, w = filter_.shape[:2]
			imgS= cv.CreateMat(h, w, cv.CV_8UC1)
			np.asarray(imgS)[:,:] = filter_
			cv.ShowImage('FilteredImage',imgS)
			#cv.Resize(imgS,largeFrame, cv.CV_INTER_LINEAR)
			
			#cv2.imshow("Filter Skin", filter_)
		
        #Elimina Â«islas huÃ©rfanasÂ» de Ã¡rea pequeÃ±a
		allIdex = []
		contours,hierarchy= cv2.findContours(filter_,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)
		#for index in range(len(contours)):
		#	contoursTemp=np.array(contours[index])
		#	area = cv2.contourArea(contoursTemp)
		#	if area < 5e3: 
		#		allIdex.append(index)
		#contours=contourD[1]
		
		index=0
		for contu in contours:
			#contoursTemp=np.array(contours[index])
			#temp=np.array(contu)
			area = cv2.contourArea(contu)
			if area < 5e3: 
				allIdex.append(index)
			index =index+1
		allIdex.sort(reverse=True)
		for index in allIdex:
			contours.pop(index)

        #Si no hay contornos, termine aquÃ­
		if len(contours) == 0: 
			return
		allIdex = []
		index_ = 0
        #Recorremos cada contorno
		for cnt in contours:
			self.Data["area"] = cv2.contourArea(cnt)
            
			tempIm = im.copy()
			tempIm = cv2.subtract(tempIm, im)
            
            #Para hallar convexidades (dedos)
			hull = cv2.convexHull(cnt)
			self.last = None
			self.Data["hulls"] = 0
			for hu in hull:
				if self.last == None: 
					cv2.circle(tempIm, tuple(hu[0]), 10, (0,0,255), 5)
				else:
					distance = self.distance(self.last, tuple(hu[0]))
					if distance > 40:  #Eliminar puntos demaciado juntos
						self.Data["hulls"] += 1
                        #CÃ­rculos rojos
						cv2.circle(tempIm, tuple(hu[0]), 10, (0,0,255), 5)
				self.last = tuple(hu[0])

            #Momento principal, que rigue el control del cursor
			M = cv2.moments(cnt)
			centroid_x = int(M['m10']/M['m00'])
			centroid_y = int(M['m01']/M['m00'])
			cv2.circle(tempIm, (centroid_x, centroid_y), 20, (0,255,255), 10) 
			self.Data["cursor"] = (centroid_x, centroid_y)
            
            #Para hallar la convexidades (espacios entre dedos)
			hull = cv2.convexHull(cnt,returnPoints = False)
			angles = []
			defects = cv2.convexityDefects(cnt,hull)
			if defects == None: 
				return
            
			self.Data["defects"] = 0
			for i in range(defects.shape[0]):
				s,e,f,d = defects[i,0]
				if d > 1000 :
					start = tuple(cnt[s][0])
					end = tuple(cnt[e][0])
					far = tuple(cnt[f][0])
					self.Data["defects"] += 1
					cv2.circle(tempIm,far,5,[0,255,255],-1)  #Marcar lo defectos con puntos amarillos
                    #LÃ­neas entre convexidades y defectos
					cv2.line(tempIm, start, far, [255, 0, 0], 5) 
					cv2.line(tempIm, far, end, [255, 0, 0], 5)
                    #Obtener el Ã¡ngulos que forman las lÃ­neas anteriores
					angles.append(self.angle(far, start, end))
                
            #Filtran Ã¡ngulos menores a 90.
			b = filter(lambda a:a<90, angles)
            
            #Se asume que si son menores de 90 grados, corresponde a un dedo.          
			self.Data["angles less 90"] = len(b)
			self.Data["fingers"] = len(b) + 1
            
            #Para almacenar los Ãºltimos estados de los dedos.
			self.Data["fingers history"].append(len(b) + 1)            
            
			if len(self.Data["fingers history"]) > 10: 
				self.Data["fingers history"].pop(0)                
			self.imOrig = cv2.add(self.imOrig, tempIm)
            
			index_ += 1
                
        #Rellenar el espacio filtrado de la piel menos las Ã¡reas pequeÃ±as de un color verde :)
		cv2.drawContours(self.imOrig,contours,-1,(64,255,85),-1)
        
        #Visualizar el estado anctual de los datos.
		self.debug()
		if self.debugMode:
			h, w = self.imOrig.shape[:2]
			imgO= cv.CreateMat(h, w, cv.CV_8UC3)
			np.asarray(imgO)[:,:] = self.imOrig
			#cv.Resize(imgS,largeFrame, cv.CV_INTER_LINEAR)
			msg=cv_bridge.CvBridge().cv_to_imgmsg(imgO,"bgr8")
			self._gui_pub.publish(msg)
			
			#cv2.imshow("\"Hulk\" Mode", self.imOrig)
		rospy.Rate(1).sleep()
        
        
    #----------------------------------------------------------------------
	def distance(self, cent1, cent2):
		"""Retorna la distancia entre dos puntos."""
		x = abs(cent1[0] - cent2[0])
		y = abs(cent1[1] - cent2[1])
		d = sqrt(x**2+y**2)
		return d
		rospy.Rate(1).sleep()
    
    #----------------------------------------------------------------------
	def angle(self, cent, rect1, rect2):
		"""Retorna el Ã¡ngulo formado entre tres puntos."""
		v1 = (rect1[0] - cent[0], rect1[1] - cent[1])
		v2 = (rect2[0] - cent[0], rect2[1] - cent[1])
		dist = lambda a:sqrt(a[0] ** 2 + a[1] ** 2)
		angle = arccos((sum(map(lambda a, b:a*b, v1, v2))) / (dist(v1) * dist(v2)))
		angle = abs(rad2deg(angle))
		return angle
		rospy.Rate(1).sleep()
        
    #----------------------------------------------------------------------
	def filterSkin(self, im):
		"""Aplica el filtro de piel."""
		UPPER = np.array([self.Vars["upper"], self.Vars["filterUpS"], self.Vars["filterUpV"]], np.uint8)
		LOWER = np.array([self.Vars["lower"], self.Vars["filterDownS"], self.Vars["filterDownV"]], np.uint8)
		hsv_im = cv2.cvtColor(im, cv2.COLOR_BGR2HSV)
		filter_im = cv2.inRange(hsv_im, LOWER, UPPER)
		return filter_im
		rospy.Rate(1).sleep()

    #----------------------------------------------------------------------
	def debug(self):
		"""Imprime mensaje de depuraciÃ³n sobre el video."""
		yPos = 10
		if self.debugMode: self.addText(self.imOrig, "Debug", (yPos, 20))
		pos = 50
		for key in self.Data.keys():
			if self.debugMode: self.addText(self.imOrig, (key+": "+str(self.Data[key])), (yPos, pos))
			pos += 20
		rospy.Rate(1).sleep()
     
    #----------------------------------------------------------------------
#	def updateMousePos(self):
#		"""Actualiza la posisiÃ³n del cursor."""
 #       pos = self.Data["cursor"]
  #      posPre = self.posPre
   #     npos = np.subtract(pos, posPre)
    #    self.posPre = pos
        
   #     if self.Data["fingers"] in [1]:  #SÃ³lo mueve el curson cuando hay un dedo extendido o el puÃ±o cerrado
    ##        try: elf.t.__stop.set()
      #      except: pass
            #Thread para mover el cursor
       #     self.t = threading.Thread(target=self.moveMouse, args=(npos))
        #    self.t.start()
            
    #----------------------------------------------------------------------
#    def interprete(self):
 #       """Interpreta los eventos."""
  #      cont = 3
        #Click Izquierdo, 5 dedos extendidos.
   #     if self.Data["fingers history"][:cont] == [5] * cont:
    #        os.system("xdotool click 1")
    #        self.Data["fingers history"] = [0]  #Elimina el historial de estado de los dedos.
        #Click Izquierdo, 3 dedos extendidos.
    #    elif self.Data["fingers history"][:cont] == [3] * cont:
    #        os.system("xdotool click 3")
    #        self.Data["fingers history"] = [0]
            
    #----------------------------------------------------------------------
    #def moveMouse(self, x, y):
    #    """Mueve el cursor a una posisiÃ³n relativa.
        
     #   Siempre de Â«invocaÂ» desde un thread!!."""
     #   mini = 10
    #    mul = 2
    #    x *= mul
    #    y *= mul
        #Un movimiento fluido...
     #   posy = lambda n:(y/x) * n  #...genera la ecuaciÃ³n de la trayectoria...
     #   stepp = 10
        #... y la recorre en x cada 10px
      #  if x > 0:
       #     for i in range(0, x, stepp): os.system("xdotool mousemove_relative -- %d %d" %(i, posy(i)))    
       # if x < 0:
       #     for i in range(x, 0, stepp): os.system("xdotool mousemove_relative -- %d %d" %(i, posy(i)))
      #  time.sleep(0.2)  #Pausa para Â«suavizarÂ» el movimiento """
		
