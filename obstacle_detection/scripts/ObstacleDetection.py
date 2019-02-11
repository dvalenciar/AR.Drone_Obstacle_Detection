#!/usr/bin/env python

'''
Author: David Valencia 

This code detects an obstacle and publishes a flag
does not move the drone only detects where the obstacle is

This code is based on differents  approaches described in differents papers
Here we just call the funcion with the algorithms
Drone video  + gazebo
'''

import sys
import rospy 
import math
import cv2
import numpy as np

from    sensor_msgs.msg   import Image
from 	cv_bridge 		  import CvBridge 
from 	cv_bridge         import CvBridgeError


from  std_msgs.msg                      import String 
from  Grid  							import draw_regions
from  Salient_Histogram_Backprojection  import *

class ControlVideo:
		
	def __init__(self):


		self.bridge 		 = CvBridge ()
		self.quad_video_sub  = rospy.Subscriber("ardrone/front/image_raw", Image,self.video_callback)
		self.Flag      		 = rospy.Publisher ("Flag_Obstaculo",String,queue_size=1)

	def video_callback(self, data):

			try:
			 self.cv_image =self.bridge.imgmsg_to_cv2(data, "bgr8")

			except CvBridgeError as e:
			   print (e)	

			Nx = 120
			Ny = 213

			cv2.imshow("Drone Image ", self.cv_image)
			draw_regions (self.cv_image,Nx,Ny) # esto para dividir en 3*3 en gazebo

			

			#Salient_Histogram_Backprojection
			small_shape = (90,160)
			frame_small =  cv2.resize(self.cv_image,small_shape[1::-1])

			mask      	 = backprojection_saliency(frame_small)
			segmentation = frame_small*mask[:,:,np.newaxis]
			segmentation =  cv2.resize(segmentation,(640,360))

			self.obstacle_flag(segmentation,Nx,Ny)


			cv2.imshow("Salient Map  Histogram Backprojection", segmentation)
			cv2.imshow("Imagen del Drone a color", self.cv_image)
			cv2.waitKey(3)
				
	def obstacle_flag (self,Mapa,Nx,Ny):


		celdas = []
		
		ThresHold_flag = 50

		for i in range(0, Mapa.shape[0], Nx):

			for j in range(0, Mapa.shape[1], Ny):

				block = Mapa[i:i+Nx,j:j+Ny]

				intensity = np.mean (block)

				celdas.append(intensity)
	

				if intensity > ThresHold_flag:
					cv2.circle(self.cv_image,( (j+(Ny/2)),(i+ (Nx/2) )), 2, (0,0,255), -1)

				else:
					cv2.circle(self.cv_image,( (j+(Ny/2)),(i+ (Nx/2) )), 2, (0,255,0), -1)


		alerta = String() 

		
		Superior_Izq   = celdas[0]
		Superior_Cen   = celdas[1]
		Superior_Der   = celdas[2]

		Izq = celdas[4]
		Cen = celdas[5]
		Der = celdas[6]
		
		Inferior_Izq   = celdas[8]
		Inferior_Cen   = celdas[9]
		Inferior_Der   = celdas[10]

		
		regiones = [celdas[0],celdas[1],celdas[2],celdas[4],celdas[5],celdas[6],celdas[8],celdas[9],celdas[10]]

		colisionRegionCentro    = np.mean([celdas[5],celdas[9]])
		colisionRegionDerecha   = np.mean([celdas[6],celdas[10]])
		colisionRegionIzquierda = np.mean([celdas[4],celdas[8]])

		#print colisionRegionCentro

		if colisionRegionCentro >= 40 :
			print "obstacle center"
			alerta.data = "cen"

		elif colisionRegionDerecha >= 40:
			print "obstacle right"
			alerta.data = "der"

		elif colisionRegionIzquierda >= 40:
			print "obstacle left"
			alerta.data = "izq"

		else:
			print "non obstacle"
			alerta.data = "libre"

		self.Flag.publish(alerta)



def shutdown_callback():
	print "Shutting down position controller."


if __name__ == "__main__":

	rospy.init_node('Video_Obstacle_Drone',anonymous=True)
	ControlVideo()
   
	rospy.on_shutdown(shutdown_callback)
	rospy.sleep(0.01)
	rospy.spin()