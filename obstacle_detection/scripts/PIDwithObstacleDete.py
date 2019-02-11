#!/usr/bin/env python

"""
Author: David Valencia

Description: Controlde posicion del dron usando PID 
			 aqui si incluye la deteccion y evasion de obstaculos
			 es necesario primero arrancar el nodo ObstacleDetection.py



		
"""
import rospy 
import math
import numpy

from   std_msgs.msg 		 import String 
from   nav_msgs.msg          import Odometry
from   ardrone_autonomy.msg  import Navdata
from   geometry_msgs.msg     import Twist

from   Rotation_Transformacion import *


class LecturePosicion:

	def __init__(self):


		rospy.Subscriber("ground_truth/state", Odometry,self.pose_callback)
		rospy.Subscriber("ardrone/navdata", Navdata,self.ang_callback)
		rospy.Subscriber("position_referencia",Twist,self.droneReferencia)
		rospy.Subscriber("Flag_Obstaculo",String,self.Movimiento)

		self.movePub       = rospy.Publisher ("cmd_vel", Twist, queue_size=1)
		
		self.X_d   = 10.0 #Valores iniciales  
		self.Y_d   = 0.0
		self.Z_d   = 1.0
		self.yaw_d = 0.0
	
	def Movimiento (self,flag):

		self.obstacle   = flag.data  
		


	def droneReferencia(self, data):

		self.X_d   = data.linear.x  #Valores Deseados del punto a donde quiero llegar  
		self.Y_d   = data.linear.y
		self.Z_d   = data.linear.z
		self.yaw_d = data.angular.z

		
	def ang_callback(self, ang_data):

		self.roll_dron  = ang_data.rotX  # angulo roll del dron en degranes
		self.pich_dron  = ang_data.rotY  # angulo pitch del dron en degranes
		self.yaw_dron   = ang_data.rotZ  # angulo yaw del dron en degranes


	def pose_callback(self, pose_data):

		self.posx_dron = pose_data.pose.pose.position.x
		self.posy_dron = pose_data.pose.pose.position.y
		self.posz_dron = pose_data.pose.pose.position.z

		teta =  [ self.roll_dron, self.pich_dron, self.yaw_dron]
		R    =  eulerAnglesToRotationMatrix(teta)	

		x = 0# la posicion del  frame de dron con respecto a las coordenadas en gazebo
		y = 0
		z = 0


		T_inversa = Homogenius_Inversa(R, x,y,z)
		#print T_inversa

		position  = np. array ([ [self.posx_dron],
								 [self.posy_dron],        
								 [self.posz_dron],
								 [1]
							  ])# position en gazebo

		odom = np.dot(T_inversa , position ) # esta es la posicion del dron con  respecto a su propio frame 


		self.x_odom = odom[0]
		self.y_odom = odom[1]
		self.z_odom = odom[2]

		position_deseada = np. array ([ [self.X_d ],
										[self.Y_d ],        
										[self.Z_d ],
										[1]
										])

		positionDesada_odom = np.dot(T_inversa , position_deseada) # posicion del punto desado convertido al frame del dron

		self.xd_odom = positionDesada_odom[0]
		self.yd_odom = positionDesada_odom[1]
		self.zd_odom = positionDesada_odom[2]


		self.calculos()

	def calculos (self):


		
		self.error_x   = self.xd_odom  - self.x_odom
		self.error_y   = self.yd_odom  - self.y_odom  
		self.error_z   = self.zd_odom  - self.z_odom
		self.error_yaw = self.yaw_d    - self.yaw_dron



		PID_X  	= pid_controller(0.4, 0.1, 0.3, 0.9) 
		PID_Y  	= pid_controller(0.1, 0.1, 0.6, 0.9)
		PID_Z  	= pid_controller(0.4,   0.2,  0.2, 1.0) 
		PID_Yaw  = pid_controller(0.02, 0.01, 0.01, 0.7)
	
		
		U1 = PID_Z.set_current_error(self.error_z)
		U2 = PID_X.set_current_error(self.error_x) 
		U3 = PID_Y.set_current_error(self.error_y) 

		U4 = PID_Yaw.set_current_error(self.error_yaw) 

		self.Action(U1,U2,U3,U4)

	def Action (self, U1, U2, U3, U4):

		twist = Twist()


		try:
			if self.obstacle  == "libre":

				twist.linear.y = U3
				twist.linear.x = U2             
				twist.linear.z = U1
				twist.angular.z= U4
				self.movePub.publish(twist)
				print "libre"


			elif  self.obstacle  == "cen":

				twist.linear.y = 1
				twist.linear.x = U2
				twist.linear.z = U1
				twist.angular.z= U4
				self.movePub.publish(twist)

			elif  self.obstacle  == "der":

				twist.linear.y = 1
				twist.linear.x = U2
				twist.linear.z = U1
				twist.angular.z= U4
				self.movePub.publish(twist)


			elif  self.obstacle  == "izq":

				twist.linear.y = -1
				twist.linear.x = U2
				twist.linear.z = U1
				twist.angular.z= U4
				self.movePub.publish(twist)

		except :
			   print "no se recibe info de obstaculo"    

		self.obstacle = " "


class pd_controller:

	def __init__(self, p_coef, d_coef, limit_out):
		
		self.kp = p_coef
		self.kd = d_coef

		self._limit_out 	 = limit_out
		self._previous_error = 0.0

	def set_current_error(self, error):

		output = error * self.kp
		error_diff = error - self._previous_error
		output += self.kd * error_diff
		self._previous_error = error

		if output > self._limit_out:
			output = self._limit_out
		elif output < (-self._limit_out):
			output = (-self._limit_out)
			
		return output

class pid_controller:

	def __init__(self, p_coef, i_coef, d_coef, limit_out):
		
		self.kp = p_coef
		self.ki = i_coef
		self.kd = d_coef

		self._limit_out 	 = limit_out
		self._previous_error = 0.0


	def set_current_error(self, error):

		output0    = error * self.kp

		error_diff = error - self._previous_error
		outpu1     = self.kd * error_diff
		
		error_intr = error + self._previous_error
		outpu2     = self.ki * error_intr

		self._previous_error = error

		output = output0 + outpu1 + outpu2 
		

		if output > self._limit_out:
			output = self._limit_out
		elif output < (-self._limit_out):
			output = (-self._limit_out)
			
		return output


def shutdown_callback():
	print "Shutting down position controller."


if __name__ == "__main__":

	rospy.init_node('position_controller_drone_PID',anonymous=True)
	LecturePosicion()
	rospy.on_shutdown(shutdown_callback)
	rospy.sleep(0.01)
	rospy.spin()
	


