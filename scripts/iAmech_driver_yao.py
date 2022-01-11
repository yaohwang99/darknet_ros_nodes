#!/usr/bin/env python3

import math,time
from math import sin, cos, pi
import rospy
import tf

from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
import pyads

class iAmechROS():
	def __init__(self):
		rospy.init_node('iAmech', log_level=rospy.DEBUG)
		
		# Connection
		PLC_AMS_ID = '192.168.100.100.1.1'
		PLC_IP = '192.168.100.100'
		self.connectToPLC(PLC_IP)
		
		print("Initialize connection to PLC")
		self.plc = pyads.Connection(PLC_AMS_ID, 801, PLC_IP)
		self.plc.open()
		print("Connection built!")
		
		# read voltage
		l_volt = self.plc.read_by_name(".SLAM_L[18]", pyads.PLCTYPE_DINT)
		r_volt = self.plc.read_by_name(".SLAM_R[18]", pyads.PLCTYPE_DINT)
		print("Current voltage is {}v {}v".format(l_volt, r_volt))
		if l_volt < 42 or r_volt < 42:
			print("AMR NEEDS CHARGING")
		
		# Unlock motor
		self.plc.write_by_name(".bSLAM_ServeON", 1, pyads.PLCTYPE_BOOL)
		x = self.plc.read_by_name(".bSLAM_ServeON",pyads.PLCTYPE_BOOL)
		print("set to %s" %x)
		time.sleep(1)
		
		# Cleanup when terminating the node
		rospy.on_shutdown(self.shutdown)
		
		
		# Subscribe to topics	
		# cmd_vel
		rospy.Subscriber("cmd_vel", Twist, self.cmdVelCallback)
		# Odom
		self.rate = float(rospy.get_param("~base_controller_rate", 10))
		self.timeout = rospy.get_param("~base_controller_timeout", 1.0)
		# self.stopped = False
		# not used
		# wheel_radius = 85  
		length = 589  
		self.theta_dot = 0
		self.d_th = 0
		self.t_last = 0
		self.x_dot= 0
		self.y_dot= 0
		self.x = 0
		self.y = 0
		self.theta = 0
		self.left_wheel = 0
		self.right_wheel = 0
		
		now = rospy.Time.now()
		self.then = now
		
		now = rospy.Time.now()    
		self.then = now # time for determining dx/dy
		self.t_delta = rospy.Duration(1.0 / self.rate)
		self.t_next = now + self.t_delta
		self.last_cmd_vel = now
		
		while not rospy.is_shutdown():
			self.poll()
				
		rospy.spin()
		
	def connectToPLC(self, plc_ip):
		pyads.open_port()
		pyads.set_local_address('1.2.3.4.1.1')
		pyads.close_port()

		pyads.open_port()
		pyads.add_route('192.168.100.100.1.1', '192.168.100.100')
		pyads.close_port()

		SENDER_AMS = '1.2.3.4.1.1'
		PLC_IP = '192.168.100.100'
		PLC_USERNAME = 'Administrator'
		PLC_PASSWORD = '1'
		ROUTE_NAME = 'RouteToMyPC'
		HOSTNAME = '192.168.100.191'
		# PLC_AMS_ID = '192.168.100.100.1.1'

		pyads.add_route_to_plc(SENDER_AMS, HOSTNAME, plc_ip, PLC_USERNAME, PLC_PASSWORD, route_name=ROUTE_NAME)
		
		
		
	def cmdVelCallback(self, req):
		self.last_cmd_vel = rospy.Time.now()
		
		x = req.linear.x	# m/s
		th = req.angular.z # rad/s
		
		cms_x = int(x * 100) # convert to m/s to cm/s
		cms_th = int((0.291 * th) * 100) # convert theta from rad/s to cm/s  0.291 is wheeltrack radius in meters
		

		# print(cms_x)
		"""
		if x == 0:
		"""	
		
		if cms_x > 1 or cms_x < -1:
			self.left_wheel = cms_x
			self.right_wheel = cms_x
		
		if cms_th != 0:
			
			self.left_wheel -= cms_th
			self.right_wheel += cms_th
		
		if cms_x == 0 and cms_th == 0:
			self.left_wheel = 0
			self.right_wheel = 0
		# print(f'Wheels vel {self.left_wheel} {self.right_wheel}')
		
		
	
	def poll(self):
		t_now_ros = rospy.Time.now()
		l_v = self.plc.read_by_name(".SLAM_L[2]", pyads.PLCTYPE_DINT)
		r_v = self.plc.read_by_name(".SLAM_R[2]", pyads.PLCTYPE_DINT)
			
		if l_v != 0 and r_v != 0:
			t_now = time.time()
			if t_now - self.t_last > 3.5:
			    self.t_last = time.time()
			    delta_t = t_now - self.t_last
			else:
				delta_t = t_now - self.t_last
			
			if abs(l_v) <= 48:
			    l_v = l_v + 0.5 * l_v * delta_t
			if abs(r_v) <= 48:
			    r_v = r_v + 0.5 * r_v * delta_t
				  
			# print("TIME")
			# print(delta_t)
			# print("----------------")
		    	
			self.x_dot = (l_v + r_v) * cos(self.theta) / 2
			self.y_dot = (l_v + r_v) * sin(self.theta) / 2
			
			self.theta_dot = (1 / 589) * (r_v - l_v)
			
			# print(f'theta_dot {self.theta_dot}')
			self.x += self.x_dot * delta_t
			self.y += self.y_dot * delta_t
			self.theta += (self.theta_dot * delta_t)
			
			self.theta = self.theta % (math.pi * 2)
			# print(f'theta {self.theta}')
			# print(l_v)
			# print(r_v)			print(quaternion.w)
			
			
			print(self.x, self.y)
			x_2 = self.x /100
			y_2 = self.y /100
			quaternion = Quaternion()
			quaternion.x = 0.0
			quaternion.y = 0.0
			quaternion.z = sin(self.theta/2.0)
			quaternion.w = cos(self.theta/2.0)


			
			
			self.t_last = t_now
		else:
			x_2 = self.x /100
			y_2 = self.y /100
			t_now = time.time()
			quaternion = Quaternion()
			quaternion.x = 0.0
			quaternion.y = 0.0
			quaternion.z = sin(self.theta / 2.0)
			quaternion.w = cos(self.theta / 2.0)

			self.t_last = t_now
			
		if t_now_ros > (self.last_cmd_vel + rospy.Duration(self.timeout)):
			self.left_wheel = 0
			self.right_wheel = 0
				
	#change here to inverse the way by add -1
		self.drive(self.left_wheel, self.right_wheel) 
		
	
	def drive(self, left, right):
		self.plc.write_by_name(".SLAM_L[2]", left, pyads.PLCTYPE_DINT)
		self.plc.write_by_name(".SLAM_R[2]", right, pyads.PLCTYPE_DINT)
			
	def stop_drive(self):
		self.plc.write_by_name(".SLAM_L[2]", 0, pyads.PLCTYPE_INT)
		self.plc.write_by_name(".SLAM_R[2]", 0, pyads.PLCTYPE_INT)
	
	#def send_odom(self):
	#	print("test")
	
	def shutdown(self):
		try:
			rospy.loginfo("Stopping robot...")
			# self.plc.write_by_name(".bSLAM_ServeON", 0, pyads.PLCTYPE_BOOL)
			self.plc.close()
			rospy.sleep(2)
		except:
			pass
		rospy.loginfo("Shutting down iAmech node...")



if __name__ == '__main__':
	new_iAmech = iAmechROS()

