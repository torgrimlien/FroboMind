#!/usr/bin/env python
import rospy
import socket
import numpy as np
import time
from sensor_msgs.msg import Joy
from nav_msgs.msg import Odometry

UDP_IP = "192.168.0.224"
UDP_PORT = 12345		
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
class elevator():
	UDP_IP = "192.168.0.224"
	UDP_PORT = 12345
	def __init__(self):


		self.joy_topic = rospy.get_param("joy_topic","/joy")
		self.odom_topic = rospy.get_param("odom_topic","/fmKnowledge/odom_estimate")
		self.odom_sub = rospy.Subscriber(self.odom_topic,Odometry,self.on_odom_topic)
		self.joy_sub = rospy.Subscriber(self.joy_topic,Joy,self.on_joy_topic)
		self.goingUp = False
		self.goingDown = False
		self.init_relay()

		rospy.spin()

	def init_relay(self):
		#set all relays of
		sock.sendto("FFE000",(UDP_IP, UDP_PORT))
		sock.sendto("FFE000",(UDP_IP, UDP_PORT))
		time.sleep(0.2)
		self.goingUp=False
		self.goingDown=False

	def elevator_up(self):
		if self.goingDown:
			self.init_relay()
		sock.sendto("FF0101",(UDP_IP,UDP_PORT))
		self.goingUp = True

	def elevator_down(self):
		if self.goingUp:
			self.init_relay()
		sock.sendto("FF0201",(UDP_IP,UDP_PORT))
		self.goingDown = True

	def on_odom_topic(self,msg):
		self.vehicle_stand_still = (msg.twist.twist.linear == [0,0,0] and msg.twist.twist.angular == [0,0,0])
			
			
		
	def on_joy_topic(self,msg):
		if (msg.buttons[13] == 1):
			if(self.goingDown):
				self.init_relay()
			else:
				self.elevator_up()
		if (msg.buttons[14] == 1):
			if(self.goingUp):
				self.init_relay()
			else:
				self.elevator_down()
		
#	def updater(self):
#		while not rospy.is_shutdown():

			

if __name__ == "__main__":
	rospy.init_node("elevator_controller")
	try:
		elev = elevator()
	except rospy.ROSInterruptException: pass

	elev.init_relay()
	
