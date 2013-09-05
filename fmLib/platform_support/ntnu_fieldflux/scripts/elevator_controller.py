#!/usr/bin/env python
import rospy
import socket
import numpy as np
import time
from sensor_msgs.msg import Joy
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
UDP_IP = "192.168.0.224"
UDP_PORT = 12345		
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
class elevator():
	UDP_IP = "192.168.0.224"
	UDP_PORT = 12345
	def __init__(self):


		self.joy_topic = rospy.get_param("joy_topic","/joy")
		self.odom_topic = rospy.get_param("odom_topic","/fmKnowledge/encoder_odom")
		self.drive_ok_topic = rospy.get_param("drive_ok_topic","/fmInformation/meas_comp")
		self.odom_sub = rospy.Subscriber(self.odom_topic,Odometry,self.on_odom_topic)
		self.joy_sub = rospy.Subscriber(self.joy_topic,Joy,self.on_joy_topic)
		self.drive_ok_pub = rospy.Publisher(self.drive_ok_topic, Bool);
		self.goingUp = False
		self.goingDown = False
		self.drive_ok = True
		self.init_relay()
		self.vehicle_stand_still=True
		rospy.spin()

	def init_relay(self):
		#set all relays of
		sock.sendto("FFE000",(UDP_IP, UDP_PORT))
		sock.sendto("FFE000",(UDP_IP, UDP_PORT))
		time.sleep(0.2)
		self.goingUp=False
		self.goingDown=False

	def elevator_up(self):
		rospy.loginfo("elevator_up")
		if self.goingDown:
			self.init_relay()
		else:
			sock.sendto("FF0101",(UDP_IP,UDP_PORT))
			self.goingUp = True

	def elevator_down(self):
		rospy.loginfo("elevator_down")
		if self.vehicle_stand_still:
			if self.goingUp:
				self.init_relay()
			else:
				sock.sendto("FF0201",(UDP_IP,UDP_PORT))
				self.goingDown = True
				self.drive_ok = False
				self.drive_ok_pub.publish(self.drive_ok)

	def on_odom_topic(self,msg):
		if (msg.twist.twist.linear.x == 0 and msg.twist.twist.linear.y == 0 and msg.twist.twist.linear.z == 0 and msg.twist.twist.angular.x == 0 and msg.twist.twist.angular.y == 0 and msg.twist.twist.angular.z == 0):
			self.vehicle_stand_still = True
		else:
			self.vehicle_stand_still = False
			
		
	def on_joy_topic(self,msg):
		if (msg.axes[7] == 1.0):
			if(self.goingDown):
				self.init_relay()
			else:
				self.elevator_up()
		if (msg.axes[7] == -1.0):
			if(self.goingUp):
				self.init_relay()
			else:
				self.elevator_down()
		if (msg.buttons[10]):
			#To be able to drive after driving the elevator, the user must press RS before driving, this to avvoid driving with the elevator down
			self.drive_ok = True;
			self.drive_ok_pub.publish(self.drive_ok)
		if(msg.buttons[5]==0):
			self.init_relay();
		
#	def updater(self):
#		while not rospy.is_shutdown():

#	def publish_drive_ok():
				

if __name__ == "__main__":
	rospy.init_node("elevator_controller")
	try:
		elev = elevator()
	except rospy.ROSInterruptException: pass

	elev.init_relay()
	
