#!/usr/bin/env python
import rospy
import socket
import numpy as np
import time
from sensor_msgs.msg import Joy
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from std_msgs.msg import Int32


UDP_IP = "192.168.0.224"
UDP_PORT = 12345		
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

STANDSTILL = 0;
GOING_UP = 1;
GOING_DOWN = 2;
#elevator positions
MIDDLE = 0;
TOP = 1;
BOTTOM = 2;
TOP_AND_BOTTOM = 3;
#elevator states in measurement loop
NONE = 0;
WAITING_FOR_ELEV_DOWN = 1;
WAITING_FOR_MEAS_COMPLETE = 2;
WAITING_FOR_ELEV_UP = 3;
class elevator():
	UDP_IP = "192.168.0.224"
	UDP_PORT = 12345
	#elevator directions

	def __init__(self):


		self.joy_topic = rospy.get_param("joy_topic","/joy")
		self.odom_topic = rospy.get_param("odom_topic","/fmKnowledge/encoder_odom")
		self.drive_ok_topic = rospy.get_param("drive_ok_topic","/fmInformation/drive_ok")
		self.end_switch_topic = rospy.get_param("end_switch_topic","/fmInformation/end_switch")
		self.start_meas_topic = rospy.get_param("start_measurement_topic","fmInformation/start_measurement")
		self.measurement_complete_sub_topic = rospy.get_param("measurement_complete_sub_topic","/fmInformation/measurement_done")
		self.init_meas_topic = rospy.get_param("init_measurement_topic","/fmInformation/initiate_measurement")
		self.init_meas_sub = rospy.Subscriber(self.init_meas_topic,Bool,self.on_initiate_measurement)
		self.odom_sub = rospy.Subscriber(self.odom_topic,Odometry,self.on_odom_topic)
		self.end_switch_sub = rospy.Subscriber(self.end_switch_topic, Int32, self.on_elevator_end_switch)
		self.joy_sub = rospy.Subscriber(self.joy_topic,Joy,self.on_joy_topic)
		self.drive_ok_pub = rospy.Publisher(self.drive_ok_topic, Bool);
		self.meas_ok_sub = rospy.Subscriber(self.measurement_complete_sub_topic,Bool,self.on_measurement_complete);
		self.start_meas_pub = rospy.Publisher(self.start_meas_topic,Bool)
		self.elev_down_succeded = 0
		self.elevator_dir = STANDSTILL
		self.elevator_pos = MIDDLE
		self.meas_state = NONE		
		self.drive_ok = Bool()
		self.drive_ok.data=True
		self.start_meas = Bool()
		self.start_meas.data = False
		self.init_relay()
		self.vehicle_stand_still=Bool()
		self.vehicle_stand_still.data=True		
		rospy.spin()

	def init_relay(self):
		#set all relays of
		rospy.loginfo("elevator_init")
		sock.sendto("FFE000",(UDP_IP, UDP_PORT))
		sock.sendto("FFE000",(UDP_IP, UDP_PORT))
		time.sleep(0.2)
		self.elevator_dir = STANDSTILL

	def elevator_up(self):
		rospy.loginfo("elevator_up")
		if self.elevator_pos == MIDDLE or self.elevator_pos == BOTTOM:
			sock.sendto("FF0101",(UDP_IP,UDP_PORT))
			self.elevator_dir = GOING_UP
			time.sleep(0.1)

	def elevator_down(self):
		rospy.loginfo("elevator_down")
		if self.vehicle_stand_still and (self.elevator_pos == TOP or self.elevator_pos == MIDDLE):
			rospy.loginfo("elev_down if ok")
			sock.sendto("FF0201",(UDP_IP,UDP_PORT))
			self.elevator_dir = GOING_DOWN
			self.drive_ok.data = False
			self.drive_ok_pub.publish(self.drive_ok)
			self.elev_down_succeded = 1

	def on_odom_topic(self,msg):
		if (msg.twist.twist.linear.x == 0 and msg.twist.twist.linear.y == 0 and msg.twist.twist.linear.z == 0 and msg.twist.twist.angular.x == 0 and msg.twist.twist.angular.y == 0 and msg.twist.twist.angular.z == 0):
			self.vehicle_stand_still = True
		else:
			self.vehicle_stand_still = False
			
		
	def on_joy_topic(self,msg):
		if (msg.axes[7] == 1.0):
			if(self.elevator_dir == GOING_DOWN):
				self.init_relay()
			else:
				self.elevator_up()
		if (msg.axes[7] == -1.0):
			if(self.elevator_dir == GOING_UP):
				self.init_relay()
			else:
				self.elevator_down()
		if (msg.buttons[10]):
			#To be able to drive after driving the elevator, the user must press RS before driving, this to avvoid driving with the elevator down
			self.drive_ok.data = True;
			self.drive_ok_pub.publish(self.drive_ok)
		if(msg.buttons[5]==0):
			self.init_relay();
	def on_initiate_measurement(self,msg):
		rospy.loginfo("initiate_measurement")
		if self.elev_down_succeded ==0:
			rospy.loginfo("measurement_initiated = 0")
			time.sleep(1)
			self.meas_state = WAITING_FOR_ELEV_DOWN;
			self.start_meas.data = True;
			self.init_relay();
			self.elevator_down();
			self.measurement_initiated = 1;
			
	def on_measurement_complete(self,msg):
		rospy.loginfo("on_meas_complete")
		self.elevator_up();
		self.meas_state = WAITING_FOR_ELEV_UP
	def on_elevator_end_switch(self,msg):

		if msg.data == 0x04:
			rospy.loginfo("0x04")
			self.init_relay();
			self.elevator_pos = TOP_AND_BOTTOM;
		if msg.data == 0x05:
			rospy.loginfo("0x05")
			if self.elevator_dir == GOING_UP:
				self.init_relay();
			self.elevator_pos = TOP
			if self.meas_state == WAITING_FOR_ELEV_UP:
				self.drive_ok.data = True
				self.drive_ok_pub.publish(self.drive_ok)
				self.meas_state = NONE
				self.elev_down_succeded = 0
		if msg.data == 0x06:
			rospy.loginfo("0x06")
			if self.elevator_dir == GOING_DOWN:
				self.init_relay();
			if self.meas_state == WAITING_FOR_ELEV_DOWN:
				self.start_meas.data = True
				self.start_meas_pub.publish(self.start_meas)
				rospy.loginfo("dummy_measurement_started")
				self.start_meas.data = False
				self.meas_state = WAITING_FOR_MEAS_COMPLETE
				
			
		if msg.data == 0x07:
			#rospy.loginfo("0x07")
			self.elevator_pos = MIDDLE;
		
#	def updater(self):
#		while not rospy.is_shutdown():

#	def publish_drive_ok():
				

if __name__ == "__main__":
	rospy.init_node("elevator_controller")
	try:
		elev = elevator()
	except rospy.ROSInterruptException: pass

	elev.init_relay()
	
