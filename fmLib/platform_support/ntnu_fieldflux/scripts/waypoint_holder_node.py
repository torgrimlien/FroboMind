#!/usr/bin/env python
import rospy,tf
import numpy as np
import time
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Bool
import csv
import sys
from tf.transformations import quaternion_from_euler
def is_number(s):
	try:
		float(s)
		return True
	except ValueError:
		return False

class WPWriter():

	
	def __init__(self):
		self.x_pos=[]
		self.y_pos=[]
		self.heading=[]
		self.wp_type=[]
		self.wq_quat=np.empty((4, ),dtype=np.float64)
		self.format_error= False
		self.wait_for_measurement = False
		self.measurement_complete = False
		self.wp_request_topic = rospy.get_param("wp_request_topic",'/fmInformation/goal_reached')
		self.wp_topic = rospy.get_param("waypoint_topic",'/fmInformation/waypoint')
		self.meas_complete_topic = rospy.get_param("measurement_complete_topic","/fmInformation/drive_ok")
		self.initiate_measurement_topic = rospy.get_param("init_measurement_topic","/fmInformation/initiate_measurement")
		self.current_waypoint_type = ''
		self.wait_for_drive_ok = False
		rospy.Subscriber(self.wp_request_topic, Bool, self.on_WP_request)
		rospy.Subscriber(self.meas_complete_topic, Bool, self.on_Measurement_complete)
		self.wp_pub = rospy.Publisher(self.wp_topic,Odometry)
		self.meas_init_pub = rospy.Publisher(self.initiate_measurement_topic, Bool);

#		rospy.loginfo("All subscribers initialized")
		with open('demoApelsvoll.csv','rb') as f:
#			rospy.loginfo("csv file opened")
			reader = csv.reader(f);
			i=1;
			
			for row in reader:
				if len(row) == 4:
					if is_number(row[0]):
						self.x_pos.append(row[0])
					else:
						self.format_error = True
						print 'row',i,'col 0 is not numeric:',row[0]
					if is_number(row[1]):			
						self.y_pos.append(row[1])
					else:
						self.format_error = True
						print 'row',i,'col 2 is not numeric:',row[1]
					if is_number(row[2]):
						self.heading.append(row[2])
					else:
						self.format_error = True
						print 'row',i,'col 2 is not numeric:',row[2]
					if row[3]=='Measure':
						self.wp_type.append(row[3])
					elif row[3] == 'Stop':
						self.wp_type.append([3])
					elif row[3] == 'DriveThrough':
						self.wp_type.append(row[3])
					else:
						self.format_error = True
						print 'row',i,'col 3 is has wrong format:',row[3],'should be Measure,Stop or DriveThrough'
				else:
					print 'ERROR: length of line',i,' in csv should be 3, is', len(row), row
					rospy.loginfo("Error in input csv file")
					self.format_error = True
				i =i+1
			if self.format_error:
				print 'Aborting due to errors in input csv'
				rospy.loginfo("Aborted due to errors in input csv")
				sys.exit(0)
		time.sleep(5)
		self.publish_waypoint();
#		rospy.loginfo("Waypoint published")
		rospy.spin()
	def on_WP_request(self,msg):
		rospy.loginfo("Waypoint requested")
		if self.current_waypoint_type == 'Measure':
			self.wait_for_measurement = True
			self.initiate_Measurement();

		if self.current_waypoint_type == 'DriveThrough':
			self.publish_waypoint()

		if self.current_waypoint_type == 'Stop':
			self.wait_for_drive_ok = True

	def on_Measurement_complete(self,msg):
		rospy.loginfo("Measurement complete")
		self.measurement_complete = msg.data;
		if self.wait_for_measurement == True and self.measurement_complete == True:
			self.wait_for_measurement = False
			rospy.loginfo("publishing waypoint")
			self.publish_waypoint()

	def initiate_Measurement(self):
		im = Bool()
		im.data = True
		self.meas_init_pub.publish(im);
		self.wait_for_measurement = True;	
	
	def publish_waypoint(self):

		x=self.x_pos.pop(0)
		y=self.y_pos.pop(0)
		h=self.heading.pop(0)
		self.current_waypoint_type = self.wp_type.pop(0);
		self.wp_quat = quaternion_from_euler(0,0,float(h))
		print x,y,h,self.wp_quat
		wp = Odometry()
		wp.pose.pose.position.x=float(x)
		wp.pose.pose.position.y=float(y)
		wp.pose.pose.position.z=0
		wp.pose.pose.orientation = Quaternion(self.wp_quat[0],self.wp_quat[1],self.wp_quat[2],self.wp_quat[3])
		self.wp_pub.publish(wp) 

if __name__ == '__main__':
	rospy.init_node('Waypoint_Keeper')
	#try:
	rospy.loginfo("Node initialized")
	node_class = WPWriter();
	#except rospy.ROSInterruptExceptrion: pass
			

