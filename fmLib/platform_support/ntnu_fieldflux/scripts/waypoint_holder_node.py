import rospy,tf
import numpy as np
import time
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
import csv
import sys
import tf.transformations
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
		self.wp_request_topic = rospy.getparam("wp_request_topic",'/fmProcessors/wp_request')
		self.wp_topic = rospy.getparam("waypoint_topic",'/fmProcessors/waypoint')
		self.meas_complete_topic = rospy.getparam("measurement_complete_topic","/fmInformation/meas_comp")
		
		rospy.Subscriber(self.wp_request_topic, Int32, self.on_WP_request)
		rospy.Subscriber(self.meas_complete_topic, Int32, self.on_Measurement_complete)
		self.wp_pub = rospy.Publisher(self.wp_topic,Odometry)
		with open('test.csv','rb') as f:
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
						self.wp_type.append(0)
					elif row[3] == 'Stop':
						self.wp_type.append(1)
					elif row[3] == 'DriveThrough':
						self.wp_type.append(2)
					else:
						self.format_error = True
						print 'row',i,'col 3 is has wrong format:',row[3],'should be Measure,Stop or DriveThrough'
				else:
					print 'ERROR: length of line',i,' in csv should be 3, is', len(row), row
					self.format_error = True
				i =i+1
			if format_error:
				print 'Aborting due to errors in input csv'
				sys.exit(0)
	def on_WP_request(self,msg):
		if msg.data == 0:
			#start Mesurement and wait
			self.wait_for_measurement = True
		if msg.data ==1:
			time.sleep(5); 
			x=x_pos.pop(0)
			y=y_pos.pop(0)
			h=heading.pop(0)
			
			self.wp_quat = createQuaternionMessageFromYaw(h)
			wp = Odometry()
			wp.pose.pose.x=x
			wp.pose.pose.y=y
			wp.pose.orientation = wp_quat
			wp_pub.publish(wp)
		if msg.data ==2:
			x=x_pos.pop(0)
			y=y_pos.pop(0)
			h=heading.pop(0)
			
			self.wp_quat = createQuaternionMessageFromYaw(h)
			wp = Odometry()
			wp.pose.pose.x=x
			wp.pose.pose.y=y
			wp.pose.orientation = wp_quat
			wp_pub.publish(wp)
	def on_Measurement_complete(self,msg):
		x=x_pos.pop(0)
		y=y_pos.pop(0)
		h=heading.pop(0)
			
		self.wp_quat = createQuaternionMessageFromYaw(h)
		wp = Odometry()
		wp.pose.pose.x=x
		wp.pose.pose.y=y
		wp.pose.orientation = wp_quat
		wp_pub.publish(wp)
		
if __name__ == '__main__':
	rospy.init_node('Waypoint_Keeper')
	try:
		node_class = WPWriter();
	except rospy.ROSInterruptExceptrion: pass
			

