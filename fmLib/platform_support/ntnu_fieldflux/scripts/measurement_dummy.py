#!/usr/bin/env python
import rospy
import time
from std_msgs.msg import Bool

class measurement_dummy():
	def __init__(self):
		self.meas_comp = Bool();
		self.meas_init_topic = rospy.get_param("init_measurement_topic","/fmInformation/start_measurement");
		self.meas_comp_topic = rospy.get_param("measurement_complete_topic","/fmInformation/measurement_done")
		self.meas_init_sub = rospy.Subscriber(self.meas_init_topic,Bool,self.on_measurement_init)
		self.meas_comp_pub = rospy.Publisher(self.meas_comp_topic,Bool)
		rospy.spin();
	def on_measurement_init(self,msg):
		time.sleep(10);
		rospy.loginfo("starting measurement")
		self.meas_comp.data = True;
		self.meas_comp_pub.publish(self.meas_comp);


if __name__ == '__main__':
	rospy.init_node('Measurement_node')

	node_class = measurement_dummy();



