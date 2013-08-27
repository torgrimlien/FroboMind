import rospy
import socket
import numpy as np
import time

UDP_IP = "192.168.1.199"
UDP_PORT = 12345

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

def init_relay():
	#set all relays of
	sock.sendto("FFE000",(UDP_IP, UDP_PORT))
	sock.sendto("FFE000",(UDP_IP, UDP_PORT))

def elevator_up():
	init_relay()
	sock.sendto("FF0101",(UDP_IP,UDP_PORT))
def elevator_down():
	init_relay()
	sock.sendto("FF0201",(UDP_IP,UDP_PORT))
if __name__ == "__main__":
	rospy.init_node("elevator_node")
	init_relay()
	
