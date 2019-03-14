#!/usr/bin/env python

import rospy
import serial
from get_serial_ports import get_serial_port 
from std_msgs.msg import String
from std_msgs.msg import Float64 
from sensor_msgs.msg import NavSatFix


# functions
def callback_receive_gps_data(fix):
	rospy.loginfo("GPS fix received : ")
	rospy.loginfo(fix)




# main
if __name__=='__main__':
	rospy.init_node("gps_subscriber")

	# create a subscriber
	sub = rospy.Subscriber("/gps_data", NavSatFix, callback_receive_gps_data)

	# wait for messages on topic, go to callback function when new messages arrive
	rospy.spin()

