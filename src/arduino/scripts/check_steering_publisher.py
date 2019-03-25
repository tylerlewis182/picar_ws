#!/usr/bin/env python

import rospy
import time
from Servo import Servo
from std_msgs.msg import String
from std_msgs.msg import UInt16
		

# main
if __name__=='__main__':

	# init node
	rospy.init_node('check_steering_publisher', anonymous=False)

	# create a publisher: (name of topic to publish to, type of message, queue size which acts like a buffe holds 10 messages)
	pub = rospy.Publisher("/steering_servo_position", UInt16, queue_size=10)

	# create a rate object
	rate = rospy.Rate(50) # Hz

	# create a Servo object
	steering_servo = Servo()

	

	# load parameter (this parameter should be set in launch file)
	runtime = rospy.get_param("runtime", 10.0) # default value is 10 (http://wiki.ros.org/rospy/Overview/Parameter%20Server)
	start_time = time.time()
	elapsed_time = time.time() - start_time

	msg = UInt16()
	# run this node for the amount of time (seconds) specified by 'runtime'
	while elapsed_time < runtime:
		steering_servo.sweep_position()
		
		msg.data = steering_servo.position
		pub.publish(msg)
		rate.sleep()
		elapsed_time = time.time() - start_time


		# for debugging
		#rospy.loginfo("position: {}".format(steering_servo.position))
		#rospy.loginfo("prev    : {}".format(steering_servo.prev_position))
		#rospy.loginfo("---")
		#rospy.loginfo(elapsed_time)

	# reset servo value to middle
	steering_servo.reset()
	msg.data = steering_servo.position
	pub.publish(msg)
	rate.sleep()



#########################################################################

''' 
NOTES:
 1900 == max left 
 1500 == straight
 1100 == max right
'''
