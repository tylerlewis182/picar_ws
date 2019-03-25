#!/usr/bin/env python

import rospy
import time
from Servo import Servo 
from std_msgs.msg import String
from std_msgs.msg import UInt16



'''
TODO: Reverse does not work...

			In order to go in reverse, he ESC requires
			that a reverse signal, followed by a neutral
			signal, followed by a reverse signal be 
			recieved. 

			So don't use the 'sweep_position()' method.
			Just go:

				forward for 2 seconds, pause for 2 seconds,
				reverse for 2 seconds, pause for 2 seconds,
				forward for 2 seconds, pause for 2 seconds, ...

				until 'elapsed_time' > 'runtime'
'''


''' 
NOTES:
 1900 == max left 
 1500 == straight
 1100 == max right
'''
		

# main
if __name__=='__main__':

	# init node
	rospy.init_node('check_throttle_publisher', anonymous=False)

	# create a publisher: (name of topic to publish to, type of message, queue size which acts like a buffe holds 10 messages)
	pub = rospy.Publisher("/throttle_servo_position", UInt16, queue_size=10)

	# create a rate object
	rate1 = rospy.Rate(0.5) # Hz
	rate2 = rospy.Rate(0.5) # Hz

	# create a Servo object
	throttle_servo = Servo()

	# load parameter (this parameter should be set in launch file)
	runcount = rospy.get_param("runcount", 3) # default value is 10 (http://wiki.ros.org/rospy/Overview/Parameter%20Server)
	stopped_speed = rospy.get_param("stopped_speed", 1500) # default value is 10 (http://wiki.ros.org/rospy/Overview/Parameter%20Server)
	forward_speed = rospy.get_param("forward_speed", 1550) # default value is 10 (http://wiki.ros.org/rospy/Overview/Parameter%20Server)
	reverse_speed = rospy.get_param("reverse_speed", 1450) # default value is 10 (http://wiki.ros.org/rospy/Overview/Parameter%20Server)
	iterations = 0

	# make the car go forward and backward 'runcount' times
	msg = UInt16()
	throttle_servo.set_position(1500) # stop
	msg.data = throttle_servo.position
	pub.publish(msg)
	rate1.sleep() 

	while iterations < runcount:
		throttle_servo.set_position(1550) # go forward
		msg.data = throttle_servo.position
		pub.publish(msg)
		rate1.sleep() 

		throttle_servo.set_position(1500) # stop
		msg.data = throttle_servo.position
		pub.publish(msg)
		rate1.sleep() 

		throttle_servo.set_position(1400) # trigger reverse (car won't actually move)
		msg.data = throttle_servo.position
		pub.publish(msg)
		rate2.sleep() 
		throttle_servo.set_position(1500) # trigger reverse (car won't actually move)
		msg.data = throttle_servo.position
		pub.publish(msg)
		rate2.sleep() 

		throttle_servo.set_position(1400) # go reverse
		msg.data = throttle_servo.position
		pub.publish(msg)
		rate1.sleep() 

		throttle_servo.set_position(1500) # stop
		msg.data = throttle_servo.position
		pub.publish(msg)
		rate1.sleep() 

		iterations += 1

	throttle_servo.set_position(1500) # stop
	msg.data = throttle_servo.position
	pub.publish(msg)
		# for debugging
		#rospy.loginfo("position: {}".format(steering_servo.position))
		#rospy.loginfo("prev    : {}".format(steering_servo.prev_position))
		#rospy.loginfo("---")

