#!/usr/bin/env python

import rospy
import time
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




# servo class
''' 
NOTES:
 1900 == max left 
 1500 == straight
 1100 == max right
'''
# servo class
class Servo:
	def __init__(self, mid_position=1500, max_position=1550, min_position=1450):

		self.position = mid_position
		self.prev_position = self.position - 1 
		self.mid_position = mid_position
		self.max_position = max_position
		self.min_position = min_position
		self.direction = 'ccw'
		self.step_size = 5 # servo adjustments step by this amount
		
	
	def sweep_position(self): 
		''' increments or decrements servo value by 1 '''

		# if at max_left, swap current and previous positions
		if self.position >= self.max_position:
			self.position -= self.step_size
			self.prev_position += self.step_size
			self.direction = 'cw'

		# if at max_right, swap current and previous positions
		elif self.position <= self.min_position:
			self.position += self.step_size
			self.prev_position -= self.step_size
			self.direction = 'ccw'

		# if turning servo clockwise
		elif self.direction == 'cw':
			self.position -= self.step_size
			self.prev_position -= self.step_size

		# if turning servo counter clockwise
		elif self.direction == 'ccw':
			self.position += self.step_size
			self.prev_position += self.step_size

	def reset(self):
		self.position = self.mid_position
		self.prev_position = self.position - 1 
		self.direction = 'ccw'
		

# main
if __name__=='__main__':

	# init node
	rospy.init_node('check_throttle_publisher', anonymous=False)

	# create a publisher: (name of topic to publish to, type of message, queue size which acts like a buffe holds 10 messages)
	pub = rospy.Publisher("/throttle_servo_position", UInt16, queue_size=10)

	# create a rate object
	rate = rospy.Rate(10) # Hz

	# create a Servo object
	throttle_servo = Servo()

	# load parameter (this parameter should be set in launch file)
	runtime = rospy.get_param("runtime", 10.0) # default value is 10 (http://wiki.ros.org/rospy/Overview/Parameter%20Server)
	start_time = time.time()
	elapsed_time = time.time() - start_time

	# while this node is running, sweep the steering back and forth
	while not rospy.is_shutdown():
		throttle_servo.sweep_position()
		msg = UInt16()
		msg.data = throttle_servo.position
		pub.publish(msg)
		rate.sleep()

		# for debugging
		#rospy.loginfo("position: {}".format(steering_servo.position))
		#rospy.loginfo("prev    : {}".format(steering_servo.prev_position))
		#rospy.loginfo("---")

