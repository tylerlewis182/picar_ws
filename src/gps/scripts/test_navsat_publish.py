import rospy
import serial
from get_serial_ports import get_serial_port 
from std_msgs.msg import String
from std_msgs.msg import Float64 
from sensor_msgs.msg import NavSatFix

if __name__=='__main__':

	rospy.init_node("gps_publisher")
	pub = rospy.Publisher("/gps_data", NavSatFix, queue_size=10)
	rate = rospy.Rate(1) # Hz

	while not rospy.is_shutdown():
		

		# Set up test data.
		navsat = NavSatFix()
		navsat.latitude = 38.149
		navsat.longitude = -76.432
		navsat.altitude = 30.48
		compass = Float64(90.0)
		pub.publish(navsat)
		rate.sleep()
		

		# msg = String()
		# msg.data = "Hi, this is Dan from the Robot News Radio!"
		# pub.publish(msg)
		# rate.sleep()



		# 				