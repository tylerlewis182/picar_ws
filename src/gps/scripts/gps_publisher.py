#!/usr/bin/env python

import rospy
import serial
from get_serial_ports import get_serial_port 
from std_msgs.msg import String
from std_msgs.msg import Float64 
from sensor_msgs.msg import NavSatFix

if __name__=='__main__':
	rospy.init_node("gps_publisher")
###################################################################


	# global constant variables
	GPS_PORT = get_serial_port('gps') # "/dev/ttyUSB0"
	GPS_BAUDRATE = 9600

	pub = rospy.Publisher("/gps_data", NavSatFix, queue_size=10)
	rate = rospy.Rate(1) # Hz


	# try to open serial port
	try:
		gps = serial.Serial(GPS_PORT, baudrate=GPS_BAUDRATE)
		# print(gps) # Serial<id=0x7fab20d80510, open=True>(port='/dev/ttyUSB0', baudrate=9600, bytesize=8, parity='N', stopbits=1, timeout=None, xonxoff=False, rtscts=False, dsrdtr=False)

	# if serial port fails to open
	except serial.serialutil.SerialException as e:
		print("Error! Could not connect to GPS sensor at port: '{}'".format(GPS_PORT))
		
	except Exception as e:
		print("Error! Something went wrong with GPS sensor.")
		print(e)


	# if serial port opens successfully
	else:
		print("Successfully connected to GPS sensor at port: {}".format(GPS_PORT))
		print("Aquiring signal...")
		print("---")

		# print current latitude and longitude to the console until this script is shutdown (Ctrl+C)
		try:
			while not rospy.is_shutdown():

				# save each NMEA sentence in string 'line'
				line = gps.readline() # line: $GPGSA,A,3,17,06,12,19,28,24,,,,,,,1.71,1.46,0.89*00

				# split string on commas to seperate NMEA items. store items in list 'data'
				data = line.split(",") # data: ['$GPGSA', 'A', '3', '17', '06', '12', '19', '28', '24', '', '', '', '', '', '', '2.48', '1.52', '1.96*03\r\n']
				
				# extract only lines that begin with '$GPRMC'
				if data[0] == "$GPRMC":

					# ensure that the NMEA sentence is valid (A == OK, V == invalid)
					if data[2] == "A":
						
						# store current latitude and longitude (format == Degrees, Decimal Minutes)
						latitude  = float(data[3])
						longitude = float(data[5])

						# print("Latitude:  {}".format(latitude))
						# print("Longitude: {}".format(longitude))
						# print("---")

						# expected print output looks similar to this:
						#
						#		Latitude:  3342.7315
						#		Longitude: 09635.6702
						#		---

						# msg = String()
						# msg.data = "Hi, this is Dan from the Robot News Radio!"
						# pub.publish(msg)
						# rate.sleep()

						navsat = NavSatFix()
						navsat.latitude = latitude
						navsat.longitude = longitude 
						#navsat.altitude = 0.0
						#compass = Float64(90.0)
						pub.publish(navsat)
						rate.sleep()


		except KeyboardInterrupt as e:
			print("KeyboardInterrupt. Program shutting down.")
			print(e)

		except Exception as e:
			print("Error! Something went wrong with GPS sensor.")
			print(e)

###################################################################
	
	rospy.loginfo("'gps_publisher' node was stopped")
