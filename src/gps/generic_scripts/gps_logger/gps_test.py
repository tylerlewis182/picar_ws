#!/usr/bin/env python

'''

gps_test.py (python2.7)

This script will recieve gps data via serial (USB with TTL converter)
and print out latitude/ longitude information to the console.

HARDWARE LIST:
	
	GPS Sensor    - Adafruit Ultimate GPS Breakout v3 
	TTY Converter - FTDI FT232RL USB to TTL Serial Adapter for 3.3 V and 5 V
	LAPTOP        - Thinkpad P50s with Ubuntu 16.04 installed. 

APPLICATIONS:

	ROS full installation
	Python 2.7
	screen

WIRING:

	GPS Breakout Board -> TTY Converter:
		GND -> GND (jumper wire)
		VIN -> VCC (jumper wire)
		TX  -> RXD (jumper wire)
		RX  -> TXD (jumper wire)

	TTY Converter -> this laptop:
		USB (Micro-B) -> USB (A) (male/male USB cable)


REFERENCES:

	Python GPS Tutorial       - https://www.youtube.com/watch?v=_DuMjcl52BU
	NMEA Sentence Information - aprs.gids.nl/nmea/



SCREEN APPLICATION NOTES:

	Check for open screen sessions:

		screen -ls

	Create new screen session:

		screen [OPTIONAL ARGUMENTS]

	Detach from screen session:

		Ctrl+A, Ctrl+D

	Re-attach to existing screen session:

		screen -r

	Kill current session (must be re-attached):

		Ctrl+A, Shift+:, ...then type: quit, ...then press: Enter



HARDWARE TEST NOTES:

	To test to make sure GPS sensor and TTY converter are working properly,
	connect the GPS Sensor, TTY Converter and this laptop as described in 'WIRING' 
	above. The Red LED on the GPS Sensor should be blinking 1 second on, 1 second off.
	After several minutes, the GPS Sensor will "lock on" to the GPS satalites and the 
	Red LED on the GPS Sensor will change its blinking to 1 second on, 15 seconds off. 

	Check to make sure the GPS Sensor shows up as a device on Ubuntu by entering in bash 
	terminal:

		ls /dev/ttyUSB0    (...you should see '/dev/ttyUSB0' appear in console if device was found)

	Use screen to view the NMEA sentences:

		screen 

	If all is working correctly, you should see output like the following:

		$GPGGA,002256.000,3342.7342,N,09635.6699,W,1,07,1.19,200.8,M,-24.7,M,,*58
		$GPGSA,A,3,28,15,19,17,13,30,01,,,,,,1.45,1.19,0.84*07
		$GPGSV,4,1,15,17,70,316,21,28,62,048,20,19,52,264,28,30,41,168,15*76
		$GPGSV,4,2,15,01,30,047,16,06,24,193,19,13,19,254,13,11,17,046,*71
		$GPGSV,4,3,15,07,09,156,,03,09,094,,22,08,072,,24,07,321,17*7B
		$GPGSV,4,4,15,15,05,284,18,18,05,037,,36,,,*76
		$GPRMC,002256.000,A,3342.7342,N,09635.6699,W,0.84,341.26,070319,,,A*70
		$GPVTG,341.26,T,,M,0.84,N,1.57,K,A*30
		$GPGGA,002257.000,3342.7347,N,09635.6702,W,1,07,1.19,200.8,M,-24.7,M,,*5F
		$GPGSA,A,3,28,15,19,17,13,30,01,,,,,,1.45,1.19,0.84*07
		$GPRMC,002257.000,A,3342.7347,N,09635.6702,W,0.86,337.78,070319,,,A*7F
		$GPVTG,337.78,T,,M,0.86,N,1.58,K,A*37
		$GPGGA,002258.000,3342.7351,N,09635.6701,W,1,07,1.19,200.8,M,-24.7,M,,*54
		$GPGSA,A,3,28,15,19,17,13,30,01,,,,,,1.45,1.19,0.84*07
		$GPRMC,002258.000,A,3342.7351,N,09635.6701,W,0.84,345.95,070319,,,A*70
		$GPVTG,345.95,T,,M,0.84,N,1.56,K,A*3D

	To kill the screen session:

		Ctrl+A, Shift+:, ...type: quit, ...press: Enter





'''




# # check python version at runtime
# import sys
# python_version = sys.version_info[0] + sys.version_info[1]/10.0
# print(python_version)

# NOTE: printing messages will only work correctly if this script is run from a terminal.
import serial
from get_serial_ports import get_serial_port



# global constant variables
GPS_PORT = get_serial_port('gps') # "/dev/ttyUSB0"
GPS_BAUDRATE = 9600


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
		while True:

			# save each NMEA sentence in string 'line'
			line = gps.readline() # line: $GPGSA,A,3,17,06,12,19,28,24,,,,,,,1.71,1.46,0.89*00

			# split string on commas to seperate NMEA items. store items in list 'data'
			data = line.split(",") # data: ['$GPGSA', 'A', '3', '17', '06', '12', '19', '28', '24', '', '', '', '', '', '', '2.48', '1.52', '1.96*03\r\n']
			
			# extract only lines that begin with '$GPRMC'
			if data[0] == "$GPRMC":

				# ensure that the NMEA sentence is valid (A == OK, V == invalid)
				if data[2] == "A":
					
					# store current latitude and longitude (format == Degrees, Decimal Minutes)
					latitude  = data[3]
					longitude = data[5]

					print("Latitude:  {}".format(latitude))
					print("Longitude: {}".format(longitude))
					print("---")

					# expected print output looks similar to this:
					#
					#		Latitude:  3342.7315
					#		Longitude: 09635.6702
					#		---

	except KeyboardInterrupt as e:
		print("KeyboardInterrupt. Program shutting down.")
		print(e)

	except Exception as e:
		print("Error! Something went wrong with GPS sensor.")
		print(e)
