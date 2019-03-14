'''

get_serial_ports.py

This script will scan USB ports on this computer and attempt 
to find the port that corresponds to Adafruit Ultimate GPS Breakout 
Board v3 and it's TTL converter.  

NOTES:

	useful bash commands:

		ty@thinkpad:~$ ls -l /dev/serial/by-id
		total 0
		lrwxrwxrwx 1 root root 13 Mar  7 13:17 usb-FTDI_FT232R_USB_UART_AI05G1RK-if00-port0 -> ../../ttyUSB0
		ty@thinkpad:~$ 

'''

import subprocess
import os.path

# device IDs
device_ids = dict()
device_ids['ttl_converter'] = "usb-FTDI_FT232R_USB_UART_AI05G1RK-if00-port0"
device_ids['gps']           = "usb-FTDI_FT232R_USB_UART_AI05G1RK-if00-port0"
device_ids['arduino']       = "usb-Arduino__www.arduino.cc__0042_95530343934351205142-if00"


'''------------------------------------------------------------'''


# get all ports
def get_serial_ports():

	fname='/dev/serial/by-id'
	cmd=['ls -l {}'.format(fname)]
	num_devices_found = 0


	# return a dictionary containing device IDs and their associated ports
	serial_ports = dict()


	# check if there are any devices connected to this computer
	file_exists = os.path.exists(fname)

	# if there are NOT any devices connected
	if not file_exists:
		#print("{} devices found".format(num_devices_found))
		return serial_ports



	# if there are devices connected
	else:

		# save output of shell command 'ls -l /dev/serial/by-id'
		output = subprocess.check_output(cmd, shell=True)

		# split the string output on every newline
		devices = output.split('\n')
		devices.pop(0) # remove first item from list (...it is always 'total 0')
		devices.pop()  # remove last  item from list (...it is always an empty line)
		num_devices_found = len(devices) # count number of valid devices found
		#print("{} devices found".format(num_devices_found))

		# store device_id, port in dict
		serial_ports = dict()

		for i, line in enumerate(devices):
			key = line.split(' ')[8]
			val = line.split(' ')[10].split('/')[:1:-1]
			val.insert(0, '/dev/')
			val = "".join(val)

			# print(key) # example: usb-FTDI_FT232R_USB_UART_AI05G1RK-if00-port0
			# print(val) # example: /dev/ttyUSB0

			serial_ports[key] = val


	return serial_ports # dictionary


# get a specific port
def get_serial_port(device):
	device_id = device_ids[device]

	serial_ports = get_serial_ports()

	# check to see if 'device_id' is in the dict. if so, return the port
	for k, v in serial_ports.iteritems():
		if k == device_id:
			return serial_ports[device_id]

	# if 'device_id' is not in dict, return None
	return None



		



'''------------------------------------------------------------'''


if __name__=='__main__':
	serial_ports = get_serial_ports()
	print(serial_ports)









