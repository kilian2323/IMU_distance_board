#!/usr/bin/python3
""" Serial interface between Olimex sbc and arduino nano

Handles bidirectional communications between two devices.

"""


import rclpy

from rclpy.node import Node

from std_msgs.msg import String, Int16
from geometry_msgs.msg import Vector3

import serial
import sys
import struct

first_byte = 3
second_byte = 6
distanceCm = Int16()
roll = 0.0
pitch = 0.0
yaw = 0.0
imu = Vector3() # will be Vector3

port = "/dev/ttyACM0"


class SerialInterface(Node):
	def __init__(self):
		super().__init__('serial_interface')
		self.ser = serial.Serial('/dev/ttyUSB0', 115200, timeout = 0)
		self.publisher_0 = self.create_publisher(Int16, 'Distance', 10)
		self.publisher_1 = self.create_publisher(Vector3, 'IMU_RPY_Deg', 10)
		timer_period = 0.1 # seconds

		#self.timer = self.create_timer(timer_period, self.sendToArduino)
		self.timer2 = self.create_timer(timer_period, self.readFromArduino)


	def readFromArduino(self):
		global distanceCm
		global roll
		global pitch
		global yaw
		global imu
		# if len(sys.argv) < 2:
		# 	print("wrong number of arguments to serial_interface")
		# 	print(sys.argv)
		# 	return
		# while not rospy.is_shutdown():
		data = self.ser.read(9999)
		#self.get_logger().info('Received: "%s"' % data)
		msg = str(data)
		#self.get_logger().info('Received: "%s"' % msg)
		lastIndexStart = msg.rfind("SYNCSYNC")		
		lastIndexStop = msg.rfind("\\n")
		#self.get_logger().info('Start: "%d"' % lastIndexStart)
		#self.get_logger().info('Stop: "%d"' % lastIndexStop)
		resultList = []
		if lastIndexStop > lastIndexStart :
			resultMsg = String()
			resultMsg = msg[lastIndexStart+8 : lastIndexStop]
			#self.get_logger().info('Received: "%s"' % resultMsg)
			resultList = resultMsg.split(';',4)
			if len(resultList) == 4 :
				#self.get_logger().info('Val1: "%s"' % resultList[0])
				#self.get_logger().info('Val2: "%s"' % resultList[1])
				try:
					distanceCm.data = int(resultList[0])
					roll = float(resultList[1])
					pitch = float(resultList[2])
					yaw = float(resultList[3])
					self.get_logger().info('Distance [cm]: "%d"' % distanceCm.data)
					self.get_logger().info('Roll [deg]: "%.2f"' % roll)
					self.get_logger().info('Pitch [deg]: "%.2f"' % pitch)
					self.get_logger().info('Yaw [deg]: "%.2f"' % yaw)
					self.get_logger().info("------------------------")
					imu.x = roll
					imu.y = pitch
					imu.z = yaw
				except ValueError:
					# nothing to do here
					self.get_logger().warning("Invalid message received.")

		self.publisher_0.publish(distanceCm)
		self.publisher_1.publish(imu)



	def sendToArduino(self):

		outbuffer = 'sync'.encode('UTF-8')

		outbuffer += struct.pack('b', first_byte)
		outbuffer += struct.pack('b', second_byte)

		outbuffer += '\n'.encode('UTF-8')

		self.get_logger().info('Sending: "%s"' % outbuffer)

		self.ser.write(outbuffer) # writes to serial port





def main(args=None):
	rclpy.init(args=args)
	serial_interface = SerialInterface()

	rclpy.spin(serial_interface)


	# Destroy the node expilicitly 
	# (optional - otherwise it will be done automatically
	# when the garbage collector destroys the node object)
	serial_interface.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
	main()

