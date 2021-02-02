#!/usr/bin/env python

""" Serial interface node between Arduino and ROS.
Made for Robotics course IAS0220 "Robotite juhtimine ja tarkvara", August 2020, CfB, Kilian Ochs

Receives Serial messages from Arduino and forwards them as ROS messages.
The code receives distance and IMU Euler angles from Arduino and publishes them as Integer and PoseStamped (Quaternion) messages.
"""
import rospy
from std_msgs.msg import Int16
from sensor_msgs.msg import Imu
import serial
import time
#import tf2_ros
import tf_conversions
import math

class SerialInterface():
	port = "/dev/ttyUSB0" # Arduino Nano seems to use ttyUSB0, some other Arduino boards might use ttyAMA0
	baudrate = 115200     # [bps]
	heartbeat = 0.1       # [seconds] How fast we're reading the serial buffer and sending messages over to ROS
	startSignature = "##"
	endSignature = "\n"
	numData = 4           # Number of fields contained in message from Arduino
	seqID = 0
	
	# Class constructor #
	def __init__(self):
		rospy.loginfo('Initializing SerialInterface node.')
		rospy.init_node('SerialInterface', anonymous=True)
		self.ser = serial.Serial(self.port,self.baudrate,parity=serial.PARITY_NONE,stopbits=serial.STOPBITS_ONE,bytesize=serial.EIGHTBITS,timeout = 0)
		rospy.loginfo('Connected to: "%s"' % self.ser.portstr)
		self.pub_distance = rospy.Publisher('distance', Int16, queue_size=10)
		self.pub_imu = rospy.Publisher('imu', Imu, queue_size=10)
		self.distanceCm = Int16()
		self.imu = Imu()
		rospy.loginfo('SerialInterface node initialized.')
		
	# Reads serial messages from Arduino and forwards them as ROS messages #
	def bridge(self):
		data = self.ser.read(9999)
		msg = str(data)		
		lastIndexStart = msg.rfind(self.startSignature)		
		lastIndexStop = msg.rfind(self.endSignature)
		resultList = []
		if lastIndexStop > lastIndexStart :
			resultMsg = msg[lastIndexStart+len(self.startSignature) : lastIndexStop]
			rospy.loginfo('Parsed data: "%s"' % resultMsg)
			resultList = resultMsg.split(';',self.numData)
			if len(resultList) == self.numData :
				try:
					self.distanceCm.data = int(resultList[0])
					roll = float(resultList[1])
					pitch = float(resultList[2])
					yaw = float(resultList[3])	
					roll_rad = math.radians(roll)
					pitch_rad = math.radians(pitch)
					yaw_rad = math.radians(yaw)
					q = tf_conversions.transformations.quaternion_from_euler(roll_rad, pitch_rad, yaw_rad)
					self.imu.orientation.x = q[0]
					self.imu.orientation.y = q[1]
					self.imu.orientation.z = q[2]
					self.imu.orientation.w = q[3]
					rospy.loginfo('  Distance [cm]: %d' % self.distanceCm.data)
					rospy.loginfo('  Roll [deg]: %.2f' % roll)
					rospy.loginfo('  Pitch [deg]: %.2f' % pitch)
					rospy.loginfo('  Yaw [deg]: %.2f' % yaw)
					rospy.loginfo('  qx: %.2f' % q[0])
					rospy.loginfo('  qy: %.2f' % q[1])
					rospy.loginfo('  qz: %.2f' % q[2])
					rospy.loginfo('  qw: %.2f' % q[3])
					rospy.loginfo("------------------------")

				except ValueError:
					rospy.logwarn("Received message contains invalid values.")
			else:
				rospy.logwarn("Received message has invalid format.")
		else:
			rospy.logwarn("No valid message found in buffer.")
		self.imu.header.seq = self.seqID					
		self.imu.header.stamp.secs = time.time()
		self.imu.header.frame_id = "map"
		self.seqID += 1
		self.pub_distance.publish(self.distanceCm)
		self.pub_imu.publish(self.imu)


def main(args=None):
	serialInterface = SerialInterface()
	while(not rospy.is_shutdown()):		
		serialInterface.bridge()
		rospy.sleep(serialInterface.heartbeat)
	serialInterface.ser.close()
	rospy.loginfo('Closed serial port.')	

if __name__ == '__main__':
    main()
