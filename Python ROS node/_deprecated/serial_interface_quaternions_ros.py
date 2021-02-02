#!/usr/bin/env python

""" Serial interface node between Arduino and ROS.
Made for Robotics course IAS0220 "Robotite juhtimine ja tarkvara", August 2020, CfB, Kilian Ochs

Receives Serial messages from Arduino and forwards them as ROS messages.
In order for this code to work, Arduino must send Quaternion IMU data.
"""
import rospy
from std_msgs.msg import Int16
from geometry_msgs.msg import PoseStamped
import serial


class SerialInterface():
	port = "/dev/ttyUSB0" # Arduino Nano seems to use ttyUSB0, some other Arduino boards might use ttyAMA0
	baudrate = 115200     # [bps]
	heartbeat = 0.1       # [seconds] How fast we're reading the serial buffer and sending messages over to ROS
	startSignature = "##"
	endSignature = "\n"
	numData = 5           # Number of fields contained in message from Arduino
	seqID = 0
	
	# Class constructor #
	def __init__(self):
		rospy.loginfo('Initializing SerialInterface node.')
		rospy.init_node('SerialInterface', anonymous=True)
		self.ser = serial.Serial(self.port,self.baudrate,parity=serial.PARITY_NONE,stopbits=serial.STOPBITS_ONE,bytesize=serial.EIGHTBITS,timeout = 0)
		rospy.loginfo('Connected to: "%s"' % self.ser.portstr)
		self.pub_distance = rospy.Publisher('distance', Int16, queue_size=10)
		self.pub_imu = rospy.Publisher('imu_quaternion', Pose, queue_size=10)
		self.distanceCm = Int16()
		self.imu = PoseStamped()
		rospy.loginfo('SerialInterface node initialized.')
		
	# Reads serial messages from Arduino and forwards them as ROS messages #
	# Called automatically by the timer at the heartbeat interval #
	def forwardMessage(self):
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
					qx = float(resultList[1])
					qy = float(resultList[2])
					qz = float(resultList[3])
					qw = float(resultList[4])
					rospy.loginfo('Distance [cm]: %d' % self.distanceCm.data)
					rospy.loginfo('Quaternion qx: %.2f' % qx)
					rospy.loginfo('Quaternion qy: %.2f' % qy)
					rospy.loginfo('Quaternion qz: %.2f' % qz)
					rospy.loginfo('Quaternion qw: %.2f' % qw)
					rospy.loginfo("------------------------")
					self.imu.header.seq = self.seqID
					self.seqID += 1
					#self.imu.header.time.sec = time.time_ns()
					# TODO: add time to message header
					self.imu.position.x = 0.0
					self.imu.position.y = 0.0
					self.imu.position.z = 0.0
					self.imu.orientation.x = qx
					self.imu.orientation.y = qy
					self.imu.orientation.z = qz
					self.imu.orientation.w = qw
				except ValueError:
					rospy.logwarn("Received message contains invalid values.")
			else:
				rospy.logwarn("Received message has invalid format.")
		else:
			rospy.logwarn("No valid message found in buffer.")
		self.pub_distance.publish(self.distanceCm)
		self.pub_imu.publish(self.imu)


def main(args=None):
	serialInterface = SerialInterface()
	while(not rospy.is_shutdown()):		
		serialInterface.forwardMessage()
		rospy.sleep(serialInterface.heartbeat)
	serialInterface.ser.close()
	rospy.loginfo('Closed serial port.')	

if __name__ == '__main__':
    main()
