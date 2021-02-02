#!/usr/bin/env python

""" Serial interface node between Arduino and ROS.

Receives Serial messages from Arduino and forwards them as ROS messages.
Made for Robotics course IAS0220 "Robotite juhtimine ja tarkvara", August 2020, CfB, Kilian Ochs

"""
import rospy
from std_msgs.msg import Int16
from geometry_msgs.msg import Vector3
import serial


class SerialInterface():
	port = "/dev/ttyUSB0" # Arduino Nano seems to use ttyUSB0, some other Arduino boards might use ttyAMA0
	baudrate = 115200     # [bps]
	heartbeat = 0.1       # [seconds] How fast we're reading the serial buffer and sending messages over to ROS
	startSignature = "##"
	endSignature = "\n"
	
	# Class constructor #
	def __init__(self):
		rospy.loginfo('Initializing SerialInterface node.')
		rospy.init_node('SerialInterface', anonymous=True)
		self.ser = serial.Serial(self.port,self.baudrate,parity=serial.PARITY_NONE,stopbits=serial.STOPBITS_ONE,bytesize=serial.EIGHTBITS,timeout = 0)
		rospy.loginfo('Connected to: "%s"' % self.ser.portstr)
		self.pub_distance = rospy.Publisher('distance', Int16, queue_size=10)
		self.pub_imu = rospy.Publisher('imu_rpy_deg', Vector3, queue_size=10)
		self.distanceCm = Int16()
		self.imu = Vector3()
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
			resultList = resultMsg.split(';',4)
			if len(resultList) == 4 :
				try:
					self.distanceCm.data = int(resultList[0])
					roll = float(resultList[1])
					pitch = float(resultList[2])
					yaw = float(resultList[3])
					rospy.loginfo('Distance [cm]: %d' % self.distanceCm.data)
					rospy.loginfo('Roll [deg]: %.2f' % roll)
					rospy.loginfo('Pitch [deg]: %.2f' % pitch)
					rospy.loginfo('Yaw [deg]: %.2f' % yaw)
					rospy.loginfo("------------------------")
					self.imu.x = roll
					self.imu.y = pitch
					self.imu.z = yaw
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
