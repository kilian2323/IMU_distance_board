#!/usr/bin/env python
import rospy

from std_msgs.msg import Bool
from tut_arrows_msgs.msg import Flipper
import serial
import struct
import math
import sys

messagelength = 49


def extractMessage(buf):
    ros_messages = []
    messagestart = buf.find('SYNCSYNC')
    if len(buf) > 1024: # limit buffer length
        return buf[-1024:], ros_messages
    if messagestart == -1: #no sync
        return buf, ros_messages
#     print buf[:messagestart]
    if len(buf) - messagestart < messagelength: #only beginning found
        return buf[messagestart:], ros_messages
    message = buf[messagestart + len('SYNCSYNC') : messagestart + messagelength - 1]
    for motor in range(4):
        flipper = Flipper()
        flipper.motorNumber = motor
        flipper.frequency = frequency = struct.unpack('f',message[motor*10:motor*10+4])[0]
        flipper.zeroDirection = zero = int(message[motor*10+4+2-1:motor*10+4-1:-1].encode('hex'),16)/1024.0*2*math.pi
        flipper.amplitude = amplitude = int(message[motor*10+6+2-1:motor*10+6-1:-1].encode('hex'),16)/1024.0*2*math.pi
        flipper.phaseOffset = phase = int(message[motor*10+8+2-1:motor*10+8-1:-1].encode('hex'),16)/1024.0*2*math.pi
        ros_messages.append(flipper)
        #print "Motor", motor, ":", frequency, zero, amplitude, phase
    return buf[messagestart + messagelength : ], ros_messages



def arduino():
    if len(sys.argv) < 2:
        #print "Wrong number of arguments to arduino_interface"
        #print sys.argv
        return
    pub = rospy.Publisher('motors', Flipper)
    # rospy.init_node('arduino')
    # rospy.Subscriber("imu", Imu, imu_callback)
    # rospy.Subscriber("pressure", Pressue, pressure_callback)
    # rospy.Subscriber("leftsensor", Bool, leftsensor_callback)
    # rospy.Subscriber("rightsensor", Bool, rightsensor_callback)
    port = sys.argv[1]
    ser = serial.Serial(port, 115200, timeout=0)
    buffer = r''
    while not rospy.is_shutdown():
        data = ser.read(9999)
        messages = []
        if len(data) > 0:
#             print data.encode('hex')
            buffer += data
            while True:
                old_length = len(buffer) 
                buffer, messages = extractMessage(buffer)
                if len(messages) == 4:
                    for msg in messages:
                        pub.publish(msg)
                if len(buffer) == old_length:
                    break
        outbuffer = r'syncsync'

        outbuffer += struct.pack('B', cameraFlag)
        outbuffer += '\n'
        ser.write(outbuffer)

        rospy.sleep(0.01)


if __name__ == '__main__':
    try:
        arduino()
    except rospy.ROSInterruptException:
        pass
