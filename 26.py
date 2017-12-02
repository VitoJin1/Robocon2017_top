# -*- coding: UTF-8 -*-
'''定位模块消息改成节点发布'''
'''时间２０１７／２／２６　晋涛'''
import BaseTalker
import BaseListener
import rospy
import os
import time
import numpy as np
from multiprocessing import Pool, Manager

# message type
from sensor_msgs.msg import *
from beginner_tutorials.msg import *
from communication.msg import *
# -*- coding: UTF-8 -*-
import sys
import serial

ser = serial.Serial('/dev/ttyUSB0',baudrate=230400, bytesize=8, parity=serial.PARITY_NONE, stopbits=1)
ser.flushInput()
ser.flushOutput()
count=0
talker = None
listener = None
def sendData(data):
    ser.write(data)

def getInfo(l):
	if l[0] == '0xd' and l[1] == '0xa' and l[26] == '0xa' and l[27] == '0xd':
		return l[2:25]
	# print l
	return None

def stringToInt(s):
	ans = []
	for _ in s:
		ans.append(hex(ord(_)))
	return ans
	
def intToDouble(int_6):
	import struct
	# print int_6
	return struct.unpack('ffffff', int_6)

def callback(data, queue):
     queue.put(data.str)
     rospy.loginfo(str(data.str))
     time.sleep(0.5)

def talking(queue):
    global talker
    talker = BaseTalker.BaseTalker(nodeName='Chassis_Talk', topic='CHASSIS', msgType=chassis_msg)
    while(True):
        mMsg = chassis_msg()
        mMsg.chassis =get_rawValue()
        rospy.loginfo("healkdhfahdsfahsdfjkh")
        talker.talk(content=mMsg)
	return talker
 
def main():
	'''
    manager = Manager()
    queue = manager.Queue()
    pool = Pool()
    data_list = []
    data_list.append(pool.apply_async(talking, (queue,)))
    #data_list.append(pool.apply_async(listening, (queue,)))
    pool.close()
    pool.join()'''
	while True:   
		rospy.loginfo("aaaa")
    

def listening(queue):
    global listener
    listener = BaseListener.BaseListener(nodeName='RemoteControlListen', topic='joy', msgType=Joy, callBack=callback, callBackArgs=queue)
    return listener

def get_rawValue():
	content = ''
	tmp = (ord(ser.read()))
	# print tmp
	if 0xd == tmp:
		# print 'b: ' + tmp
		tmp = (ord(ser.read()))
		if 0xa == tmp:
			# print 'b ' + tmp
			content = (ser.read(24))
		tmp = (ord(ser.read()))
		if 0xa == tmp:
			# print 'e:' + tmp
			tmp = (ord(ser.read()))
			if 0xd == tmp:
				# print 'e ' + tmp
				# print 'content %s, len %d' % (str(content), len(content))
				rawValue = intToDouble(content)
				#return rawValue
				# print 'contetnt ％s, len %d: '％(str(content), len(content))
	if len(content) > 0:
		pass		
	
	from time import sleep
	# sleep(0.5)
	ser.flushInput()
	ser.flushOutput()
	return rawValue

if __name__ == '__main__':
    main()

