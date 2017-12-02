# -*- coding: UTF-8 -*-
#2017/2/25晋涛
#将定位模块的数据发布到ＣＨＡＳＳＩＳ上。
import BaseTalker
import BaseListener
import os
import sys
import time
import rospy
import numpy as np
import serial
from time import sleep
from multiprocessing import Pool, Manager

from communication.msg import *
from sensor_msgs.msg import *
from beginner_tutorials.msg import *

ser = serial.Serial('/dev/ttyUSB1',baudrate=9600, bytesize=8, parity=serial.PARITY_NONE, stopbits=1)
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
def getRawValue():
    try:
        content = ''
        tmp=(ord(ser.read()))
        # print tmp
        #if 0xd == tmp:
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
                        #print rawValue
                        # print 'contetnt ％s, len %d: '％(str(content), len(content))
                        #if len(content) > 0:
                        # if len(content) > 0:
                            # pass		
                            #print "abcd"
                        #from time import sleep
                        # sleep(0.5)
                        #ser.flushInput()
                        #ser.flushOutput()
                        ser.flushInput()
                        ser.flushOutput()
                        return rawValue
                    else:
                        print 'end# 0xd not receive, receive ' + str(tmp)
			return None
                else:
                    print 'end# 0xa not receive, receive ' + str(tmp)
		    return None
            else:
                print 'head# 0xa not receive, receive ' + str(tmp)
		return None
    except Exception, e:
        print e
	return None
    return None

def talking(queue):
    global talker
    lastRawValue = None
    try:
        talker = BaseTalker.BaseTalker(nodeName='Chassis_Talk', topic='CHASSIS', msgType=chassis_msg)
    except Exception, e:
        print e
    while True:
	ser.flushInput()
        ser.flushOutput()
        mMsg = chassis_msg()
        if not queue.empty():
            while queue.qsize() > 1:
                queue.get_nowait()
        mMsg.chassis = getRawValue()
	if mMsg.chassis == None:
		sleep(0.10)
		continue
	print 'begin'
        print mMsg.chassis
        print 'end'
        talker.talk(content=mMsg)
    return talker

'''
while True:	
	
    getRawValue()
    print "abcd"
   
    
'''
def main():
    manager = Manager()
    queue = manager.Queue()
    pool = Pool()
    data_list = []
    data_list.append(pool.apply_async(talking, (queue,)))
    #data_list.append(pool.apply_async(listening, (queue,)))
    pool.close()
    pool.join()
    '''while True:
        getRawValue()
        print "abcd"
'''
if __name__ == '__main__':
    main()
