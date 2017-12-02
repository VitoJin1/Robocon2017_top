# -*- coding: UTF-8 -*-
#2017/2/25晋涛
#将定位模块的数据发布到ＣＨＡＳＳＩＳ上。

#2017/5/4改进串口用户接口
import BaseTalker
import BaseListener
import SerialCom
import os
import sys
import time
import rospy
import numpy as np
import serial
import math
from time import sleep
from multiprocessing import Pool, Manager

from communication.msg import *
from sensor_msgs.msg import *
from beginner_tutorials.msg import *

theta=0.0749


talker = None
listener = None
def sendData(data):
    ser.write(data)

def getInfo(l):
	if l[0] == '0xd' and l[1] == '0xa' and l[14] == '0xa' and l[15] == '0xd':
		return l[2:13]
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
	return struct.unpack('fff', int_6)
    
def getRawValue():
    global ser
    try:
        content = ''
        print 'start '
        tmp=(ord(ser.readSTM32Data()))
        print tmp
        #if 0xd == tmp:
        if 0xd == tmp:
            tmp = (ord(ser.readSTM32Data()))
            if 0xa == tmp:
                content = (ser.readSTM32Data(dataLen = 12))
                tmp = (ord(ser.readSTM32Data()))
                if 0xa == tmp:
                    tmp = (ord(ser.readSTM32Data()))
                    if 0xd == tmp:
                        
                        # print 'content %s, len %d' % (str(content), len(content))
                        rawValue = intToDouble(content)
                        #print rawValue
                        # print 'contetnt ％s, len %d: '％(str(content), len(content))
                        #if len(content) > 0:
                        # if len(content) > 0:
                            # pass		
                            #print "abcd"
                        #from time import sleep
                        # sleep(0.5)12
                        #ser.flushInput()
                        #ser.flushOutput()
                        # ser.flushInput()
                        # ser.flus0.0749hOutput()
                        ser.clearBuffer()
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
        print 'receive error'
	return None
    return None

def talking(queue):
    global theta
    global talker
    global ser
    lastRawValue = None
    print 'ready!'
    try:
        talker = BaseTalker.BaseTalker(nodeName='Chassis_Talk', topic='CHASSIS', msgType=chassis_msg)
    except Exception, e:
        print e
    print 'ready2'
    while True:
	    # ser.flushInput()
        # ser.flushOutput()
        ser.clearBuffer()
        mMsg = chassis_msg()
        print mMsg
        print 'ready3'
        if not queue.empty():
            while queue.qsize() > 1:
                print 'ready5'
                queue.get_nowait()
        mMsg.chassis = getRawValue()
        
        #15:53尝试加入安装矩阵
        #mMsg.chassis[0] =mMsg.chassis[0]*math.cos(theta)+mMsg.chassis[1]*math.sin(theta)
        #mMsg.chassis[1] =mMsg.chassis[0]*(-math.sin(theta))+mMsg.chassis[1]*math.cos(theta)
        #尝试结束
	if mMsg.chassis == None:
		sleep(0.10)
		continue
	print 'begin'
        #mMsg.chassis[0] =mMsg.chassis[0]*math.cos(theta)+mMsg.chassis[1]*math.sin(theta)
        #mMsg.chassis[1] =mMsg.chassis[0]*(-math.sin(theta))+mMsg.chassis[1]*math.cos(theta)
        print 'x=%f'%((mMsg.chassis[0]*math.cos(theta)+mMsg.chassis[1]*math.sin(theta)))
        print 'y=%f'%((mMsg.chassis[0]*(-math.sin(theta))+mMsg.chassis[1]*math.cos(theta)))
        print mMsg.chassis
        print 'end'
        talker.talk(content=mMsg)
    return talker



def main():
    manager = Manager()
    queue = manager.Queue()
    pool = Pool()
    data_list = []
    #print 'ready'
    data_list.append(pool.apply_async(talking, (queue,)))
    #data_list.append(pool.apply_async(listening, (queue,)))
    pool.close()
    pool.join()
    '''while True:
        getRawValue()
        print "abcd"
'''

import sys

if __name__ == '__main__':
    global ser
    if len(sys.argv) < 2:
        print 'please input the console port'
        sys.exit()
    consolePort = sys.argv[1]
    # os.system('sudo chmod 666 %s' % consolePort)
    
    if len(sys.argv) == 3:
        baudrate = int(sys.argv[2])
        ser = SerialCom.SerialCom(serialPort = consolePort, baudrate = baudrate)
    else:
        ser = SerialCom.SerialCom(serialPort = consolePort, baudrate = 115200)
    main()
