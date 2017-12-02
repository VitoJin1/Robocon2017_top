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
from time import sleep
from multiprocessing import Pool, Manager

from communication.msg import *
from sensor_msgs.msg import *
from beginner_tutorials.msg import *


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
    global ser
    try:
        content = ''
        tmp=(ord(ser.readSTM32Data()))
        #print tmp
        #if 0xd == tmp:
        if 0xd == tmp:
            # print 'b: ' + tmp
            tmp = (ord(ser.readSTM32Data()))
            if 0xa == tmp:
                # print 'b ' + tmp
                content = (ser.readSTM32Data(dataLen = 24))
                tmp = (ord(ser.readSTM32Data()))
                if 0xa == tmp:
                    # print 'e:' + tmp
                    tmp = (ord(ser.readSTM32Data()))
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
                        # ser.flushInput()
                        # ser.flushOutput()
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
	return None
    return None

def talking(queue):
    global talker
    global ser
    lastRawValue = None
    try:
        talker = BaseTalker.BaseTalker(nodeName='Chassis_Talk', topic='CHASSIS', msgType=chassis_msg)
    except Exception, e:
        print e
    while True:
	    # ser.flushInput()
        # ser.flushOutput()
        ser.clearBuffer()
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

import sys

if __name__ == '__main__':
    global ser
    if len(sys.argv) < 2:
        print 'please input the console port'
        sys.exit()
    consolePort = sys.argv[1]
    os.system('sudo chmod 666 %s' % consolePort)
    
    if len(sys.argv) == 3:
        baudrate = int(sys.argv[2])
        ser = SerialCom.SerialCom(serialPort = consolePort, baudrate = baudrate)
    else:
        ser = SerialCom.SerialCom(serialPort = consolePort, baudrate = 115200)
    main()
