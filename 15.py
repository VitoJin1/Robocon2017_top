# -*- coding: UTF-8 -*-

import BaseTalker
import BaseListener
import rospy
import os
import time
import numpy as np
import serial
from multiprocessing import Pool, Manager

# message type
from sensor_msgs.msg import *
from communication.msg import *

ser = serial.Serial('/dev/ttyUSB1',baudrate=115200, bytesize=8, parity=serial.PARITY_NONE, stopbits=1)
ser.flushInput()
ser.flushOutput()
talker = None
listener = None
a=0.0
b=0.0
c=0.0
d=0.0
def deCodeData(data):
    temp1=data.axes[4]
    temp2=data.axes[3]
    temp4=data.axes[0]
    checked_flag = 0
    pwmval = []
    #slow down the speed 3/5
    
    '''pwmval.append(300*(+temp1-temp2-temp4))
    pwmval.append(300*(+temp1+temp2+temp4))
    pwmval.append(300*(-temp1-temp2+temp4))
    pwmval.append(300*(+temp1-temp2+temp4))'''
    pwmval.append(300*(-temp2-temp4))
    pwmval.append(300*(+temp1+temp4))
    pwmval.append(300*(-temp1+temp4))
    pwmval.append(300*(-temp2+temp4))
    for i in range(4):
        if (pwmval[i])<20 and (pwmval[i]>=-20):
            pwmval[i]=0.0
        
    for pwm_raw in [pwmval[0],pwmval[1],pwmval[2],pwmval[3]]:
        if (pwm_raw>=5500.0)and (checked_flag==0):
            checked_flag=1
            for i in range(4):
                pwmval[i]=(pwmval[i]/pwm_raw)*5000.0
        # rospy.loginfo("break highest line\n")
        elif (pwm_raw<=-5500.0)and(checked_flag==0):
            checked_flag=1            
            for i in range(4):
                pwmval[i]=(pwmval[i]/pwm_raw)*(-5000.0)
    rospy.loginfo("%f %f %f %f"%(pwmval[0],pwmval[1],pwmval[2],pwmval[3]))
    #rospy.loginfo("control_temp %f %f %f "%(temp1,temp2,temp4))
    ser.write("DA%f/%f/%f/%f/\x0d\x0a" %(pwmval[0],pwmval[1],pwmval[2],pwmval[3]))
    ser.flushInput()
    ser.flushOutput()
    return pwmval

def callback(data, queue):
    queue.put(deCodeData(data))
    #rospy.loginfo("%f" % pwmval[2])
    # print deCodeData(data)


def talking(queue):
    global talker
    talker = BaseTalker.BaseTalker(nodeName='RemoteControlTalk', topic='REMOTER', msgType=pwmset)
    
    while True:
        mMsg = pwmset()
        if not queue.empty():
            while queue.qsize() > 1:
                queue.get_nowait()
            
            mMsg.pwm = queue.get(True)
            # rospy.loginfo("healkdhfahdsfahsdfjkh")
            talker.talk(content=mMsg)
    return talker

def listening(queue):
    global listener
    listener = BaseListener.BaseListener(nodeName='RemoteControlListen', topic='joy', msgType=Joy, callBack=callback, callBackArgs=queue)
    return listener

def main():
    manager = Manager()
    queue = manager.Queue()
    pool = Pool()
    data_list = []
    data_list.append(pool.apply_async(talking, (queue,)))
    data_list.append(pool.apply_async(listening, (queue,)))
    pool.close()
    pool.join()

if __name__ == '__main__':
    main()
