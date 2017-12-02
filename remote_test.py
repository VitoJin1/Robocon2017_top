# -*- coding: UTF-8 -*-
#2017/2/27 晋涛
#半自动遥控信号结合定位模块反馈数据做闭环
#position control 
#attitude control
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
'''
ser = serial.Serial('/dev/ttyUSB0',baudrate=115200, bytesize=8, parity=serial.PARITY_NONE, stopbits=1)
ser.flushInput()
ser.flushOutput()'''
talker = None
listener = None
a=0.0
b=0.0
c=0.0
d=0.0
forward=0
back=0
left=0
right=0
forward_now=0
back_now=0
left_now=0
right_now=0

def deCodeData(data):
    global forward
    global back
    global left
    global right
    global forward_now
    global back_now
    global left_now
    global right_now
    
    position =[]
    forward_last=forward_now
    back_last   =back_now
    left_last   =left_now
    right_last  =right_now
    #print 'back_last%f'%back_last
    forward_now=data.buttons[3]
    back_now   =data.buttons[0]
    left_now   =data.buttons[2]
    right_now  =data.buttons[1]
    stop_flag  =data.buttons[6]
    #print 'back_now%f'%back_now
    if forward_now==0 and forward_last==1 :
        forward=forward+1
    if back_now ==0  and back_last ==1 :
        back=back+1
    if left_now ==0  and left_last ==1 :
        left=left+1
    if right_now==0  and right_last==1 :
        right=right+1
    
    #print 'back%f'%back
    position.append(forward-back)
    position.append(right  -left)
    position.append(stop_flag)
    print 'forward_back%f'%(position[0])
    print 'left_right  %f'%(position[1])
    print 'stop_flag  %f'%(position[2])
    return position
def callback(data, queue):
    queue.put(deCodeData(data))
    #rospy.loginfo("%f" % pwmval[2])
    # print deCodeData(data)


def talking(queue):
    global talker
    talker = BaseTalker.BaseTalker(nodeName='RemoteControl_Talk', topic='REMOTER', msgType=remoter_msg)
    
    while True:
        mMsg = remoter_msg()
        if not queue.empty():
            while queue.qsize() > 1:
                queue.get_nowait()
            
            mMsg.position = queue.get(True)
            #print 'begin'
            #print deCodeData(data)
            #print 'end'
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
