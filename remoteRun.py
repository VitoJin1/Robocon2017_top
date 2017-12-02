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

talker = None
listener = None

def deCodeData(data):
    position=[]
    position.append(data.axes[6])       #上下键
    position.append(data.axes[7])       #左右键
    temp1         =(data.axes[4])       #前进的权值
    temp2         =(data.axes[3])       #左右移动的权值
    temp4         =(data.axes[0])       #旋转的权值
    position.append(data.buttons[2])    #X键
    position.append(data.buttons[0])    #A键
    position.append(data.buttons[1])    #B键
    position.append(data.buttons[3])    #Y键
    stop_flag     =(data.buttons[7])    #停止运动标志位
    shoot_flag    =(data.buttons[6])    #射击标志位
    speed_up      =(data.buttons[4])    #摩擦轮提速标志位
    speed_down    =(data.buttons[5])    #摩擦轮降速标志位
    
    #print '遥杆信号%f%f%f%f'%(temp1,temp2,stop_flag,temp4)
    #遥控信息整合
    remote_data=[0,0,0,0,0,0,0,0,0,0,0,0,0]
    remote_data[0]=position[0]
    remote_data[1]=(position[1])
    remote_data[2]=(position[2])
    remote_data[3]=(position[3])
    remote_data[4]=(position[4])
    remote_data[5]=(position[5])
    remote_data[6]=temp1
    remote_data[7]=temp2
    remote_data[8]=stop_flag
    remote_data[9]=temp4
    remote_data[10]=speed_up
    remote_data[11]=speed_down
    remote_data[12]=shoot_flag
    #print '数据接受1 position:%f %f %f %f %f %f speed:%f %f %f %f %f %f'%(position[0],position[1],position[2],position[3],position[4],position[5],temp1,temp2,stop_flag,temp4,speed_up,speed_down)
    print '数据接受2 position:%f %f %f %f %f %f speed:%f %f %f %f %f %f %f'%(remote_data[0],remote_data[1],remote_data[2],remote_data[3],remote_data[4],remote_data[5],remote_data[6],remote_data[7],remote_data[8],remote_data[9],remote_data[10],remote_data[11],remote_data[12])
    return remote_data

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
        
            mMsg.remote_data = queue.get(True)
            #remote_data为msg文件的一种类型，其中第0-6位为position，第7-10位为电机转速控制信息，第11,12位为摩擦轮转速变化信息    
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
