# -*- coding: UTF-8 -*-

import BaseTalker
import BaseListener
import rospy
import os
import time
import numpy as np
from multiprocessing import Pool, Manager

# message type
from communication.msg import *

talker = None
listener = None

'''  pid_control    '''
'''
float kp=0.0
float ki=0.0
float kd=0.0

def pid_control(ref,fed):
'''
def callback(data, queue):
     #queue.put(data.axes)
     for i in range(4):
        rospy.loginfo(str(data.pwm[i]))
     time.sleep(0.5)

def talking(queue):
    global talker
    talker = BaseTalker.BaseTalker(nodeName='RemoteControlTalk', topic='REMOTER', msgType=Joy)
    
    while True:
        mMsg = Joy()
        if not queue.empty():
            while queue.qsize() > 1:
                queue.get_nowait()
            mMsg.axes = queue.get(True)
            talker.talk(content=mMsg)
    return talker

def listening(queue):
    global listener
    listener = BaseListener.BaseListener(nodeName='ChassisPid', topic='REMOTER', msgType=pwmset, callBack=callback, callBackArgs=queue)
    return listener

def main():
    manager = Manager()
    queue = manager.Queue()
    pool = Pool()
    data_list = []
    #data_list.append(pool.apply_async(talking, (queue,)))
    data_list.append(pool.apply_async(listening, (queue,)))
    pool.close()
    pool.join()

if __name__ == '__main__':
    main()
