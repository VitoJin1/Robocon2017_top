# -*- coding: UTF-8 -*-

import BaseTalker
import BaseListener
import rospy
import os
import time
import numpy as np
from multiprocessing import Pool, Manager

# message type
from sensor_msgs.msg import *

talker = None
listener = None

def callback(data, queue):
     #queue.put(data.axes)
     rospy.loginfo(str(data.axes))
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
    listener = BaseListener.BaseListener(nodeName='RemoteControlListen', topic='REMOTER', msgType=Joy, callBack=callback, callBackArgs=queue)
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
