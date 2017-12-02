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
from beginner_tutorials.msg import *

talker = None
talkerTest = None
listener = None
listenerTest = None

def callback(data, queue):
     queue.put(data.str)
     rospy.loginfo(str(data.str))
     time.sleep(0.5)

def talking(queue):
    global talker
    talker = BaseTalker.BaseTalker(nodeName='RemoteControlTalk', topic='REMOTER', msgType=remote_control)
    
    while True:
        mMsg = remote_control()
        #if not queue.empty():
            #while queue.qsize() > 1:
                #queue.get_nowait()
        mMsg.str = 'hello world'
        talker.talk(content=mMsg)
    return talker

def talkingTest(queue):
    global talkerTest
    talkerTest = BaseTalker.BaseTalker(nodeName='RemoteControlTalk', topic='REMOTER', msgType=remote_control)
    
    while True:
        mMsg = remote_control()
        #if not queue.empty():
            #while queue.qsize() > 1:
                #queue.get_nowait()
        mMsg.str = 'hello world'
        talker.talk(content=mMsg)
    return talkerTest

def listening(queue):
    global listener
    listener = BaseListener.BaseListener(nodeName='RemoteControlListen', topic='joy', msgType=Joy, callBack=callback, callBackArgs=queue)
    return listener

def listeningTest(queue):
    global listenerTest
    listenerTest = BaseListener.BaseListener(nodeName='RemoteControlListen', topic='joy', msgType=Joy, callBack=callback, callBackArgs=queue)
    return listenerTest


def main():
    manager = Manager()
    queue = manager.Queue()

    managerTest = Manager()
    queueTest = managerTest.Queue()

    pool = Pool()
    data_list = []
    
    data_list.append(pool.apply_async(talkingTest, (queue,)))
    data_list.append(pool.apply_async(listeningTest, (queue,)))
    data_list.append(pool.apply_async(talking, (queueTest,)))
    data_list.append(pool.apply_async(listening, (queueTest,)))
    
    pool.close()
    pool.join()

if __name__ == '__main__':
    main()
