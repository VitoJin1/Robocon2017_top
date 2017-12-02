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
listener = None

def callback(data, queue):
     queue.put(data.str)
     rospy.loginfo(str(data.str))
     
     time.sleep(0.5)

def talking(queue):
    global talker
    talker = BaseTalker.BaseTalker(nodeName='RemoteControlTalk', topic='chatter', msgType=TestM)
    
    while True:
        mMsg = TestM()
        #if not queue.empty():
         #   while queue.qsize() > 1:
          #      queue.get_nowait()
	rospy.loginfo('asdkfjlaksdfasdkfaksd')
        mMsg.str = 'helloworld'
        talker.talk(content=mMsg)
    return talker

def listening(queue):
    global listener
    listener = BaseListener.BaseListener(nodeName='RemoteControlListen', topic='joy', msgType=TestM, callBack=callback, callBackArgs=queue)
    return listener

def main():
    manager = Manager()
    queue = manager.Queue()
    pool = Pool()
    data_list = []
    data_list.append(pool.apply_async(talking, (queue,)))
    #data_list.append(pool.apply_async(listening, (queue,)))
    pool.close()
    pool.join()

if __name__ == '__main__':
    main()
