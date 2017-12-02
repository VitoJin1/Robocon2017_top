# -*- coding: UTF-8 -*-
#画地图的栅格，格子大小比较大，方便坐标转换，方格大小为1.5*1.5m
#laser_test[0]-
import BaseTalker
import BaseListener
import rospy
import os
import time
import math
import numpy as np
from multiprocessing import Pool, Manager

# message type
from sensor_msgs.msg import *
from beginner_tutorials.msg import *

talker = None
listener = None
def decode(data):
    #on_car为车身坐标系，on_car_y为纵轴，on_car_x为横轴
    
    for i in range(1080):
        on_car_y=data.ranges[i]*math.cos(data.angle_min+((data.angle_increment)*i))
        on_car_x=data.ranges[i]*math.sin(data.angle_min+((data.angle_increment)*i))
        if on_car_y>6  or on_car_y<4 or on_car_x >5 or on_car_x<-5:
            pass
        else:
            print"I find the pile  %d"%i
    return data
        
    
    

def callback(data, queue):
     queue.put(decode(data))
     

def talking(queue):
    global talker
    talker = BaseTalker.BaseTalker(nodeName='laser_test_talk', topic='laser_test_topic', msgType=LaserScan)
    
    while True:
        mMsg = LaserScan()
        #if not queue.empty():
            #while queue.qsize() > 1:
                #queue.get_nowait()
        #mMsg.str = 'hello world'
        talker.talk(content=mMsg)
    return talker

def listening(queue):
    global listener
    listener = BaseListener.BaseListener(nodeName='laser_test_listen', topic='first', msgType=LaserScan, callBack=callback, callBackArgs=queue)
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
