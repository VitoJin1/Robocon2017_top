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
from communication.msg import *


talker = None
listener = None

def deCodeData(data):
    temp1=data.axes[4]
    temp2=data.axes[3]
    temp4=data.axes[0]
    checked_flag = 0
    pwmval = []
    pwmval.append(5000*(-temp1-temp2+temp4))
    pwmval.append(5000*(+temp1+temp2+temp4))
    pwmval.append(5000*(+temp1-temp2+temp4))
    pwmval.append(5000*(-temp1+temp2+temp4))

    # do for minimal value
    for i in range(4):
        if (abs(pwmval[i]))<350:
            pwmval[i]=0                      
    
    # do for maxmal value
    absPwmval = map(lambda x : abs(x), pwmval)
    if max(absPwmval) >= 5500.:
        maxPwm = max(absPwmval)
        pwmval = map(lambda x : (x * 5000.) / maxPwm, pwmval)

    # print pwmval

    # for pwm_raw in [pwmval[0],pwmval[1],pwmval[2],pwmval[3]]:
    #     if (pwm_raw>=5500.0)and (checked_flag==0):
    #         checked_flag=1
    #         for i in range(4):
    #             pwmval[i]=(pwmval[i]/pwm_raw)*5000.0
    #     # rospy.loginfo("break highest line\n")
    #     elif (pwm_raw<=-5500.0)and(checked_flag==0):
    #         checked_flag=1            
    #         for i in range(4):
    #             pwmval[i]=(pwmval[i]/pwm_raw)*5000.0
    # # rospy.loginfo("break lowest line\n")
    return pwmval

def callback(data, queue):
    queue.put(deCodeData(data))
   
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
