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
from beginner_tutorials.msg import *
from communication.msg import *
ser = serial.Serial('/dev/ttyUSB1',baudrate=115200, bytesize=8, parity=serial.PARITY_NONE, stopbits=1)
ser.flushInput()
ser.flushOutput()
talker = None
talkerTest = None
listener = None
listenerTest = None
kp_x=5.0
kp_y=5.0
kp_z=10.0
ki_x=0.0
ki_y=0.0
ki_z=0.0
kd_x=0.0
kd_y=0.0
kd_z=0.0
chassis_ref_x=0.0
chassis_ref_y=0.0
chassis_ref_z=0.0
chassis_fed_x_now=0.0  
chassis_fed_y_now=0.0  
chassis_fed_z_now=0.0  
stop_flag=0
#定位模块
def deCodeData1(data, queue):
    pwmval = []
    global chassis_fed_x_now
    global chassis_fed_y_now
    global chassis_fed_z_now
    global chassis_ref_x
    global chassis_ref_y
    global chassis_ref_z
    global stop_flag
    global kp_x
    global kp_y
    global kp_z
    global ki_x
    global ki_y
    global ki_z
    global kd_x
    global kd_y
    global kd_z
    chassis_fed_x_last=chassis_fed_x_now
    chassis_fed_y_last=chassis_fed_y_now
    chassis_fed_z_last=chassis_fed_z_now
    chassis_fed_x_now=-data.chassis[3]
    chassis_fed_y_now=-data.chassis[4]
    chassis_fed_z_now=-data.chassis[0]
    if queue.empty():
        pass
        # chassis_ref_x = 0
        # chassis_ref_y = 0
        # chassis_ref_z = 0
        # stop_flag     = 1
    else:
        while queue.qsize() > 1:
            queue.get_nowait()
        tmp = queue.get()
        chassis_ref_x = tmp[0]
        chassis_ref_y = tmp[1]
        chassis_ref_z = tmp[2]
        stop_flag     = tmp[3]
        print ("ref1:%f %f"%(chassis_ref_x,chassis_ref_y))
    
    if stop_flag==0:
        temp2=kp_x*(chassis_ref_x-chassis_fed_x_now)
        temp1=kp_y*(chassis_ref_y-chassis_fed_y_now)
        temp4=kp_z*(chassis_ref_z-chassis_fed_z_now)
    elif stop_flag==1:
        temp1=0
        temp2=0
        temp4=0

    pwmval.append(0.05*(temp2+temp4))
    pwmval.append(0.05*(temp1-temp4))
    pwmval.append(0.05*(-temp1-temp4))
    pwmval.append(0.05*(temp2-temp4))

    print ("%f %f %f %f"%(pwmval[0],pwmval[1],pwmval[2],pwmval[3]))

    ser.write("DA%f/%f/%f/%f/\x0d\x0a" %(pwmval[0],pwmval[1],pwmval[2],pwmval[3]))
    ser.flushInput()
    ser.flushOutput()
    return pwmval        
    #pid_calculatate
'''    
    P_X=0.0
    P_Y=0.0
    P_Z=0.0
    I_X=0.0
    I_Y=0.0
    I_Z=0.0
    D_X=0.0
    D_Y=0.0
    D_Z=0.0
    error_x=0.0
    error_y=0.0
    error_z=0.0
    error_last_x=0.0
    error_last_y=0.0
    error_last_z=0.0    error_sum_x=0.0
    error_sum_y=0.0
    error_sum_z=0.0
    error_x     = chassis_ref_x - chassis_fed_x_now
    error_y     = chassis_ref_y - chassis_fed_y_now
    error_z     = chassis_ref_z - chassis_fed_z_now
    error_last_x= chassis_ref_x - chassis_fed_x_last
    error_last_y= chassis_ref_y - chassis_fed_y_last
    error_last_z= chassis_ref_z - chassis_fed_z_last
    error_sum_x = error_sum_x   + error_x
    error_sum_x = error_sum_y   + error_y
    error_sum_x = error_sum_z   + error_z
    P_X         = kp_x          * error_x
    P_Y         = kp_y          * error_y
    P_Z         = kp_z          * error_z
    I_X         = ki_x          * error_sum_x
    I_Y         = ki_y          * error_sum_y
    I_Z         = ki_z          * error_sum_z
    D_X         = kd_x          * (error_x-error_last_x)
    D_Y         = kd_y          * (error_y-error_last_y)
    D_Z         = kd_z          * (error_z-error_last_z)
    if stop_flag==0:
        temp2       = P_X+I_X+D_X
        temp1       = P_Y+I_Y+D_Y
        temp4       = P_Z+I_Z+D_Z
    elif stop_flag==1:
        temp1       = 0
        temp2       = 0
        temp4       = 0
'''    
    #pid_calculatate
    #print ("ref1:%f %f"%(chassis_ref_x,chassis_ref_y))
    #print ("fed:%f %f"%(chassis_fed_x_now,chassis_fed_y_now))
    

#遥控器数据，同步到全局变量
def deCodeData2(data):
    global chassis_ref_x
    global chassis_ref_y
    global chassis_ref_z
    global stop_flag
    #半自动位ｍｍ
    chassis_ref_x=data.position[1]*1000
    chassis_ref_y=data.position[0]*1000
    chassis_ref_z=0
    stop_flag    =data.position[2]
    #print ("ref:%f %f"%(chassis_ref_x,chassis_ref_y))
    return (chassis_ref_x, chassis_ref_y, chassis_ref_z, stop_flag)
    #print '%f%f'%(chassis_ref_x,chassis_ref_y)
#对定位模块的解析，fedback
def callback1(data, queue):
    #  queue.put(deCodeData1(data))
    deCodeData1(data, queue)
    return None
     
#对遥控器信号的解析，reference,目标为上传数据到全局变量，送到反馈数据端去处理
def callback2(data, queue):
     queue.put(deCodeData2(data))


def listening(queue):
    global listener
    listener = BaseListener.BaseListener(nodeName='Chassis_Listen', topic='CHASSIS', msgType=chassis_msg, callBack=callback1, callBackArgs=queue)
    return listener

def listeningTest(queue):
    global listenerTest
    listenerTest = BaseListener.BaseListener(nodeName='RemoteControl_Talk', topic='REMOTER', msgType=remoter_msg, callBack=callback2, callBackArgs=queue)
    return listenerTest


def main():
    manager = Manager()
    queue = manager.Queue()

    managerTest = Manager()
    queueTest = managerTest.Queue()

    pool = Pool()
    data_list = []
    
    #data_list.append(pool.apply_async(talkingTest, (queue,)))
    data_list.append(pool.apply_async(listeningTest, (queue,)))
    #data_list.append(pool.apply_async(talking, (queueTest,)))
    data_list.append(pool.apply_async(listening, (queue,)))
    
    pool.close()
    pool.join()

if __name__ == '__main__':
    main()
