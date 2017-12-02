# -*- coding: UTF-8 -*-

import rospy
import os
import time
import numpy as np
import serial
import PID
import BaseTalker
import BaseListener
import SerialCom
from multiprocessing import Pool, Manager

# message type
from sensor_msgs.msg import *
from beginner_tutorials.msg import *
from communication.msg import *

# ser = None
talker = None
talkerTest = None
listener = None
listenerTest = None
pid=(PID.PID(),PID.PID(),PID.PID())

#pid[0]--x方向的pid运算，pid[1]--y方向的pid运算，pid[2]--z方向的pid运算
pid[0].PID_init()
pid[1].PID_init()
pid[2].PID_init()
position=[0,0,0,0,0,0]
speed   =[0,0,0,0,0,0,0]

#定位模块
def deCodeData1(data, queue):
    global pid
    pwmval=[0,0,0,0]
    ser_data=[0,0]#最后两位控制信号，第一位解析为：0-无；1-发射任务；2-转速上升任务；3-转速下降任务
    #第二位解析为：0-无；1-位置1;2-位置2;3-位置3;4-位置4;5-位置5;6-位置6;7-位置7;8-位置8
    place=[[1000,0],[1000,0],[1000,0],[1000,0],[1000,0],[1000,0],[1000,0],[1000,0]]
    global position
    global speed
    tempx=0.0
    tempy=0.0
    tempz=0.0
    if queue.empty():
        pass
    else:
        while queue.qsize() > 1:
            queue.get_nowait()
        tmp=queue.get()
        position[0]=tmp[0]
        position[1]=tmp[1]
        position[2]=tmp[2]
        position[3]=tmp[3]
        position[4]=tmp[4]
        position[5]=tmp[5]
        speed[0]   =tmp[6]
        speed[1]   =tmp[7]
        speed[2]   =tmp[8]
        speed[3]   =tmp[9]
        speed[4]   =tmp[10]
        speed[5]   =tmp[11]
        speed[6]   =tmp[12]
    #print 'tmp=%f %f %f %f %f %f %f %f %f %f %f %f'%(position[0],position[1],position[2],position[3],position[4],position[5],speed[0],speed[1],speed[2],speed[3],speed[4],speed[5])

    #pid[0]._fdb=-data.chassis[3]
    #pid[1]._fdb=-data.chassis[4]
    #pid[2]._fdb=-data.chassis[0]
    #pid[0]._ref=speed[0]
    #pid[1]._ref=speed[1]
    #pid[2]._ref=speed[2]
    stop_flag  =speed[2]
    shoot_flag =speed[6]
    #if stop_flag==0:
     #   pid[0]._ref=0
      #  pid[1]._ref=0
       # pid[2]._ref=0
    if position[0]==1 or position[1]==1 or position[2]==1 or position[3]==1 or position[4]==1 or position[5]==1 :
        for i in range(6):
            if position[i]==1:
                pid[0]._ref=place[i][0]
                pid[1]._ref=place[i][1]
                pid[2]._ref=0
                pid[0]._fdb=-data.chassis[3]
                pid[1]._fdb=-data.chassis[4]
                pid[2]._fdb=-data.chassis[0]
                tempx=PID.PID.PID_Calc(pid[0])
                tempy=PID.PID.PID_Calc(pid[1])
                tempz=PID.PID.PID_Calc(pid[2])
                pwmval[0]=+tempx-tempy-tempz
                pwmval[1]=+tempx+tempy+tempz
                pwmval[2]=-tempx-tempy+tempz
                pwmval[3]=+tempx-tempy+tempz
                print 'ref:%f feb:%f'%(pid[0]._ref,pid[0]._fdb)
                print '%f PID_result %f %f %f'%(i,tempx,tempy,tempz)
        #手动控制
    elif abs(speed[0])>0.06 or abs(speed[1])>0.06 or abs(speed[3])>0.06 :
        tempx=speed[0]
        tempy=speed[1]
        tempz=speed[3]
        pwmval[0]=300*(+tempx-tempy-tempz)
        pwmval[1]=300*(+tempx+tempy+tempz)
        pwmval[2]=300*(-tempx-tempy+tempz)
        pwmval[3]=300*(+tempx-tempy+tempz)
        for i in range(4):
            if (pwmval[i])<20 and (pwmval[i]>=-20):
                pwmval[i]=0.0
    #tempx=speed[0]
    #tempy=speed[1]
    #tempz=speed[3]
    #pwmval[0]=300*(+tempx-tempy-tempz)
    #pwmval[1]=300*(+tempx+tempy+tempz)
    #pwmval[2]=300*(-tempx-tempy+tempz)
    #pwmval[3]=300*(+tempx-tempy+tempz)
    for i in range(4):
        if (pwmval[i])<20 and (pwmval[i]>=-20):
            pwmval[i]=0.0
    if stop_flag==1:
        for i in range(4):
            pwmval[i]=0
    if shoot_flag==1:
        ser_data[0]=1
        print 'shooting!'
    elif speed[4]==1:
        ser_data[0]=2
        print 'shooting speed up!'
    elif speed[5]==1:
        ser_data[0]=3
        print 'shooting speed down!'
    else:
        ser_data[0]=0

    if position[0]==1:
        ser_data[1]=1
    elif position[0]==-1:
        ser_data[1]=2
    elif position[1]==1:
        ser_data[1]=3
    elif position[1]==-1:
        ser_data[1]=4
    elif position[2]==1:
        ser_data[1]=5
    elif position[3]==1:
        ser_data[1]=6
    elif position[4]==1:
        ser_data[1]=7
    elif position[5]==1:
        ser_data[1]=8
    else:
        ser_data[1]=0
    
    #print 'for:%f lef:%f turn: %f stop=%f'%(tempx,tempy,tempz,stop_flag)
    print 'pwmval1=%f pwmval2=%f pwmval3=%f pwmval4=%f speed_up=%d speed_down=%d'%(pwmval[0],pwmval[1],pwmval[2],pwmval[3],ser_data[0],ser_data[1])
    #ser.write("DA%f/%f/%f/%f/\x0d\x0a" %(pwmval[0],pwmval[1],pwmval[2],pwmval[3]))
    # ser.write("DA%f/%f/%f/%f/%d%d\x0d\x0a" %(pwmval[0],pwmval[1],pwmval[2],pwmval[3],ser_data[0],ser_data[1]))
    # ser.flushInput()
    # ser.flushOutput()
    global ser
    sendData = 'DA%f/%f/%f/%f/%d%d\x0d\x0a' %(pwmval[0],pwmval[1],pwmval[2],pwmval[3],ser_data[0],ser_data[1])
    ser.sendToSTM32(sendData)
    #ser.write("DA%f/%f/%f/%f/%d%d\x0d\x0a" %(pwmval[0],pwmval[1],pwmval[2],pwmval[3],ser_data[0],ser_data[1]))
    return pwmval      
    
    

#遥控器数据，同步到全局变量
def deCodeData2(data):
    position=[0,0,0,0,0,0]
    speed   =[0,0,0,0,0,0,0]
    position[0]=data.remote_data[0]
    position[1]=data.remote_data[1]
    position[2]=data.remote_data[2]
    position[3]=data.remote_data[3]
    position[4]=data.remote_data[4]
    position[5]=data.remote_data[5]
    speed[0]   =data.remote_data[6]
    speed[1]   =data.remote_data[7]
    speed[2]   =data.remote_data[8]
    speed[3]   =data.remote_data[9]
    speed[4]   =data.remote_data[10]
    speed[5]   =data.remote_data[11]
    speed[6]   =data.remote_data[12]
    #print 'decode %f %f %f %f %f %f %f %f %f %f %f %f'%(position[0],position[1],position[2],position[3],position[4],position[5],speed[0],speed[1],speed[2],speed[3],speed[4],speed[5])
    return (position[0],position[1],position[2],position[3],position[4],position[5],speed[0],speed[1],speed[2],speed[3],speed[4],speed[5],speed[6])
    
#对定位模块的解析，fedback
def callback1(data, queue):
    
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
    # managerTest = Manager()
    # queueTest = managerTest.Queue()
    pool = Pool()
    data_list = []
    data_list.append(pool.apply_async(listeningTest, (queue,)))
    data_list.append(pool.apply_async(listening, (queue,)))
    pool.close()
    pool.join()


import sys

if __name__ == '__main__':
    global ser
    if len(sys.argv) < 2:
        print 'please input the console port'
        sys.exit()
    consolePort = sys.argv[1]
    # os.system('sudo chmod 666 %s' % consolePort)
    
    if len(sys.argv) == 3:
        baudrate = int(sys.argv[2])
        ser = SerialCom.SerialCom(serialPort = consolePort, baudrate = baudrate)
        main()
    else:
        ser = SerialCom.SerialCom(serialPort = consolePort, baudrate = 115200)
    
    main()
