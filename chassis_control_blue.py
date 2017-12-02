# -*- coding: UTF-8 -*-
import math
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
theta=0.0749
angle_tan_max=0.5

#pid[0]--x方向的pid运算，pid[1]--y方向的pid运算，pid[2]--z方向的pid运算
pid[0].PID_init()
pid[1].PID_init()
pid[2].PID_init()
position=[0,0,0,0,0,0]
speed   =[0,0,0,0,0,0,0]
extra   =[0,0,0,0,0]
cali_xyz=[0,0,0]
cali_flag=0
mid_position   =[6560,  355]
reload_position=[13300,-299]
place=[mid_position,reload_position,[0,0],[0,0],[0,0],[0,0],[0,0]]
def matrixCal (localX, localY, theta, caliX, caliY):
    import numpy as np
    import math
    alpha = np.array([localX, localY])
    beta  = np.array([[math.cos(theta), -math.sin(theta)], [math.sin(theta), math.cos(theta)]])
    lemma = np.array([caliX, caliY])
    ans = np.dot(alpha, beta) + lemma
    print ans
    return (ans[0], ans[1])

def initSerialPort(consolePort):
    ser = serial.Serial(consolePort,baudrate=115200, bytesize=8, parity=serial.PARITY_NONE, stopbits=1)
    ser.flushInput()
    ser.flushOutput()
    return ser

#定位模块
def deCodeData1(data, queue):
    global angle_tan_max
    global pid
    global cali_xyz
    global cali_flag
    output=0
    pwmval=[0,0,0,0]
    ser_data=[0,0]#最后两位控制信号，第一位解析为：0-无；1-发射任务；2-转速上升任务；3-转速下降任务
    #第二位解析为：0-无；1-位置1;2-位置2;3-位置3;4-位置4;5-位置5;6-位置6;7-位置7;8-位置8
    global place
    #（6574,345）
    #place 的第一个是十字的左键，第二个是十字的下键,第三个为下键，第四个为上建
    global position
    global speed
    global extra
    global theta
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
        position[2]=tmp[2]#丝杠停止
        position[3]=tmp[3]#丝杠下降
        position[4]=tmp[4]
        position[5]=tmp[5]#丝杠上升
        speed[0]   =tmp[6]
        speed[1]   =tmp[7]
        speed[2]   =tmp[8]
        speed[3]   =tmp[9]
        speed[4]   =tmp[10]
        speed[5]   =tmp[11]
        speed[6]   =tmp[12]
        extra[0]   =tmp[13]
        extra[1]   =tmp[14]
        extra[2]   =tmp[15]
        extra[3]   =tmp[16]
        extra[4]   =tmp[17]
        
    #print 'tmp=%f %f %f %f %f %f %f %f %f %f %f %f'%(position[0],position[1],position[2],position[3],position[4],position[5],speed[0],speed[1],speed[2],speed[3],speed[4],speed[5])

    #pid[0]._fdb=-data.chassis[3]9430
    #pid[1]._fdb=-data.chassis[4]
    #pid[2]._fdb=-data.chassis[0]
    #pid[0]._ref=speed[0]
    #pid[1]._ref=speed[1]
    #pid[2]._ref=speed[2]
    pitch_up   =extra[0]   #mid_button
    pitch_down =speed[2] #右侧start键
    shoot_flag =speed[6]
    left_axes  =extra[3]
    right_axes =extra[4]
    #if stop_flag==0:
     #   pid[0]._ref=0
      #  pid[1]._ref=0
       # pid[2]._ref=0
    if position[4]==1:
        place[0]=[0,0]
        place[1]=[6726,-635]
    
    if left_axes==-1 or right_axes==-1 or pitch_down==1 or pitch_up==1 or position[0]==1 or position[0]==-1 or position[1]==1 or position[1]==-1:
        if left_axes==-1:#中间位置
            pid[0]._ref=place[0][0]
            pid[1]._ref=place[0][1]
            pid[2]._ref=0+1
        elif right_axes==-1:#装弹位置
            pid[0]._ref=place[1][0]
            pid[1]._ref=place[1][1]
            pid[2]._ref=0 
        elif pitch_down==1:#近台
            print 'position 6!!!!!!'
            ser_data[0]=5
            pid[2]._ref=-9.7-1
            #print '*******************ser_data[0]=%d'%(ser_data[0])
        elif pitch_up==1:#远台
            print 'position 5!!!!'
            ser_data[0]=4
            pid[2]._ref=-15
            print 'angle*****%f'%(pid[2]._ref)
        elif position[0]==1:#中1.5米台
            print 'position 1!!!!!'
            ser_data[0]=6
            pid[2]._ref=-14.75-1
            print 'angle*****%f'%(pid[2]._ref)
        elif position[1]==-1:#右1米台
            print 'positon 2!!!!!'
            ser_data[0]=7
            #pid[2]._ref=-35
            pid[2]._ref=-36.5-2
            print 'angle*****%f'%(pid[2]._ref)
        elif position[0]==-1:#右0.5米台
            print 'positon 3!!!!!'
            ser_data[0]=8
            #pid[2]._ref=-51
            pid[2]._ref=-52-4
            print 'angle*****%f'%(pid[2]._ref)
        elif position[1]==1:#左0.5米台
            print 'position 4!!!!'
            ser_data[0]=9
            pid[2]._ref=16+1
            print 'angle*****%f'%(pid[2]._ref)
        else:
            pid[2]._ref=0+1
            
        pid[0]._fdb=data.chassis[0]-cali_xyz[0]
        pid[1]._fdb=data.chassis[1]-cali_xyz[1]
        pid[2]._fdb=data.chassis[2]-cali_xyz[2]
        if left_axes==-1 :    
            if abs(pid[0]._ref-pid[0]._fdb)>1000:
                print '***********%f,%f,%f'%(pid[0]._ref,pid[0]._fdb,abs(pid[0]._ref-pid[0]._fdb))
                
                if cali_flag==1:
                    pid[1]._ref=reload_position[1]-mid_position[1]
                elif cali_flag==0:
                    pid[1]._ref=reload_position[1]
            else:
                print 'MMMMMMMMMMMMM'
        if (cali_flag==1 and right_axes==-1):
            if abs(pid[1]._ref-pid[1]._fdb)>300:
                pid[0]._ref=0
        
        print 'pid_feb     =%f %f %f'%(pid[0]._fdb,pid[1]._fdb,pid[2]._fdb)
        
        tempx=-PID.PID.PID_Calc(pid[0])
        
        tempy=PID.PID.PID_Calc(pid[1])
        tempz=17*PID.PID.PID_Calc(pid[2])
        if pitch_down==1 or pitch_up==1 or position[0]==1 or position[0]==-1 or position[1]==1 or position[1]==-1:
            tempx=0
            tempy=0
        
        print ' X:ref:%f feb:%f'%(pid[0]._ref,pid[0]._fdb)
        print ' Y:ref:%f feb:%f'%(pid[1]._ref,pid[1]._fdb)
        print ' Z:ref:%f feb:%f'%(pid[2]._ref,pid[2]._fdb)
        print ' PID_result %f %f %f'%(tempx,tempy,tempz)
        pwmval[0]=(+tempx+tempz)
        pwmval[1]=(-0.5*tempx-0.866*tempy+tempz)
        pwmval[2]=(-0.5*0.9957*tempx+0.866*tempy+tempz)
        pwmval[3]=0


        #手动控制
    elif abs(speed[0])>0.03 or abs(speed[1])>0.03 or abs(speed[3])>0.03 :
        tempx=speed[1]
        tempy=speed[0]
        
        if tempx!=-0.0 and abs(tempy/tempx)<angle_tan_max:
            output=abs(tempy/tempx)
            print '!!!!!!%f'%(output)
            tempx=speed[1]
            if(abs(tempx)<0.14):
                tempx=0
            if(abs(tempx)>=0.14 and abs(tempx)<0.6):
                tempx=0.25*tempx
            
            tempy=0
        elif tempy!=-0.0 and abs(tempx/tempy)<angle_tan_max:
            output=abs(tempx/tempy)
            print '!!!!!!!!%f'%(output)
            tempx=0
            tempy=speed[0]
            if(abs(tempy))<0.8:
                tempy=0.25*tempy
        elif tempx==0 or tempy==0:
            tempx=speed[1]
            tempy=speed[0]
            if(abs(tempy))<0.8:
                tempy=0.25*tempy
            if(abs(tempx))<0.6:
                tempx=0.25*tempx
        else: 
            tempx=0
            tempy=0
        tempz=0.113*speed[3]
        pwmval[0]=600*(+tempx+tempz)
        pwmval[1]=600*(-0.5*tempx-0.866*tempy+tempz)
        pwmval[2]=600*(-0.5*0.9957*tempx+0.866*tempy+tempz)
        pwmval[3]=0
     
    if shoot_flag==1:
        ser_data[0]=1
        print 'shooting!'
    elif speed[4]==1:
        ser_data[0]=2
        print 'shooting speed up!'
    elif speed[5]==1:
        ser_data[0]=3
        print 'shooting speed down!'
    elif  position[1]==1:
        ser_data[0]=9 
    elif pitch_down==1:#近台
       
        ser_data[0]=5
        
    elif pitch_up==1:#远台
        
        ser_data[0]=4
        
    elif position[0]==1:#中1.5米台
        
        ser_data[0]=6
        
    elif position[1]==-1:#右1米台
        
        ser_data[0]=7
       
    elif position[0]==-1:#右0.5米台
        
        ser_data[0]=8
        
    else:
        ser_data[0]=0
   
    if position[2]==1:#丝杠停止
        ser_data[1]=5
    elif position[3]==1:#丝杠下降
        ser_data[1]=6
        
    elif position[4]==1:#摩擦轮初始化
        ser_data[1]=7
        
        cali_xyz=[data.chassis[0],data.chassis[1],data.chassis[2]]
        #print '--------------cali_xyz=%f %f %f'%(cali_xyz[0],cali_xyz[1],cali_xyz[2])
    elif position[5]==1:#丝杠上升
        ser_data[1]=8
    else:
        ser_data[1]=0
    #print ' X:ref:%f feb:%f'%(pid[0]._ref,pid[0]._fdb)
    #print ' Y:ref:%f feb:%f'%(pid[1]._ref,pid[1]._fdb)
    #print ' PID_result %f %f %f'%(tempx,tempy,tempz)
    #print 'for:%f lef:%f turn: %f stop=%f'%(tempx,tempy,tempz,stop_flag)
    #ser.write("DA%f/%f/%f/%f/\x0d\x0a" %(pwmval[0],pwmval[1],pwmval[2],pwmval[3]))
    # ser.write("DA%f/%f/%f/%f/%d%d\x0d\x0a" %(pwmval[0],pwmval[1],pwmval[2],pwmval[3],ser_data[0],ser_data[1]))
    # ser.flushInput()
    # ser.flushOutput()
    global ser
    
    for i in range(4):
        if (pwmval[i])<20 and (pwmval[i]>=-20):
            pwmval[i]=0.0
    print 'tempx=%f tempy=%f tempz=%f'%(tempx,tempy,tempz)
    print 'pwmval1=%f pwmval2=%f pwmval3=%f pwmval4=%f speed_up=%d speed_down=%d '%(pwmval[0],pwmval[1],pwmval[2],pwmval[3],ser_data[0],ser_data[1])
    sendData = 'DA%f/%f/%f/%f/%d%d\x0d\x0a' %(pwmval[0],pwmval[1],pwmval[2],pwmval[3],ser_data[0],ser_data[1])
    ser.sendToSTM32(sendData)
        
    #ser.write("DA%f/%f/%f/%f/%d%d\x0d\x0a" %(pwmval[0],pwmval[1],pwmval[2],pwmval[3],ser_data[0],ser_data[1]))
    return pwmval      
    
    

#遥控器数据，同步到全局变量
def deCodeData2(data):
    
    position=[0,0,0,0,0,0]
    speed   =[0,0,0,0,0,0,0]
    extra   =[0,0,0,0,0]
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
    extra[0]   =data.remote_data[13]
    extra[1]   =data.remote_data[14]
    extra[2]   =data.remote_data[15]
    extra[3]   =data.remote_data[16]
    extra[4]   =data.remote_data[17]
    
     
    #print 'decode %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f'%(position[0],position[1],position[2],position[3],position[4],position[5],speed[0],speed[1],speed[2],speed[3],speed[4],speed[5],speed[6],extra[0],extra[1],extra[2],extra[3],extra[4])
    return (position[0],position[1],position[2],position[3],position[4],position[5],speed[0],speed[1],speed[2],speed[3],speed[4],speed[5],speed[6],extra[0],extra[1],extra[2],extra[3],extra[4])
    
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
    managerTest = Manager()
    queueTest = managerTest.Queue()
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
