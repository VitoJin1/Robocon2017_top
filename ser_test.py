# -*- coding: UTF-8 -*-
import serial
import rospy
import os
import time
import numpy as np
ser = serial.Serial('/dev/ttyUSB0',baudrate=115200, bytesize=8, parity=serial.PARITY_NONE, stopbits=1)
pwmval=[100.0,100.0,100.0,100.0,1,2]
ser.write("DA%f/%f/%f/%f/%d%d\x0d\x0a" %(pwmval[0],pwmval[1],pwmval[2],pwmval[3],pwmval[4],pwmval[5]))