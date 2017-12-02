# -*- coding: UTF-8 -*-

import serial
import numpy as np
import time
import os
import sys

ser = serial.Serial('/dev/ttyUSB2',baudrate=115200, bytesize=8, parity=serial.PARITY_NONE, stopbits=1)
ser.flushInput()
ser.flushOutput()
count=0
a=-123456789.213
b=123.43
c=123.45
d=234.35
def getInfo(l):
	if l[0] == '0xd' and l[1] == '0xa' and l[26] == '0xa' and l[27] == '0xd':
		return l[2:25]
	# print l
	return None

def stringToInt(s):
	ans = []
	for _ in s:
		ans.append(hex(ord(_)))
	return ans
	
def intToDouble(int_6):
	import struct
	# print int_6
	return struct.unpack('ffffff', int_6)

while True:
    
    
    ser.write("%f/%f/%f/%f/\x0d\x0a" %(a,b,c,d))
    from time import sleep
    sleep(0.5)
    	
    ser.flushInput()
    ser.flushOutput()

