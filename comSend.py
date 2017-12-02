# -*- coding: UTF-8 -*-

import os
import sys
import time

import numpy as np
import serial

ser = serial.Serial('/dev/ttyUSB0',baudrate=230400, bytesize=8, parity=serial.PARITY_NONE, stopbits=1)
ser.flushInput()
ser.flushOutput()
count=0;

def sendData(data):
    ser.write(data)

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
	# _s=ser.read(28)
	# _s=stringToInt(_s)
	# _s = getInfo(_s)
	# if _s == None:
	# 	print 'can not get package from STM32'
	# else:
	# 	print _s
	# time.sleep(0.5)
	content = ''
	tmp = (ord(ser.read()))
	# print tmp
	if 0xd == tmp:
		# print 'b: ' + tmp
		tmp = (ord(ser.read()))
		if 0xa == tmp:
			# print 'b ' + tmp
			content = (ser.read(24))
		tmp = (ord(ser.read()))
		if 0xa == tmp:
			# print 'e:' + tmp
			tmp = (ord(ser.read()))
			if 0xd == tmp:
				# print 'e ' + tmp
				# print 'content %s, len %d' % (str(content), len(content))
				rawValue = intToDouble(content)
				print rawValue
				# print 'contetnt ％s, len %d: '％(str(content), len(content))
	if len(content) > 0:
		pass		
	
	from time import sleep
	# sleep(0.5)
	ser.flushInput()
	ser.flushOutput()
