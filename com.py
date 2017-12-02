# -*- coding: UTF-8 -*-

import serial
import numpy as np
import time
import os
import sys

ser = serial.Serial('/dev/ttyUSB25',baudrate=115200, bytesize=8, parity=serial.PARITY_NONE, stopbits=1)
ser.flushInput()
ser.flushOutput()
count=0;

# while True:
# 	_s = ser.read(1)
# 	#_s = list(_s)
# 	if(_s=="0d"):
# 		count=count+1
# 		print count
# 	elif count==1:
# 		if(_s=="0a"):
# 			count=count+1
		
# 		elif(_s=="0d"):
# 			count=0
# 	elif count==2:
# 		_s = ser.read(24)
# 		output = list(_s)
# 	elif count==3:
# 		if(_s=="0a"):
# 			count=count+1
# 	elif count==4:
# 		if(ch=="0d"):
# 			print output	
# 			count=0
# 	else :
# 		count=0
			

# 	#print _s
	
# 	time.sleep(0.5)

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