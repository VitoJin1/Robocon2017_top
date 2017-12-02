# -*- coding: UTF-8 -*-

import serial

class SerialCom(object):
    def __init__(self, serialPort, baudrate = 230400, bytesize = 8, parity = serial.PARITY_NONE, stopbits = 1):
        self.ser = serial.Serial(serialPort, baudrate=baudrate, bytesize=bytesize, parity=parity, stopbits=stopbits)
        # self.ser.flushInput()
        # self.ser.flushOutput()
        self.clearBuffer
    
    def sendToSTM32(self, data):
        self.ser.write(data)
        # self.ser.write("DA%f/%f/%f/%f/%d%d\x0d\x0a" %(pwmval[0],pwmval[1],pwmval[2],pwmval[3],ser_data[0],ser_data[1]))
        # self.ser.flushInput()
        # self.ser.flushOutput()
        self.clearBuffer()
    
    def readSTM32Data(self, dataLen = 1):
        content = self.ser.read(dataLen)
        # print content
        return content

    def clearBuffer(self):
        self.ser.flushInput()
        self.ser.flushOutput()
    
    def closeSerial(self):
        self.ser.close()
        