# -*- coding: UTF8 -*-

#矩阵变换测试，theta为局部相对于全局变换的角度，顺时针为正，逆时针为负
def _matrixCal(localX, localY, theta, caliX, caliY):
    import numpy as np
    import math
    alpha = np.array([localX, localY])
    beta  = np.array([[math.cos(theta), -math.sin(theta)], [math.sin(theta), math.cos(theta)]])
    lemma = np.array([caliX, caliY])
    ans = np.dot(alpha, beta) + lemma
    print ans
    return (ans[0], ans[1])
import math
XY0=[9047, -168]
XY1=[]
XY2=[]

# XY1=_matrixCal(XY0[0],XY0[1],math.pi/4,0,0)
# XY2=_matrixCal(XY1[0],XY1[1],math.pi/4,0,0)
# 
# print 'final%f%f'%(XY2[0],XY2[1])

from BaseLib import matrixCal

XY1=matrixCal(XY0[0],XY0[1],-0.01111,0,0)
# XY2=matrixCal(XY1[0],XY1[1],math.pi/4,0,0)
