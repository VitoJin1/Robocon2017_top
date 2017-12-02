# -*- coding: UTF-8 -*-

#矩阵变换测试，theta为局部相对于全局变换的角度，顺时针为正，逆时针为负
def matrixCal(localX, localY, theta, caliX, caliY):
    import numpy as np
    import math
    alpha = np.array([localX, localY])
    beta  = np.array([[math.cos(theta), -math.sin(theta)], [math.sin(theta), math.cos(theta)]])
    lemma = np.array([caliX, caliY])
    ans = np.dot(alpha, beta) + lemma
    print ans
    return (ans[0], ans[1])
    