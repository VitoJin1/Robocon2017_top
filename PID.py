# -*- coding: UTF-8 -*-
#
class PID(object):
    
    def __init__(self, ref = 0.0, fdb = 0.0, err = [0, 0], Kp = 0.45, Ki = 0.0, Kd = 0.0, Integral = 0.0, KiMax = 0.0, output = 0.0, outputMax = 900, i = 1):
        self._ref = ref
        self._fdb = fdb
        self._err = err
        self._Kp = Kp
        self._Ki = Ki
        self._Kd = Kd
        self._Integral = Integral
        self._KiMax = KiMax
        self._output = output
        self._outputMax = outputMax 
        self._i = i
    
    def PID_init(self, ref = 0.0, fdb = 0.0, err = [0, 0], Kp = 0.45, Ki = 0.0, Kd = 0.0, Integral = 0.0, KiMax = 0.0, output = 0.0, outputMax = 900, i = 1):
        self._ref = ref
        self._fdb = fdb
        self._err = err
        self._Kp = Kp
        self._Ki = Ki
        self._Kd = Kd
        self._Integral = Integral
        self._KiMax = KiMax
        self._output = output
        self._outputMax = outputMax 
        self._i = i
        
    def PID_update_ref(self, ref):
        self._ref = ref

    def PID_Calc(self):
        limit=2000.0
        increase_step=6
        if self._err == None:
            print 'error# 1, self._error is None'
            self._err = [0, 0]
        
        self._err[1] = self._ref - self._fdb
        if abs(self._Integral)>20:
            self._Integral=self._Integral
        else:
            self._Integral += self._err[1]
        #print 'Intergral**********=%f'%(self._Integral)
        # from math import abs
        #误差较大时用斜坡启动的方法,limit代表斜坡的高度，step代表斜坡的斜率
        if abs(self._err[1]) > limit:
            print 'limiting!!\n'
            #print 'error1=%f'%self._err[1]
            self._output = self._i *increase_step 
            self._i +=increase_step
            #print 'error%f'%(self._err[1])
            #print 'output%f'%(self._output)
            if self._err[1] < 0:
                self._output = -self._output
        elif abs(self._err[1]) <= limit :
            print 'not limiting!'
            #print 'error2=%f'%self._err[1]
            self._output = self._Kp * self._err[1] + self._Ki * self._Integral + self._Kd * (self._err[1] - self._err[0])
            self._i = 1
        
        if self._err[1] > 0 and abs(self._output) > self._outputMax:
            self._output = self._outputMax
            print '+++'
        elif self._err[1] < 0 and abs(self._output) > self._outputMax:
            self._output = -self._outputMax
            print '---'
        self._err[0] = self._err[1]
        
        #return self
        
        #print 'output:**********%f'%(self._output)
        return self._output
    
    @staticmethod
    def CalCMSpeed(pids, pos_x, pos_y, zangle):
        '''
        pids: list of PIDs = [X, Y, Z]
        '''
        from math import sqrt
        pids[0]._fdb = -pos_x * sqrt(2.) + pos_y * sqrt(2)
        pids[1]._fdb = -pos_x * sqrt(2.) - pos_y * sqrt(2)
        pids[2]._fdb = zangle
        
        def pidCale(x):
            x.PID_Calc()
            return x
        pids = map(pidCale, pids)

        SpeedX = pids[0]._output
        SpeedY = pids[1]._output
        SpeedZ = pids[2]._output

        return (SpeedX + SpeedY - SpeedZ, -SpeedX + SpeedY + SpeedZ, SpeedX - SpeedY + SpeedZ, SpeedX + SpeedY + SpeedZ)

    @staticmethod
    def ClearCMSpeed():
        return(0, 0, 0, 0)
