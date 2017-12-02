# -*- coding: UTF-8 -*-

import PID

pid = [PID.PID(),PID.PID(),PID.PID()]
pid[0].PID_init()
pid[0].PID_update_ref(-10000.0)
pid[0]._Kp=1

print pid[0]._Kp

#print pid[0]._fdb
#print pid[0]._err[1]
print PID.PID.PID_Calc(pid[0]) 
 
#print PID.PID.CalCMSpeed(pid, 0, 0, 0)

