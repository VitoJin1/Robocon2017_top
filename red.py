# -*- coding: UTF-8 -*-
#6/4调试完成，改脚本，拨杆，旋转副值
import sys
import os
import time

# modified by AIF
import serial
import serial.tools.list_ports 
# end AIF

def main():
    pass

if __name__ == '__main__':
    # modified by AIF

    # if len(sys.argv) < 2:
    #     print 'please input the console port'
    #     sys.exit()
    
    # end AIF
    baseCmd = 'gnome-terminal -x zsh -c '
    comPort = list(serial.tools.list_ports.comports())
    # modified by AIF
    if len(comPort) <= 0:
        print 'can not get serial list'
        exit(-1)  
    else:
       comPort = str(comPort[0][0])
    # end AIF

    
    # chassisPort = sys.argv[2]
    
    os.system('sudo chmod 666 %s' % comPort)
    # os.system('sudo chmod 666 %s' % chassisPort)
    
    os.system(baseCmd + '''"roscore;exec zsh"''')
    time.sleep(0.5)
    os.system(baseCmd + '''"rosrun joy joy_node;exec zsh"''')
    time.sleep(0.500)
    os.system(baseCmd + '''"python ./remote_control_new.py;exec zsh"''')
    time.sleep(0.500)
   
    os.system(baseCmd + '''"python ./chassis_control_red.py %s;exec zsh"''' % comPort)
    time.sleep(0.500)
    os.system(baseCmd + '''"python ./comtest.py %s;exec zsh"''' % comPort)
    time.sleep(0.500)
    
    
    
    
    main()
