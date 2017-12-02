# -*- coding: UTF-8 -*-
#6/4调试完成，改脚本，拨杆，旋转副值
import sys
import os
import time

def main():
    pass

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print 'please input the console port'
        sys.exit()
    baseCmd = 'gnome-terminal -x zsh -c '
    comPort = sys.argv[1]
    # chassisPort = sys.argv[2]
    
    os.system('sudo chmod 666 %s' % comPort)
    # os.system('sudo chmod 666 %s' % chassisPort)
    
    os.system(baseCmd + '''"roscore;exec zsh"''')
    time.sleep(0.5)
    os.system(baseCmd + '''"rosrun joy joy_node;exec zsh"''')
    time.sleep(0.500)
    os.system(baseCmd + '''"python ./remote_control_new.py;exec zsh"''')
    time.sleep(0.500)
    
    os.system(baseCmd + '''"python ./chassis_control_new_back.py %s;exec zsh"''' % comPort)
    time.sleep(0.500)
    os.system(baseCmd + '''"python ./comtest.py %s;exec zsh"''' % comPort)
    time.sleep(0.500)
    
    
    
    
    main()
