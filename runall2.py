# -*- coding: UTF-8 -*-

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
    # os.system(baseCmd + '''"python ./comtest.py %s;exec zsh"''' % comPort)
    # time.sleep(0.500)
    os.system(baseCmd + '''"python ./chassis_control_change.py %s;exec zsh"''' % comPort)
    time.sleep(0.500)
    os.system(baseCmd + '''"python ./remoteRun.py;exec zsh"''')
    
    main()
