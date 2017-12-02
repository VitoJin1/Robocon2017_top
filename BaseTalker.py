# -*- coding: UTF-8 -*-
#!/usr/bin/python

import rospy

class BaseTalker(object):
    '''
    base talker
    '''

    def __init__(self, nodeName, topic, msgType, rate=10, queueSize=1):
        '''
        :nodeName: name of this talker nodeName
        :topic: the topic that push to
        :msgType: the type of the message
        :rate: the rate of publishing, default=10
        :queueSize: the size of the waitting queue, default-10
        '''
        self._pubulisher = rospy.Publisher(topic, msgType, queue_size=queueSize)
        try:
            rospy.init_node(nodeName, anonymous=True)
        except Exception,e:
            print '#error: cant note init talker'
        self._rate = rospy.Rate(rate)
        

    def talk(self, content):
        if not rospy.is_shutdown():
            self._pubulisher.publish(content)

