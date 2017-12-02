# -*- coding: UTF-8 -*-

import rospy

class BaseListener(object):
    '''
    base listener
    '''

    def __init__(self, nodeName, topic, msgType, callBack, callBackArgs=None, queueSize=1):
        '''
        :nodeName: name of this talker nodeName
        :topic: the topic that push to
        :msgType: the type of the message
        :callBack: call back function
        :callBackArgs: the parameter of call back function
        '''
        try:
            rospy.init_node(nodeName, anonymous=True)
        except Exception, e:
            print e
        rospy.Subscriber(topic, msgType, callBack, callback_args=callBackArgs, queue_size=queueSize)
        rospy.spin()
