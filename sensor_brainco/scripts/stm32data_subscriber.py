#!/usr/bin/env python
# -*- coding: utf-8 -*-

########################################################################
####          Copyright 2020 GuYueHome (www.guyuehome.com).          ###
########################################################################

# 该例程将订阅/person_info话题，自定义消息类型uart_com1::Person

import rospy
from uart_stm32.msg import stm32data

def stm32dataInfoCallback(msg):
    rospy.loginfo("Subcribe stm32data Info: %d %d %d %d %d %d %d %d %d %d %d %d %d %d", 
            msg.voltage00,
            msg.voltage01,
            msg.voltage02,
            msg.voltage03,
            msg.voltage04,
            msg.voltage05,
            msg.voltage06,
            msg.voltage07,
            msg.voltage08,
            msg.voltage09,
            msg.voltage10,
            msg.voltage11,
            msg.voltage12,
            msg.voltage13
            )

def stm32data_subscriber():
	# ROS节点初始化
    rospy.init_node('stm32data_subscriber', anonymous=True)

	# 创建一个Subscriber，订阅名为/stm32data_info的topic，注册回调函数stm32dataInfoCallback
    rospy.Subscriber("/stm32data_info", stm32data, stm32dataInfoCallback)

	# 循环等待回调函数
    rospy.spin()

if __name__ == '__main__':
    stm32data_subscriber()
