#!/usr/bin/env python3
# -*- coding: UTF-8 -*-

#功能：十字路口调度，订阅/cam_1 2 3 4, 发布/vertical_permission (为-1时允许东西向车通行，为1时允许南北向车辆通行)

#摄像头识别二维码并得到二维码位姿
#from asyncio.windows_events import NULL
from math import trunc
from threading import get_ident
from numpy.core.records import array
import rospy
import random
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int16
import numpy as np
import time
import cv2
import cv2.aruco as aruco
from functions import *
import way_1
import way_4
import way_3 
import way_2
font = cv2.FONT_HERSHEY_SIMPLEX #font for displaying text (below)
from digi.xbee.devices import XBeeDevice
import  digi
from digi.xbee.exception import TransmitException
import random
import time

#路径
ap1_1,ap1_2,ap2_1,ap2_2,p1b,p2b,bc, da = way_1.way()
#轴距
l  = 45.641

#---------------未使用--------------------------
topic = ["/cam_1", "/cam_2", "/cam_3", "/cam_4"]

#用于计算存储路口附近车道车辆的坐标、
class V_near_intersc:
    def __init__(self, cam = 1):
        rospy.Subscriber(topic[cam],Float32MultiArray, self.callback_image(cam))
        
    # def callback_image(self, msg, cam):
#--------------------未使用------------------------


class intersc_manage:
    def __init__(self):
        rospy.init_node('intersc_manage', anonymous=True)
        # rospy.Subscriber("/cam_1",Float32MultiArray,self.callback_image13(1))
        # rospy.Subscriber("/cam_3", Float32MultiArray, self.callback_image13(3)) 
        # rospy.Subscriber("/cam_2", Float32MultiArray, self.callback_image24)
        # rospy.Subscriber("/cam_4", Float32MultiArray, self.callback_image24) 
        self.vertical_permission = 1
        self.pub_permission = rospy.Publisher('/vertical_permission', Int16, queue_size=1 )#用于判断南北向车能通行（1）还是东西向车能通行（-1）
        self.time_init = time.time()

#主函数++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    def compute(self):#定时红绿灯，十秒东西向通行，十秒南北向通行 需要调节的参数：vertical_period,horizontal_period
        vertical_period = 10
        horizontal_period = 10
        while not rospy.is_shutdown():
            time_now = time.time()
            if self.vertical_permission == 1:
                if time_now - self.time_init >= vertical_period:#计时到达10s
                    self.time_init = time_now#更新self.time_init
                    self.vertical_permission = self.vertical_permission * (-1)#改变通行方向
                # print( self.vertical_permission, "time_remaining:", vertical_period - (time_now-self.time_init))
                rospy.loginfo( "%s time_remaining: %s", self.vertical_permission, vertical_period - (time_now-self.time_init))

            elif self.vertical_permission == -1:
                if time_now - self.time_init >= horizontal_period:#计时到达10s
                    self.time_init = time_now#更新self.time_init
                    self.vertical_permission = self.vertical_permission * (-1)#改变通行方向
                # print( self.vertical_permission, "time_remaining:", horizontal_period - (time_now-self.time_init))
                rospy.loginfo( "%s time_remaining: %s", self.vertical_permission, horizontal_period - (time_now-self.time_init))

                
        
            #发布话题
            self.pub_permission.publish(Int16(data = self.vertical_permission))
            # self.pub_permission.publish(Int16(data = 1))
            
                
        


if __name__ == '__main__':
    try:
        manager = intersc_manage()
        manager.compute()

    except rospy.ROSInterruptException:
        pass
