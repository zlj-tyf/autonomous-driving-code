#!/usr/bin/env python3
# -*- coding: UTF-8 -*-

#摄像头识别二维码并得到二维码位姿
import rospy
from std_msgs.msg import Float32MultiArray
import numpy as np
import time
import cv2
import cv2.aruco as aruco
from functions import *
font = cv2.FONT_HERSHEY_SIMPLEX #font for displaying text (below)
from digi.xbee.devices import XBeeDevice
# pygame用于获取键盘按键信息，控制发射指令
# 发送信息途中会出现错误，可能子板不存在（小车没开电）
from digi.xbee.exception import TransmitException
import datetime
import sys
#定义全局变量

class car_1:
    def __init__(self):
        self.xbee_flag = False
        self.open_xbee()
        rospy.init_node('initi_node', anonymous=True)
        rospy.Subscriber("/move", Float32MultiArray, self.callback_move)



    def open_xbee(self):
        PORT = "/dev/ttyUSB0"                       # xbee主板的com口
        BAUD_RATE = 115200                          # 波特率，主板子板要一致，如果控制多辆小车，建议115200， 9600太小
        self.REMOTE_NODE_ID = ['1','2','4','5','6','7']                     # xbee的id，子板要配置，主板不用配置。使用的是186固件，升级过的
        self.IDS = [1,2,4,5,6,7]   #与xbee的id对应的二维码id值
        self.inputdata = [999,999,999,999,999,999]
        self.device = XBeeDevice(PORT, BAUD_RATE)        # 创建一个主板对象
        self.end_device = []             # 用于存储已经连接上的子板
        self.device.open()
        xbee_network = self.device.get_network()

        for i in range(len(self.REMOTE_NODE_ID)):
            try:
                print("connecting %d"%self.IDS[i])
                tmp = xbee_network.discover_device(self.REMOTE_NODE_ID[i])  # 用于链接子板
                time.sleep(0.5)
            except ValueError:
                print("连接子板%d出现错误"%(i+1))
            self.end_device.append(tmp)
            print(tmp)
            # if tmp == None:
            #     break

        if None  not in self.end_device:
            self.xbee_flag = True
        else:
            self.xbee_flag = False

    def callback_move(self,msg):
        self.id_index = self.IDS.index(int(msg.data[1]))
        self.inputdata[self.id_index] = int(msg.data[0])

    def send_message(self):
        while not rospy.is_shutdown():
            start = datetime.datetime.now()
            # if self.xbee_flag :
            if True:
                for i in range(len(self.inputdata)):
                    try:
                        self.device.send_data(self.end_device[i], str(self.inputdata[i]))
                        time.sleep(0.02)
                    except :
                        if self.end_device[i] != None:
                            print("error !!!",i)
                    #print(self.end_device[i], str(self.inputdata[i]))
            end = datetime.datetime.now()
            print ("运行一次时间：",end-start)
            print("inputdata:",self.inputdata)


        for j in range(len(self.end_device)):           #也可以用while not rospy.is_shutdown()： ctrl+c停止运动
            time.sleep(0.1)
            self.device.send_data(self.end_device[j], "999")
        sys.exit(0)
if __name__ == '__main__':
    try:
        detector = car_1()

            #if detector.xbee_flag :
        detector.send_message()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
