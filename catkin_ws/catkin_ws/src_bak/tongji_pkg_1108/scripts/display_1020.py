#!/usr/bin/env python3
# -*- coding: UTF-8 -*-
import time
import rospy
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Image
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError
import sys
import cv2.aruco as aruco
from functions import *
font = cv2.FONT_HERSHEY_SIMPLEX #font for displaying text (below)
import way_1, way_2, way_3, way_4

#订阅四个/img 将四幅图拼接成一幅，便于展示

bridge = CvBridge()
img = [[],[],[],[]]

#input:data--订阅话题的msg i：第几个摄像头传来的话题
#功能：将/img_i中的msg存为图像，并且将四幅画面拼接在一起
def callback(data,i):
    global  img
    if True:
        img[i-1] = bridge.imgmsg_to_cv2(data, "bgr8")#将话题中的msg解码为图像存入对饮的img[i]中
        if img[0] != [] and img[1] !=[] and img[2] != [] and img[3] != []:
            pic = [img[3-1],img[2-1],img[4-1],img[1-1]]
            angle = [0,0,0,0]
            result = pic_concat(pic, angle)
            cv2.imshow("frame" , result)
            cv2.waitKey(1)
    else:
        pass
    

def pic_concat( pic, angle):
    #旋转任意度数,旋转之后图像的宽高并没有交换
    def rotate(image, angle, scale=1):
        w = image.shape[1]
        h = image.shape[0]
        #rotate matrix
        M = cv2.getRotationMatrix2D((w/2,h/2), angle, scale)
        #rotate
        image = cv2.warpAffine(image,M,(w,h))
        return image
    #图片归一化、旋转图片
    for i in range(0,4):
        pic[i] = cv2.resize(pic[i],dsize=(500,500),fx=1,fy=1,interpolation=cv2.INTER_LINEAR)#将img尺寸缩放为500*500
        pic[i] = rotate(pic[i],angle[i])
    #拼接图片
    output_1 = np.hstack([pic[3],pic[1]])#第一行拼接
    output_2 = np.hstack([pic[2],pic[0]])#第二行拼接
    output = np.vstack([output_1, output_2])#拼接为四宫格
    return output


def callback1(data):
    callback(data, 1)
    
def callback2(data):
    if True:
        img[2-1] = bridge.imgmsg_to_cv2(data, "bgr8")
    else:
        pass

def callback3(data):
    if True:
        img[3-1] = bridge.imgmsg_to_cv2(data, "bgr8")
    else:
        pass

def callback4(data):
    if True:
        img[4-1] = bridge.imgmsg_to_cv2(data, "bgr8")
    else:
        pass



    
 
def displayWebcam():
    rospy.init_node('display', anonymous=True) 
    rospy.Subscriber('/img_1', Image, callback1)
    rospy.Subscriber('/img_2',Image, callback2)
    rospy.Subscriber('/img_3',Image, callback3)
    rospy.Subscriber('/img_4',Image, callback4)

    rospy.spin()
 
if __name__ == '__main__':
    displayWebcam()
