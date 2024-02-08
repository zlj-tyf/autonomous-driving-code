#!/usr/bin/env python3
# -*- coding: UTF-8 -*-

#摄像头识别二维码并得到二维码位姿
#from asyncio.windows_events import NULL
from math import trunc
from threading import get_ident
from numpy.core.records import array
import rospy
import random
from std_msgs.msg import Float32MultiArray, Int16
import numpy as np
import time
import cv2
import cv2.aruco as aruco
from functions import *
from way_1 import Point_c
import way_1
import way_4
font = cv2.FONT_HERSHEY_SIMPLEX #font for displaying text (below)
from digi.xbee.devices import XBeeDevice
import  digi
from digi.xbee.exception import TransmitException
import message_filters

#路径
path_p1a, path_ab,path_b8= way_4.way()
#轴距
l = l = 45.641



#定义全局变量
'''
1. way_2中的返回路径得要改
2. way_2中的点需要测量
3. 公共点也需要测量

'''


class compute_node:
    def __init__(self):
        rospy.init_node('compute_node_4', anonymous=True)
        rospy.Subscriber("/cam_4", Float32MultiArray, self.callback_image4) 
        rospy.Subscriber("/vertical_permission", Int16, self.callback_intersc)#十字路口调度topic，-1为东西向通行，即车可从b出库
        rospy.Subscriber("/enPermission_cam1",Int16,self.callback_enpermission)
        self.pub_move = rospy.Publisher('/move', Float32MultiArray, queue_size=1)
        self.pub_en_permission = rospy.Publisher("/enPermission_cam4",Int16, queue_size = 1)
        self.image_flag = False
        self.flag_out = 0
        
        self.pub_flag_out = rospy.Publisher('/flag_out', Float32MultiArray, queue_size=1)
        
        self.vertical_permission = -1
        self.en_permission = 1 #1-允许进入4区 0-不允许进入4区
        
        #记录5帧，若有4帧数为0，则认为flag_out = 0
        self.out_count_all = 0#总帧数
        self.out_count_now = 0 #计数帧，如果下个场景没车加一

    def callback_intersc(self,msg):
        self.vertical_permission = msg.data


    #传入cam2小车位姿       
    def callback_image4(self, msg):
        self.pose_now = []
        self.theta = []
        self.ids = []
        
        if len(msg.data) != 0 :
            for i in range(len(msg.data)//4):       #对每个二维码信息读取, 并且二维码的数据都已经根据id的大小排序 //:整除，舍去小数
                self.ids.append(int(msg.data[i*4]))
                self.pose_now.append([msg.data[i*4+1], msg.data[i*4+2]])  #self.pose_now[i][3] 为id , 前面为xy
                self.theta.append(msg.data[i*4+3])
                self.image_flag = True
        else :
            self.image_flag = False
        
    def callback_enpermission(self,msg):
        if msg.data == 1:
            self.flag_out = 0
        else:
            self.flag_out = 1

       
        
    #判断3区入口有没有车
    #有缺陷，进一步改进：判断衔接处-a整个线段是不是有车
    def callback_image1(self, msg):   
                #记录到第五帧时，统计入口没车的帧数，若大于1则认为入口有车，即flag_out = 1
        if self.out_count_all == 4:
            if self.out_count_now > 2:
                self.flag_out = 1
            else:
                self.flag_out = 0
            #重置计数器
            self.out_count_now = 0
            self.out_count_all = 0
        else:
            self.out_count_all += 1
            if len(msg.data) != 0 :
                for i in range(len(msg.data)//4):      #判断本帧3区每一个是否靠近小车入口a，若是，则out_count_now +=1
                    if distance(msg.data[i*4+1], msg.data[i*4+2], way_1.Point_a[0], way_1.Point_a[1]) < 40 and msg.data[i*4] < 10:
                        self.out_count_now  += 1
                        break
        
        array = [self.flag_out]
        self.pub_flag_out.publish(Float32MultiArray(data = array))          

                
       
        
        
       
        
      
    # #判断cam4中是否有小车进入    
    # def callback_image2(self, msg):
    #     self.out4 = 0
    #     self.all = []
    #     if len(msg.data) != 0 :
    #         for i in range(len(msg.data)//4):       #对每个二维码信息读取, 并且二维码的数据都已经根据id的大小排序
    #             self.all.append(msg.data[i*4+3])
    #         if 1 in self.all:
    #             self.out4 = 1
    #         else :
    #             self.out4 = 0
    #     else:
    #         self.out4 = 0
            
    #判断point附近的车辆id号，如果theta!=-1, 则还要看车辆朝向角是否和theta接近 #待改进：theta不缺省时有逻辑bug
    def get_id(self, point, theta = -100):
        dis_all = []
        x = point[0]
        y = point[1]
        if len(self.pose_now)> 0:
            for i in range(len(self.pose_now)):
                if theta == -100:
                    dis_all.append(distance(self.pose_now[i][0],self.pose_now[i][1],x,y))
                else:
                    if abs(abs(self.theta[i]) - abs(theta)) > 1 :
                        dis_all.append(100)
                    else:
                        dis_all.append(distance(self.pose_now[i][0],self.pose_now[i][1],x,y))
                        
            index = np.argmin(dis_all)
            if dis_all[index] < 40:
                index_min = np.argmin(dis_all)
                if not self.ids is None:
                    if index_min < len(self.ids):
                        id = self.ids[np.argmin(dis_all)]
                    else:
                        id = 0
                else: 
                    id = 0
            else:
                id = 0
        else: id = 0
        return id

    def send_stop(self,id_name):
        array = []
        array.append(999)
        array.append(float(id_name))
        self.pub_move.publish(Float32MultiArray(data = array))
        
    def send_stright(self,id_name):
        array = []
        array.append(350)
        array.append(float(id_name))
        self.pub_move.publish(Float32MultiArray(data = array))

    def send(self, id_name, v, path, l, Lf):
        try:
            array = []
            conseq_name  =  self.ids.index(id_name)
            input_data, ind = move(self.pose_now[conseq_name], self.theta[conseq_name], v, path, l, Lf)  #发送指定的name的pose_now
            array.append(input_data)
            array.append(float(id_name))
            self.pub_move.publish(Float32MultiArray(data = array))
            #删除print后小车行进卡顿消失
            # print("id:", id_name)
            # print("当前坐标", self.pose_now)
            # print("当前数据", input_data)
            # print("-------------")
        except:
            print("send false")
            
    def send_judge_arrive(self, pose_goal, id_name, v, path, l, Lf):
        array = []
        try:
            if if_filled_specific(self.pose_now[self.ids.index(id_name)], pose_goal) :
                array.append(999)
                array.append(float(id_name))
                self.pub_move.publish(Float32MultiArray(data = array))
                print("arrive", pose_goal)
                #self.data_update()          #当前位姿计算完后更新数据，用于while判断和处理
                return True
            else :
                self.send(id_name = id_name, v = v, path = path, l = l, Lf = Lf)
                #self.data_update()          #当前位姿计算完后更新数据，用于while判断和处理
                return False
        except:
            print("sja failed")


    def compute(self):
        vf = 3  #前进速度
        l = 40  #车轴距
        Lf = 80 #前视距离
        Point_a = way_4.Point_a
        Point_b = way_4.Point_b
        Point_8 = way_4.Point_8
        Point_1 = way_4.Point_1
        
        flag_1a = 0 #1a段被占用为1
        flag_ab = 0#ab段是否被占用，存在id_ab出发未到达和id_ab等在a处两种情况
        flag_b = 0#b点是否有车
        
        id_1_tmp = 0#驶入场景的车号
        id_ab_tmp = 0 #行驶在ab上的车号
        id_b8_tmp = 0 #经过b点到出口的车号
        id_a_tmp = 0#停在a点的车号
        id_b_tmp = 0#停在b点的车号
        
        self.id_1 = 0#驶入场景的车号
        self.id_ab = 0 #行驶在ab上的车号
        self.id_b8 = 0 #经过b点到出口的车号
        self.id_a = 0#停在a点的车号
        self.id_b = 0#停在b点的车号
        self.flag_out = 0


        while not rospy.is_shutdown():
            
            if self.image_flag == True:

                #-------------------------------------------
                #获取各个检测点的车号  角度需要根据场景修改
                id_1_tmp = self.get_id(Point_1,1.51)#entrance 带角度判断
                id_a_tmp = self.get_id(Point_a,1.51)#a
                id_b_tmp = self.get_id(Point_b, 0)
                # id_ab_tmp = self.get_id(Point_a,1.51)#a-b
                id_b8_tmp = self.get_id(Point_b,0)#b-8
                
                rospy.loginfo("id_1:%s, id_a:%s, id_ab:%s, id_b:%s, id_b8: %s", self.id_1, self.id_a, self.id_ab, self.id_b, self.id_b8)
                rospy.loginfo(self.flag_out)
                
                                    
                #检测点1 a b 8,停车点a b
                
                #获取各个检测点的状态
                if id_1_tmp != 0 and id_1_tmp in self.ids:#防止毛刺使得id_x误识别
                    flag_1a = 1#1-a有车
                    self.id_1 = id_1_tmp
                    
                # if id_ab_tmp != 0 and id_ab_tmp in self.ids and self.id_ab == 0:
                #     flag_ab = 1
                #     self.id_ab = id_ab_tmp
                    
                # if id_b8_tmp != 0 and id_b8_tmp in self.ids :
                #     flag_b = 1
                #     self.id_b8 = id_b8_tmp
                # else:
                #     flag_b = 0#b点有车，但可能有几帧识别不到导致flag_b = 0 待修改以提高鲁棒性
                
                if id_a_tmp != 0 and id_a_tmp in self.ids:
                    self.id_a = id_a_tmp
                else:
                    self.id_a = 0
                    
                if id_b_tmp != 0 and id_b_tmp in self.ids:
                    self.id_b = id_b_tmp
                else:
                    self.id_b = 0

                
                # print(self.id_1,self.id_ab,self.id_b8)
                
                #补丁：有时候ab b8之间的交接点b会误识别，导致id_ab 与 id_b8等于同一个车
                if self.id_ab == self.id_b8:
                    self.id_ab = 0
                
                #1-a
                if self.id_1 != 0:
                    if if_filled(self.pose_now, Point_a) == 0:
                        self.send(self.id_1,vf,path_p1a,l,Lf)
                        print("id_1:en-a")
                    else:
                        print("id_1:arrive at a")
                        self.send_stop(self.id_1)
                        #self.id_ab = self.id_1
                        self.id_1 = 0
                        
                # #a-b    
                # if self.id_ab != 0:
                #     if self.id_b8 != 0:
                #         self.send_stop(self.id_ab)
                #         print("stop at a")
                #     else:
                #         #a-b巡线
                #         if if_filled(self.pose_now, Point_b) == 0:
                #             print("id_ab:a-b")
                #             self.send(self.id_ab, vf, path_ab, l, Lf)
                #         else:
                #             self.send_stop(self.id_b8)
                #             print("id_ab:arrive at b")
                #             #self.id_b8 = self.id_ab
                #             self.id_ab = 0
                
                #a-b
                if self.id_ab == 0:
                    self.id_ab = self.id_a
                else:
                    if self.id_b8 != 0 or self.id_b != 0:
                        self.send_stop(self.id_ab)
                        print("stop at a")
                    else:
                        #a-b巡线
                        if if_filled(self.pose_now, Point_b) == 0:
                            print("id_ab:a-b")
                            self.send(self.id_ab, vf, path_ab, l, Lf)
                        else:
                            self.send_stop(self.id_ab)
                            print("id_ab:arrive at b")
                            #self.id_b8 = self.id_ab
                            self.id_ab = 0

                        
                if self.id_b8 == 0:
                    self.id_b8 = self.id_b
                #b-8 加入十字路口调度功能(判断是否允许小车离开场景)
                elif self.id_b8 != 0:                       
                    if self.flag_out == 0:
                        #id_b8行驶出场景
                        print("id_b8:b-8")
                        if self.vertical_permission == 1:#南北向通行，东西向不能通行，停止出场景
                            self.send_stop(self.id_b8)
                        else:#d东西向可以通行（绿灯）
                            if if_filled(self.pose_now, Point_8) == 0:#小车未抵达出口p8
                                self.send(self.id_b8,vf,path_b8,l,Lf)#巡线b-8
                            else:#小车抵达出口p8
                                # while not self.id_b8 in self.ids:
                                print("moving out of cam2")
                                self.send_stright(self.id_b8)#小车直行出场景
                                self.id_b8 = 0#重置id_b8
                    else:#继续监听b
                        print("stop at b")
                        self.send_stop(self.id_b8)
                      
                    #判断是否允许小车进入场景       
                if self.id_1 !=0 or self.get_id(Point_a) != 0:
                    self.en_permission = 0
                else:
                    self.en_permission = 1
                self.pub_en_permission.publish(data = self.en_permission)
                    
            #-----------------------------------------------------
            else:
                print("waiting")
                


if __name__ == '__main__':
    try:
        detector = compute_node()
        detector.compute()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
