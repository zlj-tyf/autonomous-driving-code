#!/usr/bin/env python3
# -*- coding: UTF-8 -*-

#车流量不能太大，最好保证ap阶段，没有车在en-a，否则可能不稳定

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
import way_2
import way_3
import way_4 
font = cv2.FONT_HERSHEY_SIMPLEX #font for displaying text (below)
from digi.xbee.devices import XBeeDevice
import  digi
from digi.xbee.exception import TransmitException
import random
import message_filters

l = 45.641
exit_enter_theta1 = np.float32([])
exit_enter_theta2 = np.float32([3.14,-1.58])
exit_enter_theta3 = np.float32([-1.6,0])
exit_enter_theta4 = np.float32([])


class compute_node:
    def __init__(self, cam_num = 1):
        self.way_function = {'1':way_1.way(), '2':way_2.way(), '3': way_3.way(), '4':way_4.way()}
        self.changeable_param = {'1': [-1.6, 0], '3':[-1.6, 0]}#第一个参数是theta_ex,第二个为theta_en, 通过朝向角区分进id_en,id_ex
        
        self.cam_num = cam_num
        self.node_name = 'compute_node_%d'% cam_num
        self.cam_subsucribe = '/cam_%d'%cam_num
        self.enPermission_subscribe = '/enPermission_cam%d'%(cam_num%4+1)
        self.enPermission_publish = '/enPermission_cam%d'%cam_num
        
        rospy.init_node(self.node_name, anonymous=True)
        self.pub_move = rospy.Publisher('/move', Float32MultiArray, queue_size=1)
        self.pub_en_permission = rospy.Publisher(self.enPermission_publish, Int16, queue_size=1 )#用于判断IV区的车是否能进入I区
        
        
        self.image_flag = False
        self.flag_out = 0
        
        self.vertical_permission = -1
        self.ex_permission = 1#1-可出3区 0-不可出3区
        
        #记录5帧，若有4帧数为0，则认为flag_out = 0
        self.out_count_all = 0#总帧数
        self.out_count_now = 0 #计数帧，如果下个场景没车加一
        
        self.id_en = 0
        self.id_ex = 0
        self.id_intersec_ctrl = 0
        
        
    def get_id(self, point, theta = -100, radius = 40):
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
            if dis_all[index] < radius:
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
            rospy.loginfo("send false")
            
    def send_judge_arrive(self, pose_goal, id_name, v, path, l, Lf):
        array = []
        try:
            if if_filled_specific(self.pose_now[self.ids.index(id_name)], pose_goal) :
                array.append(999)
                array.append(float(id_name))
                self.pub_move.publish(Float32MultiArray(data = array))
                # print("arrive", pose_goal)
                rospy.loginfo("arrive %s", pose_goal)
                #self.data_update()          #当前位姿计算完后更新数据，用于while判断和处理
                return True
            else :
                self.send(id_name = id_name, v = v, path = path, l = l, Lf = Lf)
                #self.data_update()          #当前位姿计算完后更新数据，用于while判断和处理
                return False
        except:
            rospy.loginfo("sja failed")

        
    def listener(self):
        while not rospy.is_shutdown():       
            t_cam = message_filters.Subscriber(self.cam_subsucribe, Float32MultiArray)
            t_vertical = message_filters.Subscriber("/vertical_permission", Int16)
            t_en = message_filters.Subscriber(self.enPermission_subscribe, Int16)
            ts = message_filters.ApproximateTimeSynchronizer([t_cam, t_vertical, t_en], 10, 1, allow_headerless=True)#融合同步三个话题
            ts.registerCallback(self.compute)
            rospy.spin()
            
            
    def compute(self, data1, data2, data3):
        #处理订阅的话题
        self.ex_permission = data3.data              
        if self.id_intersec_ctrl == 0:#距离十字路口较远时，允许小车运行 不鲁邦
            self.vertical_permission = 1
        else:#小车靠近路口时， 判断红绿灯
            self.vertical_permission = data2.data
            if data2.data == 1:
                self.id_intersec_ctrl = 0        
        #传入cam2小车位姿       
        self.pose_now = []
        self.theta = []
        self.ids = []
        if len(data1.data) != 0 :
            for i in range(len(data1.data)//4):       #对每个二维码信息读取, 并且二维码的数据都已经根据id的大小排序 //:整除，舍去小数
                self.ids.append(int(data1.data[i*4]))
                self.pose_now.append([data1.data[i*4+1], data1.data[i*4+2]])  #self.pose_now[i][3] 为id , 前面为xy
                self.theta.append(data1.data[i*4+3])
                self.image_flag = True
        else :
            self.image_flag = False
            
        self.management()
        
    def management(self):
        rospy.loginfo("compute node class")

        
        
# class parking_lot_compute():
    

class road_compute(compute_node):
    def __init__(self, cam_num = 1):
        super(road_compute,self).__init__(cam_num)
        self.get_ways()
    
    def get_ways(self):
        cam_num_tmp = str(self.cam_num)
        self.p1a, self.ab,self.b8 = self.way_function[cam_num_tmp]
        self.x_all = np.concatenate((self.p1a[0],self.ab[0],self.b8[0]),axis=0) 
        self.y_all = np.concatenate((self.p1a[1],self.ab[1],self.b8[1]), axis=0)
        way = eval('way_{}'.format(self.cam_num))#动态获取way类
        # self.important_points = {"a":way_1.Point_a, "b":way_1.Point_b, "ex":way_1.Point_c, "en":way_1.Point_d, "p1":way_1.Point_p1, "p2": way_1.Point_p2} 
        self.important_points = {"a":way.Point_a, "b":way.Point_b, "ex":way.Point_8, "en":way.Point_1} 

    
if __name__ == '__main__':
    manager = road_compute(2)
    manager.listener()