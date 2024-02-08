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
import way_3 as way_1
import way_2
font = cv2.FONT_HERSHEY_SIMPLEX #font for displaying text (below)
from digi.xbee.devices import XBeeDevice
import  digi
from digi.xbee.exception import TransmitException
import random
import message_filters

#路径
ap1_1,ap1_2,ap2_1,ap2_2,p1b,p2b,bc, da = way_1.way()
#轴距
l  = 45.641



class compute_node:
    def __init__(self):
        rospy.init_node('compute_node_3', anonymous=True)
        self.pub_move = rospy.Publisher('/move', Float32MultiArray, queue_size=1)
        self.pub_en_permission = rospy.Publisher('/enPermission_cam3', Int16, queue_size=1 )#用于判断IV区的车是否能进入I区
        self.image_flag = False
        self.flag_out = 0
        
        self.vertical_permission = -1
        self.ex_permission = 1#1-可出3区 0-不可出3区
        
        #记录5帧，若有4帧数为0，则认为flag_out = 0
        self.out_count_all = 0#总帧数
        self.out_count_now = 0 #计数帧，如果下个场景没车加一
        
        #小车进出口的姿态，各个摄像头场景都不同，需要调参
        self.theta_ex = -1.6
        self.theta_en = 0
        
    def callback_exPermission(self, msg):
        self.ex_permission = msg.data        

        
    def callback_intersc(self,msg):
        if self.id_intersec_ctrl == 0:#距离十字路口较远时，允许小车运行 不鲁邦
            self.vertical_permission = 1
        else:#小车靠近路口时， 判断红绿灯
            self.vertical_permission = msg.data
            if msg.data == 1:
                self.id_intersec_ctrl = 0        

        
    #传入cam2小车位姿       
    def callback_image1(self, msg):
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
                print("arrive", pose_goal)
                #self.data_update()          #当前位姿计算完后更新数据，用于while判断和处理
                return True
            else :
                self.send(id_name = id_name, v = v, path = path, l = l, Lf = Lf)
                #self.data_update()          #当前位姿计算完后更新数据，用于while判断和处理
                return False
        except:
            print("sja failed")

#主函数++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    def compute(self,data1, data2, data3):
        #根据场地调试
        vf = 4  #前进速度
        vb = 7 #后退速度
        
        l = 40  #车轴距
        Lf = 80 #前视距离
        Point_a = way_1.Point_a
        Point_b = way_1.Point_b
        Point_en = way_1.Point_d
        Point_ex = way_1.Point_c
        
        self.is_lot_occupied = 0#停车场区域是否有车在运行
        self.is_entrance_occupied = 0#en-a是否有车，包括a
        self.is_exit_occupied = 0#b-ex是否有车,包括b
        self.is_reverse = 0#倒车为1
        
        
        #待修改，可以用长度为1的栈实现
        self.id_en = 0#驶入场景的车号 en-a
        self.id_p1 = 0 #p1上的车号
        self.id_p2 = 0#p2上的车号
        self.id_ex = 0 #b-ex上的车号
        self.id_ap = 0#正在入库的车号
        self.id_pb = 0#正在出库的车号
        self.id_b = 0#停在b点的车号
        self.id_intersec_ctrl = 0

        
        self.flag_p = 0
        
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

        
        #获得车号，并比get_id有一定鲁棒性
        def get_id_robust(id, point, theta = -100):
            id_tmp = self.get_id(point,theta)
            if id_tmp !=0 and id_tmp in self.ids:
                id = id_tmp
            return id
        
        
        #en-a
        #已测试
        def move_en_to_a():
            if self.id_en != 0:
                if if_filled(self.pose_now, Point_a) == 0:
                    self.send(self.id_en,vf, da, l, Lf)
                    # print("id_en:",self.id_en,"en-a")
                    rospy.loginfo("id_en:%s, en-a", self.id_en)
                else:
                    rospy.loginfo("id_en:waiting at a")
                    self.send_stop(self.id_en)
                    self.id_ap = self.id_en
                    self.id_en = 0
                    # self.id_en = 0#待修改：id_en离开a后置0

            
        # #b-ex 带路口调度功能
        #已测试
        def move_b_to_ex():#外围函数需要判断下个场景入口是否有车
            if self.id_ex != 0:#b-ex段有车
               if self.vertical_permission == 1 and self.ex_permission == 1:#允许南北向车通行
                    if self.get_id(Point_ex, 1.5) == 0:#未到达ex
                        self.send(self.id_ex, vf, bc, l, Lf)
                        # print("id_ex:", self.id_ex, "b-ex")
                        rospy.loginfo("id_ex:%s, b-ex",self.id_ex)
                    else:#到达exit
                        # print("id_ex", self.id_ex, "arrive at exit")
                        rospy.loginfo("id_ex: %s, arrive at exit", self.id_ex)
                        self.send_stright(self.id_ex)
                        self.id_ex = 0
                        self.is_exit_occupied = 0
                        
               else:#南北方向为红灯    
                    self.send_stop(self.id_ex)
                    rospy.loginfo("red light")        
            # else:
            #     rospy.loginfo("no car at b")
            
        # # #a-p
        # #output:正在执行a-p操作return true, 在a等待return false
        # def move_a_p(id):#id需要外围语句判断，此时id_en会变成id_ap
        #     # if id not in self.ids:
        #     #     return False
        #     #判断车库有无空位
        #     if self.id_p1 * self.id_p2 == 0:#可以执行a-p操作，函数return 1    
        #         # #判断那个车库为空
        #         if self.id_p1 == 0:#进1库
        #             # print("id_ap:", id, "a-p1")
        #             #判断当前是倒车还是向前行驶
        #             if self.is_reverse == 0:#前进
        #                 # if if_filled(self.pose_now, way_1.way_ap1[1])== 1:#判断是否到达倒车起始点                        
        #                 if id in self.ids:
        #                     if if_filled_specific(self.pose_now[self.ids.index(id)], way_1.way_ap1[1])== 1:#判断是否到达倒车起始点
        #                         self.is_reverse = 1
        #                         self.send_stop(id)
        #                         rospy.loginfo("id_ap:%s,a-p1, stop to reverse",id)
        #                     else:#直行且未进入倒车其实点
        #                         rospy.loginfo("id_ap:%s,a-p1, forward",id)
        #                         self.send(id, vf, ap1_1, l, Lf)
        #             else:#倒车
        #                 # if if_filled(self.pose_now, way_1.Point_p1)==1:#到达车库 
        #                 if id in self.ids:#有时候识别不稳定，避免报错的下策                       
        #                     if if_filled_specific(self.pose_now[self.ids.index(id)], way_1.Point_p1)==1:#到达车库 待改进：到达车库时，id_p1!=0 因此程序跳不到这步！！！！
        #                         rospy.loginfo("id_ap:%s,a-p1, arrive at p1",id)
        #                         self.send_stop(id)
        #                         self.is_reverse = 0#reset
        #                         self.id_ap = 0#reset
        #                         return False
        #                     else:#倒车且没有入库，继续倒车
        #                         self.send(id, vb, ap1_2,l, Lf)
        #                         rospy.loginfo("id_ap:%s,a-p1, reversing",id)
        #         elif self.id_p2 == 0:#进2库
        #             # print("id_ap",id,"a-p2" )
        #             #判断当前是倒车还是向前行驶
        #             if self.is_reverse == 0:#前进
        #                 # if if_filled(self.pose_now, way_1.way_ap1[1])== 1:#判断是否到达倒车起始点                        
        #                 if id in self.ids:
        #                     if if_filled_specific(self.pose_now[self.ids.index(id)], way_1.way_ap2[1])== 1:#判断是否到达倒车起始点
        #                         self.is_reverse = 1
        #                         self.send_stop(id)
        #                         rospy.loginfo("id_ap:%s, a-p2, stop to reverse",id)
        #                         # print("stop to reverse")
        #                     #else:
        #                     elif if_filled_specific(self.pose_now[self.ids.index(id)], way_1.way_ap2[1])== 0:#直行且未进入倒车其实点
        #                         rospy.loginfo("id_ap:%s, a-p2, forward",id)
        #                         self.send(id, vf, ap2_1, l, Lf)
        #             else:#倒车
        #                 # if if_filled(self.pose_now, way_1.Point_p1)==1:#到达车库 
        #                 if id in self.ids:#有时候识别不稳定，避免报错的下策                       
        #                     if if_filled_specific(self.pose_now[self.ids.index(id)], way_1.Point_p2)==1:#到达车库
        #                         rospy.loginfo("id_ap:%s, a-p2, arrive at p2",id)
        #                         self.send_stop(id)
        #                         self.is_reverse = 0#reset
        #                         self.id_ap = 0#reset
        #                         return False
        #                     else:#倒车且没有入库，继续倒车
        #                         self.send(id, vb, ap2_2,l, Lf)
        #                         rospy.loginfo("id_ap:%s, a-p2, reversing",id)
        #         return True
            
        #     else:#车库没有空位，静止等待
        #         self.send_stop(id)
        #         rospy.loginfo("id_ap:%s,parking lot is full", id)
        #         return False
            
        #a-p
        #output:正在执行a-p操作return true, 在a等待return false
        def move_a_p_new(id):#id需要外围语句判断，此时id_en会变成id_ap
            def assign_destination(id):
                #判断是否还未进入ap步
                if self.flag_p == 0:#没有进入ap步
                    #给id_ap指定目的地
                    if self.id_p1 == 0:
                        self.flag_p = 1
                    elif self.id_p2 == 0:
                        self.flag_p = 2
                    else:
                        self.flag_p = 0
            # print("flag_p", self.flag_p)
            assign_destination(id)
            #判断车库有无空位
            if self.id_p1 * self.id_p2 == 0:#可以执行a-p操作，函数return 1    
                # #判断目的地是哪个
                if self.flag_p == 1:#进1库
                    # print("id_ap:", id, "a-p1")
                    # rospy.loginfo("id_ap:%s,a-p1,",id)
                    #判断当前是倒车还是向前行驶
                    if self.is_reverse == 0:#前进
                        # if if_filled(self.pose_now, way_1.way_ap1[1])== 1:#判断是否到达倒车起始点                        
                        if id in self.ids:
                            if if_filled_specific(self.pose_now[self.ids.index(id)], way_1.way_ap1[1])== 1:#判断是否到达倒车起始点
                                self.is_reverse = 1
                                self.send_stop(id)
                                rospy.loginfo("id_ap:%s,a-p1,stop to reverse",id)
                                
                            else:#直行且未进入倒车其实点
                                rospy.loginfo("id_ap:%s,a-p1,forward",id)
                                self.send(id, vf, ap1_1, l, Lf)
                    else:#倒车
                        # if if_filled(self.pose_now, way_1.Point_p1)==1:#到达车库 
                        if id in self.ids:#有时候识别不稳定，避免报错的下策                       
                            if if_filled_specific(self.pose_now[self.ids.index(id)], way_1.Point_p1)==1:#到达车库 待改进：到达车库时，id_p1!=0 因此程序跳不到这步！！！！
                                rospy.loginfo("id_ap:%s,a-p1,arrive at p1",id)
                                self.send_stop(id)
                                self.is_reverse = 0#reset
                                self.id_ap = 0#reset
                                self.flag_p = 0
                                return False
                            else:#倒车且没有入库，继续倒车
                                self.send(id, vb, ap1_2,l, Lf)
                                rospy.loginfo("id_ap:%s,a-p1,reversing",id)
                elif self.flag_p == 2:#进2库
                    #判断当前是倒车还是向前行驶
                    if self.is_reverse == 0:#前进
                        # if if_filled(self.pose_now, way_1.way_ap1[1])== 1:#判断是否到达倒车起始点                        
                        if id in self.ids:
                            if if_filled_specific(self.pose_now[self.ids.index(id)], way_1.way_ap2[1])== 1:#判断是否到达倒车起始点
                                self.is_reverse = 1
                                self.send_stop(id)
                                rospy.loginfo("id_ap:%s,a-p2,stop to reverse",id)
                            #else:
                            elif if_filled_specific(self.pose_now[self.ids.index(id)], way_1.way_ap2[1])== 0:#直行且未进入倒车其实点
                                rospy.loginfo("id_ap:%s,a-p2,forward",id)
                                self.send(id, vf, ap2_1, l, Lf)
                    else:#倒车
                        # if if_filled(self.pose_now, way_1.Point_p1)==1:#到达车库 
                        if id in self.ids:#有时候识别不稳定，避免报错的下策                       
                            if if_filled_specific(self.pose_now[self.ids.index(id)], way_1.Point_p2)==1:#到达车库
                                rospy.loginfo("id_ap:%s,a-p2,arrive at p1",id)
                                self.send_stop(id)
                                self.is_reverse = 0#reset
                                self.id_ap = 0#reset
                                self.flag_p = 0
                                return False
                            else:#倒车且没有入库，继续倒车
                                self.send(id, vb, ap2_2,l, Lf)
                                rospy.loginfo("id_ap:%s,a-p2,reversing",id)
                else:#flag_p = 0时
                    return False
                
                return True
            
                
            
            else:#车库没有空位，静止等待
                self.send_stop(id)
                rospy.loginfo("id_ap:%s, parking lot is full", id)
                return False

            
        
        
        #p-b #已调试
        #output:到达目的地时返回True，没有到达返回False
        def move_p_b(id):#id由外围函数判断那个车需要出库决定
            if id in self.ids:#摄像头识别不稳定，没有识别到小车上的二维码（不加这句可能报错）
                if id == self.id_p1:#p1-b
                    # if if_filled_specific(self.ids.index(id), way_1.Point_b) == 1:#到达b时 #必须判断if_filled_specific == 1 因为存在该函数没有识别到车号返回None的情况，而判断0是一个持续动作，非常不稳定
                    if if_filled(self.pose_now, way_1.Point_b)==1:
                        # print("id_pb:", id, ", arrive at b")
                        rospy.loginfo("id_pb:%s, arrive at b", id)
                        self.send_stop(id)
                        return True
                    else:#没有到达b时
                        # print("id_pb:", id, ", moving to b")
                        rospy.loginfo("id_pb:%s, moving to b", id)
                        self.send(id, vf, p1b, l, Lf)
                        return False

                elif id == self.id_p2:#p2-b
                    # if if_filled_specific(self.ids.index(id), way_1.Point_b) == 1:#到达b时
                    if if_filled(self.pose_now, way_1.Point_b)==1:
                        # print("id_pb:", id, ", arrive at b")
                        rospy.loginfo("id_pb:%s, arrive at b", id)
                        self.send_stop(id)
                        return True
                    else:#没有到达b时
                        # print("id_pb:", id, ", moving to b")
                        rospy.loginfo("id_pb:%s, moving to b", id)
                        self.send(id, vf, p2b, l, Lf)
                        return False
            elif id == 0:#针对id_pb=0的情况，即没有开始pb操作
                # print("no p-b operation")
                return False
            else:#摄像头识别不稳定，没有识别到小车上的二维码（不加这句可能报错）
                return False
              
            
        

        ##############主循环###################################################
        while not rospy.is_shutdown():
            if self.image_flag == True:
                #-------------------------------------------
                #获取各个检测点的车号和状态,一旦识别到车号，小车离开该点需要程序手动置为零
                self.id_ex = get_id_robust(self.id_ex,Point_b)
                self.id_en = get_id_robust(self.id_en,Point_en,self.theta_en)
                self.id_p1 = get_id_robust(self.id_p1,way_1.Point_p1)
                self.id_p2 = get_id_robust(self.id_p2,way_1.Point_p2)
                self.id_b = get_id_robust(self.id_b, Point_b)
                self.id_intersec_ctrl = get_id_robust(self.id_intersec_ctrl, way_1.Point_intersc_ctrl)

                rospy.loginfo("id_en:%s, id_ex:%s, id_p1:%s, id_p2:%s, id_ap:%s, id_pb:%s", self.id_en,self.id_ex,self.id_p1,self.id_p2,self.id_ap,self.id_pb)
                rospy.loginfo(self.ex_permission)
                # print(self.id_en, self.id_ap, self.is_lot_occupied)
                # print(self.vertical_permission)
                
                #检测各个区域的状态
                if self.id_en != 0 :
                    self.is_entrance_occupied = 1#该状态辆在车辆离开a时置0
                if self.id_ex != 0:
                    self.is_exit_occupied = 1#该状态在车辆离开exit时置0
                    
                #检测停车场的空地是否被使用
                if self.id_ap !=0 or self.id_pb !=0:
                    self.is_lot_occupied = 1
                else:
                    self.is_lot_occupied = 0
                    
                # print(self.id_ap,self.id_pb)
                #test----------------------------------------------- 
                #b-ex
                move_b_to_ex()
                
                #p-b
                if self.is_lot_occupied == 0 and self.id_ex == 0:#当停车场公共区域没有被占用。且b点没有车时
                    #执行p-b前，确定id_pb
                    if self.id_pb == 0:#id_pb == 0：没有开始pb操作
                        if self.id_p1 == 0 and self.id_p2 == 0:#车位没车
                            rospy.loginfo("no car parking")
                            self.id_pb = 0
                        if self.id_p1 * self.id_p2 !=0: #p1 2 同时有车，随机抽取一个车出库
                            i = random.randint(0,1)
                            if i == 0:
                                self.id_pb = self.id_p1
                            else:
                                self.id_pb = self.id_p2
                        else:
                            if self.id_p1 != 0:
                                self.id_pb = self.id_p1
                            elif self.id_p2 != 0:
                                self.id_pb = self.id_p2
                is_end = move_p_b(self.id_pb)
                #pb结束时reset id_pb id_p1/2
                if is_end == 1:
                    if self.id_pb == self.id_p1:
                        self.id_p1 = 0
                    elif self.id_pb == self.id_p2:
                        self.id_p2 = 0
                    # self.id_b = self.id_pb
                    self.id_pb = 0#reset id_pb，表示已经跳出pb状态
                    
                    
                    
                # print(self.is_reverse)
                # print(self.id_pb)    
                # a-p  
                if self.is_lot_occupied == 0 or (self.id_ap != 0 and self.id_pb == 0):
                    self.id_ap = get_id_robust(self.id_ap, Point_a)
                    if self.id_ap != 0:
                        is_ap = move_a_p_new(self.id_ap)
                        if is_ap == 0:#若车等在a，没有进入停车场公共区域,将id_ap置为0，以免出现车等在a，id_ap ！=0使得is_lot_occupied为1，影响出库
                            self.id_ap = 0 
                            # print("================")
                        elif is_ap == 1:
                            self.is_entrance_occupied = 0
                            self.id_en = 0
                            # print("++++++++++++++++")
                        else:#move_ap返回None的情况，为何为None未知
                            self.id_ap = 0
                            # print("-------------------")
                            
                #en-a
                move_en_to_a()
                        
                        
            is_permit = 1
            # if self.id_en != 0 or self.id_ex != 0 or self.id_ap != 0:
            if self.id_en != 0 or self.id_ap!=0: #id_ex的判断交给十字路口调度节点
                is_permit = 0#不允许2区进入
            else:
                is_permit = 1#允许2区进入
            self.pub_en_permission.publish(Int16(data = is_permit))

                    
    def listener(self): 
        while not rospy.is_shutdown():       
            t_cam = message_filters.Subscriber("/cam_3", Float32MultiArray)
            t_vertical = message_filters.Subscriber("/vertical_permission", Int16)
            t_en = message_filters.Subscriber("/enPermission_cam4", Int16)
            ts = message_filters.ApproximateTimeSynchronizer([t_cam, t_vertical, t_en], 10, 1, allow_headerless=True)#融合同步三个话题
            ts.registerCallback(self.compute)
            rospy.spin()


if __name__ == '__main__':
    try:
        detector = compute_node()
        detector.listener()

    except rospy.ROSInterruptException:
        pass
