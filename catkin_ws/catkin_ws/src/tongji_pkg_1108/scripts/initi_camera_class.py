#!/usr/bin/env python3
# -*- coding: UTF-8 -*-

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


#四个相机透视变换参数
perspective_1 = np.float32([[126,435],[121,59],[502,437],[505,55]])
perspective_2 = np.float32([[565,432],[190,410],[585,32],[187,29]])
perspective_3 = np.float32([[533,30],[524,400],[138,31],[146,405]])
perspective_4 = np.float32([[130,63],[504,58],[122,446],[517,440]])
#四个相机自适应二值化参数
thres_1 = [11,3]
thres_2 = [13,2]
thres_3 = [13,2]
thres_4 = [13,3]

#相机号
cam_id = [0, 4, 2, 6]



class cam:
    def __init__(self, cam_num = 1):#type有两种：parking_lot 和 road
        self.way_function = {'1':way_1.way(), '2':way_2.way(), '3': way_3.way(), '4':way_4.way()}
        self.x_all = []
        self.y_all = []
        
        self.cam_num = cam_num
        #node_name:开启节点的名称 topic_name:发布话题的名称
        self.node_name = 'initi_node_%d'% cam_num
        self.topic_name = '/cam_%d' % (cam_num)#输出小车位置的topic名称
        self.topic_img_name = '/img_%d'%(cam_num)#输出一个视频帧的话题名称
        
        
        self.thres_param = [11, 2]#自适应二值化函数的参数
        self.perspective_shift_param = np.float32([[[0,0],[0,0],[0,0],[0,0]]])#透视变换参数
        self.important_points = {}
        
        rospy.init_node(self.node_name, anonymous=True)
        self.pub_pos = rospy.Publisher(self.topic_name, Float32MultiArray, queue_size=1)
        self.pub_image = rospy.Publisher(self.topic_img_name, Image, queue_size= 1)
        self.image_initi()
        
        self.ids_1 = [0]
        self.ids_2 = [0]
        
        
    def image_initi(self):
        cam_id_tmp = cam_id[-1+self.cam_num]
        self.cap = cv2.VideoCapture(cam_id_tmp, cv2.WINDOW_NORMAL)  ##4为1号； 6为4号； 8为3号; 10为2号
        self.cap.set(5, 30)                              # 设置帧率60
        # cv2.namedWindow('cam%d'%self.cam_num, cv2.WINDOW_NORMAL)

    def pic_prehandle(self,mask, thres_param):#mask--输入的图像 thres_param:长度为2的数组，adaptiveThreshold待修改参数
        mask = cv2.cvtColor(mask, cv2.COLOR_BGR2GRAY)
        mask = cv2.adaptiveThreshold(mask,255,cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY,thres_param[0], thres_param[1])
        return mask
    
    def get_id(self, pose_now,ids,point):
        dis_all = []
        x = point[0]
        y = point[1]
        for i in range(len(pose_now)):
            dis_all.append(distance(pose_now[i][0],pose_now[i][1],x,y))
        index = np.argmin(dis_all)
        if dis_all[index] < 20:
            id = ids[np.argmin(dis_all)]
        else:
            id = 0
        return id
    
    

    def image_hanle(self):
        array_backup = []
        point_backup = [0,0]
        pts1 = self.perspective_shift_param
        pts2 = np.float32([[0,0],[500,0],[0,500],[500,500]])
        bridge = CvBridge()#发布图像用
        while(self.cap.isOpened()):         #相机1的图像坐标系的y对应于世界坐标系的x，x对应于世界坐标系的y
            array = []
            pose_now = []
            point_ids=[]
            id_index = []
            ids_backup = []
            array_end = []
            self.image_flag = False
            
            ret, frame_pre = self.cap.read()

            M = cv2.getPerspectiveTransform(pts1,pts2)
            dst = cv2.warpPerspective(frame_pre,M,(500,500))

            frame = self.pic_prehandle(dst, self.thres_param)
            aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)       #设置预定义的字典
            parameters =  aruco.DetectorParameters_create()             #使用默认值初始化检测器参数
            corners, ids, rejectedImgPoints = aruco.detectMarkers(frame, aruco_dict, parameters=parameters)

            if not ids is None:
                for i in range(len(ids)):
                    if ids[i] < 10:
                        self.image_flag = True
                        markerCenter, pose, theta0, delta_x, delta_y=location(i, corners)
                        #画出二维码位置与朝向
                        cv2.arrowedLine(dst, (int(markerCenter[0]), int(markerCenter[1])), (int(markerCenter[0]+delta_x/2), int(markerCenter[1]+delta_y/2)), (255, 0, 255), 5, 8, 0, 0.2)
                        cv2.arrowedLine(dst, (int(markerCenter[0]), int(markerCenter[1])), (int(markerCenter[0]+delta_x/2), int(markerCenter[1]+delta_y/2)), (255, 0, 255), 5, 8, 0, 0.2)
                        #屏幕显示id
                        cv2.putText(dst, str(ids[i]), (int(markerCenter[0]) +10 , int(markerCenter[1])+10), font, 1, (0, 255, 0), 2, cv2.LINE_AA)
                        ids_backup.append(ids[i])

                        array.append(ids[i])
                        array.append(markerCenter[0])
                        array.append(markerCenter[1])
                        array.append(theta0)
                        pose_now.append([markerCenter[0],markerCenter[1]])
                    else:
                        print("ghost")
            else:
            #屏幕显示没有id
                cv2.putText(dst, "No Ids", (0, 64), font, 1, (0, 255, 0), 2, cv2.LINE_AA)

            for i in range(len(self.x_all)):
                cv2.circle(dst, (self.x_all[i], self.y_all[i]), 1, (0, 0, 255), 1)
             
            #  #在图上标记重要点
            for key,value in self.important_points.items():
                cv2.putText(dst,'{key}'.format(key = key),self.important_points[key], font, 1, (0, 0, 255), 2, cv2.LINE_AA)
            
            # cv2.imshow('cam%d'%self.cam_num, frame_pre)
            # cv2.waitKey(1)

            self.dst_img = dst
            #发布图像
            msg = bridge.cv2_to_imgmsg(dst, encoding="bgr8")
            self.pub_image.publish(msg)
            # rospy.loginfo(msg)


            if ids is not None and self.image_flag:
                if if_filled(pose_now,way_2.Point_b):
                    id_out2 = self.get_id(pose_now,ids,way_2.Point_b)
                else:
                    id_out2 = 0
                point_ids.append(id_out2)       #相机2中出停车点
                point_end = point_ids
                id_sort = ids.copy()
                if all(i < 10 for i in ids):
                    id_sort.sort()  #对ID排序                         #是根据ID的大小对数据进行排序
                    for i in range(len(ids)):
                        id_index.append(ids_backup.index(id_sort[i]))  #对排序后的id在ids找出索引值
                    for i in range(len(ids)):
                        for j in range(4):
                            array_end.append(array[id_index[i]*4+j])

                array_backup = array_end
                array_backup = array
                point_backup = point_ids
            else:
                array_end = array_backup
                point_end = point_backup
            point_ids_pub = Float32MultiArray(data = point_end)


            print(array_end)
            corners_ids = Float32MultiArray(data = array_end)           #6个元素为一个图片，
            self.pub_pos.publish(corners_ids)

            # key_code = cv2.waitKey(1)
            # if key_code & 0xFF == ord('q'):
            #     # 按按键‘q’退出程序
            #     print('Quit')
            #     break
        cv2.destroyAllWindows()
        self.cap.release()
        sys.exit(0)

class parking_lot_cam(cam):#对应13摄像头 继承way对象
    def __init__(self, cam_num=1):
        super(parking_lot_cam,self).__init__(cam_num)
        self.get_ways()
    
    def get_ways(self):#从way_x中取出路径坐标
        cam_num_tmp = str(self.cam_num)
        self.ap1_1, self.ap1_2,self.ap2_1, self.ap2_2, self.p1b, self.p2b, self.bc, self.da = self.way_function[cam_num_tmp]
        self.x_all = np.concatenate((self.ap1_1[0],self.ap1_2[0],self.ap2_1[0],self.ap2_2[0],self.p1b[0],self.p2b[0],self.bc[0],self.da[0]),axis=0) 
        self.y_all = np.concatenate((self.ap1_1[1],self.ap1_2[1],self.ap2_1[1],self.ap2_2[1],self.p1b[1],self.p2b[1],self.bc[1],self.da[1]), axis=0)
        way = eval('way_{}'.format(self.cam_num))#动态获取way类
        # self.important_points = {"a":way_1.Point_a, "b":way_1.Point_b, "ex":way_1.Point_c, "en":way_1.Point_d, "p1":way_1.Point_p1, "p2": way_1.Point_p2} 
        self.important_points = {"a":way.Point_a, "b":way.Point_b, "ex":way.Point_c, "en":way.Point_d, "p1":way.Point_p1, "p2": way.Point_p2} 

class road_cam(cam):#对应24摄像头，继承way对象
    def __init__(self, cam_num=1):
        super(road_cam,self).__init__(cam_num)
        self.get_ways()
    
    def get_ways(self):#从way_x中取出路径坐标
        cam_num_tmp = str(self.cam_num)
        self.p1a, self.ab,self.b8 = self.way_function[cam_num_tmp]
        self.x_all = np.concatenate((self.p1a[0],self.ab[0],self.b8[0]),axis=0) 
        self.y_all = np.concatenate((self.p1a[1],self.ab[1],self.b8[1]), axis=0)
        way = eval('way_{}'.format(self.cam_num))#动态获取way类
        # self.important_points = {"a":way_1.Point_a, "b":way_1.Point_b, "ex":way_1.Point_c, "en":way_1.Point_d, "p1":way_1.Point_p1, "p2": way_1.Point_p2} 
        self.important_points = {"a":way.Point_a, "b":way.Point_b, "ex":way.Point_8, "en":way.Point_1} 




# if __name__ == '__main__':
#     try:
#         # detector_1 = parking_lot_cam(1)
#         # detector_1.perspective_shift_param = np.float32([[163,429],[159,63],[539,444],[565,65]])
#         # detector_1.thres_param = [11,2]
        
#         detector_3 = road_cam(2)
#         detector_3.perspective_shift_param = np.float32([[182,450],[162,1],[549,450],[566,5]])
#         detector_3.thres_param = [15,5]
        
#         while True:
#             # detector_1.image_hanle()
#             detector_3.image_hanle()
            
#     except rospy.ROSInterruptException:
#         pass

    


# class road_way:
