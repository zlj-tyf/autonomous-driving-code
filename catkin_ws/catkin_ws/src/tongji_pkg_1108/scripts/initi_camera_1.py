#!/usr/bin/env python3
# -*- coding: UTF-8 -*-

import rospy
from std_msgs.msg import Float32MultiArray
import numpy as np
import cv2
import sys
import cv2.aruco as aruco
from functions import *
font = cv2.FONT_HERSHEY_SIMPLEX #font for displaying text (below)
import way_1

ap1_1,ap1_2,ap2_1,ap2_2,p1b,p2b,bc,da = way_1.way()
x_all = np.concatenate((ap1_1[0],ap1_2[0],ap2_1[0],ap2_2[0],p1b[0],p2b[0],bc[0],da[0]), axis=0)
y_all = np.concatenate((ap1_1[1],ap1_2[1],ap2_1[1],ap2_2[1],p1b[1],p2b[1],bc[1],da[1]), axis=0)
# x_all = ap2_1[0]
# y_all = ap2_1[1]
class init_car_camera:
    def __init__(self):
        rospy.init_node('initi_node', anonymous=True)
        # self.pub_image = rospy.Publisher('/image_Handle_2', Float32MultiArray, queue_size=1)
        self.pub_image = rospy.Publisher('/cam_1', Float32MultiArray, queue_size=1)


        self.image_initi()
        self.ids_1 = [0]
        self.ids_2 = [0]
        self.ids = [0]

    def image_initi(self):
        self.cap = cv2.VideoCapture(0, cv2.WINDOW_NORMAL)  ##4为1号； 6为4号； 8为3号; 10为2号
        self.cap.set(3, 640)
        self.cap.set(4, 480)
        self.cap.set(5, 30)                              # 设置帧率60

        cv2.namedWindow('cam1', cv2.WINDOW_NORMAL)
        #cv2.namedWindow('warp', cv2.WINDOW_NORMAL)

    def pic_prehandle(self,mask):   #二值化
        mask = cv2.cvtColor(mask, cv2.COLOR_BGR2GRAY)
        mask = cv2.adaptiveThreshold(mask,255,cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY,11,2)
        #cv2.threshold(mask, 50, 255, cv2.THRESH_BINARY)
        
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
        pts1 = np.float32([[163,429],[159,63],[539,444],[565,65]])
        pts2 = np.float32([[0,0],[500,0],[0,500],[500,500]])
        while(self.cap.isOpened()):         #相机1的图像坐标系的y对应于世界坐标系的x，x对应于世界坐标系的y
            array = []
            pose_now = []
            self.image_flag = False
            id_index = []
            ids_backup = []
            array_end = []

            ret, frame_pre = self.cap.read()
            M = cv2.getPerspectiveTransform(pts1,pts2)
            dst = cv2.warpPerspective(frame_pre,M,(500,500))
            frame = self.pic_prehandle(dst)            
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

            for i in range(len(x_all)):
                cv2.circle(dst, (x_all[i], y_all[i]), 1, (0, 0, 255), 1)
             
             #在图上标记重要点
            important_points = {"a":way_1.Point_a, "b":way_1.Point_b, "ex":way_1.Point_c, "en":way_1.Point_d, "p1":way_1.Point_p1, "p2": way_1.Point_p2}    
            for key,value in important_points.items():
                cv2.putText(dst,'{key}'.format(key = key),important_points[key], font, 1, (0, 0, 255), 2, cv2.LINE_AA)
            
                
            cv2.imshow('cam1', dst)
            cv2.imshow('frame', frame)

            #fillter
            # ids_3 = self.ids_2
            # self.ids_2 = self.ids_1
            # if ids is None:
            #     self.ids_1 = [1,1,1,1,1,1,1,1]
            # else:
            #     self.ids_1 = ids
                
            # if len(ids_3) == len(self.ids_1) and len(ids_3) == len(self.ids_2):         #
            #     self.image_flag = True
            # elif len(self.ids_1) > len(self.ids_2) and not ids is None:
            #     self.image_flag = True
            # else :
            #     self.image_flag = False
            #in out ID

            if ids is not None and self.image_flag:
                if all(i < 10 for i in ids):
                    #摄像头1中进出口车的id值   
                    id_sort = ids.copy()
                    id_sort.sort()  #对ID排序                         #是根据ID的大小对数据进行排序
                    for i in range(len(ids)):
                        id_index.append(ids_backup.index(id_sort[i]))  #对排序后的id在ids找出索引值
                    for i in range(len(ids)):
                        for j in range(4):
                            array_end.append(array[id_index[i]*4+j])

                    array_backup = array_end
                    array_backup = array
                else:
                    print("ghost")
            else:
                array_end = array_backup


            print(array_end)
            corners_ids = Float32MultiArray(data = array_end)           #6个元素为一个图片，
            self.pub_image.publish(corners_ids)
            

            key_code = cv2.waitKey(1)
            if key_code & 0xFF == ord('q'):  # 按按键‘q’退出程序
                print('Quit')
                break
        cv2.destroyAllWindows()
        self.cap.release()
        sys.exit(0)

if __name__ == '__main__':
    try:
        detector = init_car_camera()
        while True:
            detector.image_hanle()
    except rospy.ROSInterruptException:
        pass


