import cv2
import numpy as np
import math
# -*- coding: utf-8 -*-
"""
Created on Wed Oct 13 13:10:31 2021

@author: wang_returns
"""

l = 40#20cm = 45.641 像素
#小车运动

def imgwd(markerCenter):
    x_img, y_img = markerCenter
    return([x_img,y_img])
#计算位姿
def location(i,corners):
    #将四个角点坐标求平均得到中心点坐标
    markerCenter = corners[i][0].sum(0)/4.0
    #计算二维码中心点在建立的坐标系中的坐标
    pose=imgwd(markerCenter)
    #计算两对边在x轴的投影长度和
    delta_x = corners[i][0][0][0]+corners[i][0][1][0]-corners[i][0][2][0]-corners[i][0][3][0]
    #计算相同边在y轴的投影长度和
    delta_y = corners[i][0][0][1]+corners[i][0][1][1]-corners[i][0][2][1]-corners[i][0][3][1]
    #计算二维码朝向角度（反三角函数）
    theta = math.atan2(delta_y, delta_x)
    return markerCenter,pose,theta,delta_x,delta_y


min_con = 40 #判断点上有车的最小距离

#计算距离
def distance(x1,y1,x2,y2):
    dis = abs(np.sqrt((x2-x1) ** 2 + (y2-y1) ** 2))
    return int(dis)

#判断点上是否有车
def if_filled (pose, point):

    x_p = point[0]
    y_p = point[1]
    k = 0
    for i in range(len(pose)):
        #print(pose[i])
        if distance(pose[i][0],pose[i][1],x_p,y_p) < 40:   # 30
            k =1
    if k == 0:
        return False
    else:
        return True

#判断点上是否有车
def if_filled_specific(pose, point):
    try:
        if distance(pose[0],pose[1],point[0],point[1]) < min_con:
            return True
        else:
            return False
    except:
        print("cam lose ids")



#小车运动
def move(pose,theta,v,point_goal,l,Lf):
    X = pose[0]
    Y = pose[1]
    cx = point_goal[0]
    cy = point_goal[1]
    delta,ind = pure_pursuit_control(X, Y, theta, v, cx, cy,l,Lf)
    if delta > np.pi/4:
        delta = np.pi/4
    elif delta < -np.pi/4:
        delta = -np.pi/4
    input_data = v*100 + 50 - delta*180/np.pi
    input_data = int(input_data)
    return input_data,ind
#横向控制算法
def pure_pursuit_control(X, Y, theta, v, cx, cy,l,Lf):
    ind = calc_target_index(X, Y, cx, cy,Lf)

    if ind < len(cx):
        tx = cx[ind]
        ty = cy[ind]
    else:
        tx = cx[-1]
        ty = cy[-1]
        ind = len(cx) - 1
    alpha = math.atan2(ty - Y, tx - X) - theta
    delta = 1.1*math.atan2(2.0 * l * np.sin(alpha) / distance(X,Y,tx,ty), 1.0)

    if v > 5:  # back
        alpha = np.pi - alpha
        delta = 2.8*math.atan2(2.0 * l * np.sin(alpha) / distance(X,Y,tx,ty), 1.0)

    return delta, ind

#定义函数用于搜索最临近的路点：
def calc_target_index(X, Y, cx, cy,Lf):
    # 搜索最临近的路点
    dx = [X - icx for icx in cx]
    dy = [Y - icy for icy in cy]
    d = [abs(np.sqrt(idx ** 2 + idy ** 2)) for (idx, idy) in zip(dx, dy)]
    ind = d.index(min(d))
    L = 0.0

    while Lf > L and (ind + 1) < len(cx):
        dx = cx[ind + 1] - cx[ind]
        dy = cy[ind + 1] - cy[ind]
        L += np.sqrt(dx ** 2 + dy ** 2)
        ind += 1

    return ind
