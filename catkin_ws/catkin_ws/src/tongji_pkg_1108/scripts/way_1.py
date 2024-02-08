import numpy as np
import cv2
from numpy.core.numeric import zeros_like

k = 1
Point_intersc_check = (460, 300)#识别是否有车接近十字路口
Point_a = (39,275) #入口
Point_b = (360,40) #出口
Point_c = (452,450) #12区域交界点 exit
Point_d = (455,450) #41区域交界点 entrance
Point_p1 = (175,370) #车库0
Point_p2 = (270,370) #车库2
Point_p = [Point_p1,Point_p1]
way_ap1 = [Point_a,(175,110),Point_p1]
way_ap2 = [Point_a,(270,110),Point_p2]
way_p1b = [Point_p1,(176,110),Point_b]
way_p2b = [Point_p2,(176,180),(176,110),Point_b]
way_bc = [Point_b,(453,111),Point_c]
way_da = [Point_d,(85,450),Point_a]
# way_ce = [Point_c,Point_e]

def way():
    # #a点到车库p1
    ap1_1y1 = np.arange(way_ap1[0][1],way_ap1[1][1]+60,-1)
    ap1_1x1 = np.zeros_like(ap1_1y1)
    for i in range(len(ap1_1x1)):
        ap1_1x1[i] = (way_ap1[0][0] + way_ap1[1][0])/2 + (way_ap1[0][0] - way_ap1[1][0])/2*np.cos(np.pi*i/len(ap1_1y1))
    ap1_1y2 = np.arange(way_ap1[1][1]+60,way_ap1[1][1],-1)
    ap1_1x2 = np.ones_like(ap1_1y2)*way_ap1[1][0]
    ap1_1 = (np.append(ap1_1x1,ap1_1x2),np.append(ap1_1y1,ap1_1y2))
    
    ap1_2y = np.arange(way_ap1[1][1],way_ap1[2][1])
    ap1_2x = np.zeros_like(ap1_2y)
    for i in range(len(ap1_2y)):
        ap1_2x[i] = (way_ap1[2][0]-way_ap1[1][0])*i/len(ap1_2x) + way_ap1[1][0]
    ap1_2 = (ap1_2x,ap1_2y)

    #a点到车库p2
    ap2_1y1 = np.arange(way_ap2[0][1],way_ap2[1][1]+60,-1)
    ap2_1x1 = np.zeros_like(ap2_1y1)
    for i in range(len(ap2_1x1)):
        ap2_1x1[i] = (way_ap2[0][0] + way_ap2[1][0])/2 + (way_ap2[0][0] - way_ap2[1][0])/2*np.cos(np.pi*i/len(ap2_1y1))
    ap2_1y2 = np.arange(way_ap2[1][1]+60,way_ap2[1][1],-1)
    ap2_1x2 = np.ones_like(ap2_1y2)*way_ap2[1][0]
    ap2_1 = (np.append(ap2_1x1,ap2_1x2),np.append(ap2_1y1,ap2_1y2))
    
    ap2_2y = np.arange(way_ap2[1][1],way_ap2[2][1])
    ap2_2x = np.zeros_like(ap2_2y)
    for i in range(len(ap2_2y)):
        ap2_2x[i] = (way_ap2[2][0]-way_ap2[1][0])*i/len(ap2_2x) + way_ap2[1][0]
    ap2_2 = (ap2_2x,ap2_2y)

    #车库p1到b点
    p1by1 = np.arange(way_p1b[0][1],way_p1b[1][1],-1)
    p1bx1 = np.zeros_like(p1by1)
    for i in range(len(p1bx1)):
        p1bx1[i] = (way_p1b[1][0]-way_p1b[0][0])*i/len(p1bx1) + way_p1b[0][0]

    p1by2 = np.arange(90)
    p1bx2 = np.zeros_like(p1by2)
    for i in range(len(p1bx2)):
        p1bx2[i] = way_p1b[2][0] - (way_p1b[2][0] - way_p1b[1][0])*np.cos(i*np.pi/180)
        p1by2[i] = way_p1b[1][1] + (way_p1b[2][1] - way_p1b[1][1])*np.sin(i*np.pi/180)

    p1b = (np.append(p1bx1,p1bx2),np.append(p1by1,p1by2))

    #车库p2到b点
    p2by1 = np.arange(way_p2b[0][1],way_p2b[1][1],-1)
    p2bx1 = np.zeros_like(p2by1)
    for i in range(len(p2bx1)):
        p2bx1[i] = (way_p2b[0][0] + way_p2b[1][0])/2 + (way_p2b[0][0] - way_p2b[1][0])/2*np.cos(np.pi*i/len(p2by1))
    p2by2 = np.arange(way_p2b[1][1],way_p2b[2][1],-1)
    p2bx2 = np.ones_like(p2by2)*way_p2b[2][0]

    p2by3 = np.arange(90)
    p2bx3 = np.zeros_like(p2by3)
    for i in range(len(p2bx3)):
        p2bx3[i] = way_p2b[3][0] - (way_p2b[3][0] - way_p2b[2][0])*np.cos(i*np.pi/180)
        p2by3[i] = way_p2b[2][1] + (way_p2b[3][1] - way_p2b[2][1])*np.sin(i*np.pi/180)

    p2bx = np.concatenate((p2bx1,p2bx2,p2bx3),axis=0)
    p2by = np.concatenate((p2by1,p2by2,p2by3),axis=0)
    p2b = (p2bx,p2by)

    #bc
    bcx1 = np.arange(90)
    bcy1 = np.zeros_like(bcx1)
    for i in range(90):
        bcx1[i] = way_bc[0][0] + np.sin(i*np.pi/180)*(way_bc[1][0]-way_bc[0][0])
        bcy1[i] = way_bc[1][1] - np.cos(i*np.pi/180)*(way_bc[1][1]-way_bc[0][1])

    bcy2 = np.arange(way_bc[1][1],way_bc[2][1])
    bcx2 = np.zeros_like(bcy2)
    for i in range(len(bcy2)):
        bcx2[i] = (way_bc[2][0]-way_bc[1][0])*(i/len(bcy2)) + way_bc[1][0]
    bc = (np.append(bcx1,bcx2),np.append(bcy1,bcy2))

    dax1 = np.arange(way_da[0][0],way_da[1][0],-1)
    day1 = zeros_like(dax1)
    for i in range(len(day1)):
        day1[i] = way_da[0][1]-(way_da[0][1]-way_da[1][1])*i/len(day1)

    dax2 = np.arange(90)
    day2 = np.arange(90)
    for i in range(90):
        dax2[i] = way_da[1][0] + (way_da[2][0]-way_da[1][0])*np.sin(i*np.pi/180)
        day2[i] = way_da[2][1] - (way_da[2][1]-way_da[1][1])*np.cos(i*np.pi/180)
    dax = np.append(dax1,dax2)
    day = np.append(day1,day2)
    da = (dax,day)

    return ap1_1,ap1_2,ap2_1,ap2_2,p1b,p2b,bc,da



# img = np.zeros((640, 640, 3))
# ap0_1,ap0_2,ap1,p0b,p1b_1,p1b_2,bc,da,ce = way()
# X= np.concatenate((ap0_1[0],ap0_2[0],ap1[0],p0b[0],p1b_1[0],p1b_2[0],bc[0],da[0],ce[0]),axis=0)
# Y= np.concatenate((ap0_1[1],ap0_2[1],ap1[1],p0b[1],p1b_1[1],p1b_2[1],bc[1],da[1],ce[1]),axis=0)
# print(p2b)
# # X= bc[0]
# # Y= bc[1]
# while 1:
#     for i in range(len(X)):
#         cv2.circle(img, (X[i], Y[i]), 1, (0, 0, 255), 1)
#         cv2.imshow('img', img)
#         cv2.waitKey(1)
