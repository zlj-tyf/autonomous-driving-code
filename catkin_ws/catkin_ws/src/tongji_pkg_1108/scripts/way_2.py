import numpy as np
import cv2


Point_1 = (55, 450)  #p1口 
Point_a = (55, 260) #a,入口         //单独内圈
Point_2 = (60, 168)  #p2口
Point_3 = (170,110) #p3口
Point_4 = (320, 110) #p4口
Point_5 = (420, 206)  #p5口
Point_6 = (420, 350)  #p6口
Point_7 = (325, 450)  #p7口
Point_8 = (60, 450)  #p8口           //21出口公共点   
Point_b = (250,450) #b,出口         //单独内圈


way_aba = [Point_a,Point_2,Point_3,Point_4,Point_5,Point_6,Point_7,Point_b,Point_a]


def way():
    
    #1点到a点
    p1ay = np.arange(Point_1[1],Point_a[1],-1)
    p1ax = np.zeros_like(p1ay)
    for i in range(len(p1ax)):
       p1ax[i] = (Point_a[0]-Point_1[0])*i/len(p1ay) + Point_1[0]
    p1a = (p1ax,p1ay)
    
    #a点到2点
    a2y = np.arange(Point_a[1],Point_2[1],-1)
    a2x = np.zeros_like(a2y)
    for i in range(len(a2y)):
       a2x[i] = (Point_2[0]-Point_a[0])*i/len(a2x) + Point_a[0]
    a2 = (a2x,a2y)


    #2点到3点
    p23x = np.arange(90)
    p23y = np.zeros_like(p23x)
    for i in range(90):
        p23y[i] = Point_2[1] + (Point_3[1]-Point_2[1])*np.sin(i*np.pi/180)
        p23x[i] = Point_3[0] - (Point_3[0]-Point_2[0])*np.cos(i*np.pi/180)
    p23 = (p23x,p23y)
    #print(a2)

    #3点到4点
    p34x = np.arange(Point_3[0],Point_4[0])
    p34y = np.zeros_like(p34x)
    for i in range(len(p34y)):
       p34y[i] = (Point_4[1]-Point_3[1])*i/len(p34y) + Point_3[1]
    p34 = (p34x,p34y)
    #print(a2)
    
    #4点到5点  Point_4 = (254, 207) #p4口   Point_5 = (312, 149) #p5口

    p45x = np.arange(90)
    p45y = np.zeros_like(p45x)
    for i in range(90):
        p45y[i] = Point_5[1] - (Point_5[1]-Point_4[1])*np.cos(i*np.pi/180)
        p45x[i] = Point_4[0] + (Point_5[0]-Point_4[0])*np.sin(i*np.pi/180)
    p45 = (p45x,p45y)
   # print(p45)
    
    #5点到6点  Point_5 = (336,152) #p5口  Point_6 = (442,152) #p6口
    p56y = np.arange(Point_5[1],Point_6[1])
    p56x = np.zeros_like(p56y)
    for i in range(len(p56y)):
        p56x[i] = (Point_6[0]-Point_5[0])*i/len(p56x) + Point_5[0]
    p56 = (p56x,p56y)
    #print(p56)

    #6点到7点  
    p67x = np.arange(90)
    p67y = np.zeros_like(p67x)
    for i in range(90):
        p67x[i] = Point_7[0] - (Point_7[0]-Point_6[0])*np.cos(i*np.pi/180)
        p67y[i] = Point_6[1] + (Point_7[1]-Point_6[1])*np.sin(i*np.pi/180)
    p67 = (p67x,p67y)
    #print(p56)

    #7点到b点   Point_7 = (512,227) #p7口  Point_b = (512,325) #b,入口         //单独内圈
    p7bx = np.arange(Point_7[0],Point_b[0],-1)
    p7by = np.zeros_like(p7bx)
    for i in range(len(p7by)):
        p7by[i] = (Point_b[1]-Point_7[1])*i/len(p7by) + Point_7[1]
    p7b = (p7bx,p7by)
    #print(p7b)

    #b点到8点
    b8x = np.arange(Point_b[0],Point_8[0],-1)
    b8y = np.zeros_like(b8x)
    for i in range(len(b8x)):
       b8y[i] = (Point_8[1]-Point_b[1])*i/len(b8y) + Point_b[1]
    b8 = (b8x,b8y)


    abx= np.concatenate((a2[0],p23[0],p34[0],p45[0],p56[0],p67[0],p7b[0]),axis=0)
    aby= np.concatenate((a2[1],p23[1],p34[1],p45[1],p56[1],p67[1],p7b[1]),axis=0)
    ab = (abx,aby)
    return p1a, ab,b8


# img = np.zeros((640, 640, 3))
# p1a, ab, b8= way()

# # X= np.concatenate((p1a[0],ab[0],b8[0]),axis=0)
# # Y= np.concatenate((p1a[1],ab[1],b8[1]),axis=0)

# # print(X,Y)
# X= ab[0]
# Y= ab[1]
# while 1:
#     for i in range(len(X)):
#         cv2.circle(img, (X[i], Y[i]), 1, (0, 0, 255), 1)
#         cv2.imshow('img', img)
#         cv2.waitKey(1)

