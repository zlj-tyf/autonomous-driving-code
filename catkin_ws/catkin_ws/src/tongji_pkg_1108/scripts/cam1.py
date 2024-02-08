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
import initi_camera_class as cam

if __name__ == '__main__':
    try:
        detector_1 = cam.parking_lot_cam(1)
        detector_1.perspective_shift_param = cam.perspective_1
        detector_1.thres_param = [11,2]
        
        # detector_3 = road_cam(2)
        # detector_3.perspective_shift_param = np.float32([[182,450],[162,1],[549,450],[566,5]])
        # detector_3.thres_param = [15,5]
        
        while True:
            # detector_1.image_hanle()
            detector_1.image_hanle()
            
    except rospy.ROSInterruptException:
        pass
