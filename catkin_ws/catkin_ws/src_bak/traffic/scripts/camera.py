#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import cv2
import rospy
import numpy as np
import cv2.aruco as aruco
from std_srvs.srv import SetBool
from std_msgs.msg import Float32MultiArray
from tools import calc_pose, preprocess


class Camera:
    def __init__(self):
        rospy.init_node('camera', anonymous=True)
        # get ros parameters
        node_name = rospy.get_name()
        self.caps = {}
        for i in range(4):
            cam = rospy.get_param(node_name + '/cam' + str(i + 1))
            self.caps[i] = cv2.VideoCapture(cam)  # open camera
            self.caps[i].set(cv2.CAP_PROP_FPS, 30)  # set frame per second
        self.img_sz = rospy.get_param(node_name + '/img_sz')
        self.view_path = rospy.get_param(node_name + '/view_path')
        recalibrate = rospy.get_param(node_name + '/recalibrate')
        rechoose = rospy.get_param(node_name + '/rechoose')
        self.ids = list(map(int, eval(rospy.get_param(node_name + '/ids'))))

        # initialize ArUco
        self.aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
        self.aruco_params = aruco.DetectorParameters_create()

        self.cars = {}
        self.pose_pub = rospy.Publisher('/pose', Float32MultiArray, queue_size=1)

        # preprocess: perspective and rotation transform, calculate map contours and paths
        while True:
            images = []
            for cam in self.caps.keys():
                cap = self.caps[cam]
                ret, image = cap.read()
                if ret:
                    images.append(image)
            if len(images) == 4:
                self.matrices, self.paths = preprocess(images, self.img_sz, recalibrate, rechoose)
                break
        # ask node 'controller' to access map
        rospy.wait_for_service('get_map')
        get_map = rospy.ServiceProxy('get_map', SetBool)
        while not get_map(True):
            pass

    def run(self):
        while not rospy.is_shutdown():
            frames = []
            for cam in self.caps.keys():
                ret, frame = self.caps[cam].read()
                if ret:
                    frames.append(frame)
            if len(frames) == 4:
                for i in range(4):
                    frames[i] = cv2.warpPerspective(
                        frames[i], self.matrices['cam' + str(i)], (self.img_sz, self.img_sz)
                    )
                top = np.hstack([frames[2], frames[1]])
                bottom = np.hstack([frames[3], frames[0]])
                frame = np.vstack([top, bottom])
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                corners, ids, _ = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)

                if self.view_path:  # draw paths
                    for key in self.paths.keys():
                        path = self.paths[key]
                        if key[-1] == '0':
                            color = (0, 0, 255)
                        elif key[-1] == '1':
                            color = (0, 255, 255)
                        elif key[-1] == '2':
                            color = (0, 255, 0)
                        else:
                            color = (255, 255, 0)
                        for i in range(path.shape[0] - 1):
                            cv2.line(
                                frame, (int(path[i, 0]), int(path[i, 1])),
                                (int(path[i + 1, 0]), int(path[i + 1, 1])), color, 2, cv2.LINE_AA
                            )

                if ids is not None:  # draw direction arrows and ids
                    for i in range(len(ids)):
                        if int(ids[i]) in self.ids:
                            x, y, theta, sz = calc_pose(corners[i].squeeze())
                            cv2.arrowedLine(
                                frame, (int(x), int(y)),
                                (int(x + sz * np.cos(theta)), int(y + sz * np.sin(theta))), (0, 0, 255), 5, 8, 0, 0.25
                            )
                            cv2.putText(
                                frame, str(ids[i]), (int(x), int(y)), cv2.FONT_HERSHEY_SIMPLEX, 0.75,
                                (255, 255, 0), 2
                            )
                            self.cars[int(ids[i])] = [x, y, theta]
                else:
                    cv2.putText(
                        frame, 'no car detected', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 0, 255), 2
                    )

                if self.cars:  # publish pose topic
                    data = []
                    for key in list(self.cars.keys()):
                        x, y, theta = self.cars[key]
                        data += [key, x, y, theta]
                    self.pose_pub.publish(Float32MultiArray(data=data))

                cv2.imshow('camera', frame)
                cv2.waitKey(1)
        for cam in self.caps.keys():
            self.caps[cam].release()


if __name__ == '__main__':
    try:
        Camera().run()
    except rospy.ROSInterruptException:
        pass
