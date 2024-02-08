#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import Float32MultiArray
from std_srvs.srv import SetBool, SetBoolResponse
from tools import load_paths_from_json, init_pos_check, StateMachine


class Controller:
    def __init__(self):
        rospy.init_node('controller', anonymous=True)
        rospy.Subscriber('/pose', Float32MultiArray, self.callback, queue_size=1)

        node_name = rospy.get_name()
        self.img_sz = rospy.get_param(node_name + '/img_sz')
        self.ids = eval(rospy.get_param(node_name + '/ids'))

        self.paths = None
        rospy.Service('get_map', SetBool, self.handler)

        self.state_machines = {}
        self.cmd_pub = rospy.Publisher('/command', Float32MultiArray, queue_size=1)

        rospy.spin()

    def callback(self, msg):
        if self.paths is not None and msg.data:
            cars = {}
            for i in range(len(msg.data) // 4):
                cars[int(msg.data[i * 4])] = msg.data[i * 4 + 1: i * 4 + 4]

            if len(self.state_machines) != len(self.ids):
                if not init_pos_check(self.paths, cars, self.img_sz / 500):  # check initial positions
                    rospy.loginfo('Please reset the cars')
                else:
                    for key in cars.keys():  # initialize state machine for each car
                        self.state_machines[key] = StateMachine(cars[key], self.paths, self.img_sz)
                return

            data = []
            for key in self.state_machines.keys():
                # get states of other cars
                states = {key: self.state_machines[key].state for key in self.state_machines.keys()}
                states.pop(key)
                # update state and get control command
                data.append(int(key))
                data += self.state_machines[key].update(cars[key], states.values())
            self.cmd_pub.publish(Float32MultiArray(data=data))

    def handler(self, req):
        if req.data:
            try:
                self.paths = load_paths_from_json()
            except FileNotFoundError:
                pass

            if self.paths is not None:
                return SetBoolResponse(success=True)
            else:
                return SetBoolResponse(success=False)
        else:
            return SetBoolResponse(success=False)


if __name__ == '__main__':
    try:
        Controller()
    except rospy.ROSInterruptException:
        pass
