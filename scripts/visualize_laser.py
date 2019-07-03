# -*- coding: utf-8 -*-

import sys
import qi
import numpy as np
import cv2
import signal
import math
import almath
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from laser_config import *

class LaserViz:
    def __init__(self, robot_ip):
        self.laser_callback = qi.PeriodicTask()
        self.laser_callback.setCallback(self.get_laser_values)
        self.laser_callback.setUsPeriod(200 * 1000)
        
        self.connect(robot_ip)

    def connect(self, robot_ip):
        try:
            self.session = qi.Session()
            port = 9559
            self.session.connect("tcp://{}:9559".format(robot_ip))
            print("connection successful!!!")
        except Exception as errorMsg:
            print("can not connect...")
            exit(-1)
        self.memory = self.session.service("ALMemory")
        self.monitor = self.session.service("MoveMonitorLoc")
        self.motion = self.session.service("ALMotion")
        self.init_config()

    def init_config(self):
        self.device_position_list = list()
        device_list = ["LaserSensor/Front", "LaserSensor/Left", "LaserSensor/Right"]
        for device in device_list:
            self.device_position_list.append(almath.Position6D(self.motion.getPosition(device, 2, 0)))
        self.robot_pose = self.motion.getRobotPosition(True)

        self.robot_moving = self.monitor.isMoving.value()
        self.move_signal_link = self.monitor.isMoving.connect(self.on_robot_moving)

       # self.laser_callback.start(True)

    # センサデータの座標変換
    def convert_laser_position(self, laser_values):
        try:
            # ワールド座標系におけるロボット位置の変換行列
            t_world2robot = almath.transformFromPose2D(almath.Pose2D(self.robot_pose))
            for sensor_device in range(1):
                for point_num in range(15):
                    laser = laser_values[sensor_device][point_num]
                    # デバイスから点群データの位置
                    t_device2point = almath.transformFromPosition3D(almath.Position3D(laser[0], laser[1], 0))
                    # ロボット座標からデバイスの位置姿勢
                    t_robot2device = almath.transformFromPosition6D(self.device_position_list[sensor_device])
                    # ワールド座標系に点群データの座標系を変換する
                    t_world2point = t_world2robot * t_robot2device * t_device2point
                    p2d_world2point = almath.pose2DFromTransform(t_world2point)
                    laser_values[sensor_device][point_num] = (p2d_world2point.x, p2d_world2point.y)
        except Exception as errorMsg:
            failure(errorMsg)
        return laser_values

    def on_robot_moving(self, is_moving):
        pass
        #self.robot_pose = self.motion.getRobotPosition(True)

    def convert_laser_position(self, sensor_device, laser_x, laser_y):
        # ワールド座標系におけるロボット位置の変換行列
        t_world2robot = almath.transformFromPose2D(almath.Pose2D(self.robot_pose))
        # デバイス→レーザー点群
        t_device2point = almath.transformFromPosition3D(almath.Position3D(laser_x, laser_y, 0))
        # ロボット→デバイス
        t_robot2device = almath.transformFromPosition6D(self.device_position_list[sensor_device])
        # ワールド座標系におけるレーザー点群
        t_world2point = t_world2robot * t_robot2device * t_device2point
        p2d_world2point = almath.pose2DFromTransform(t_world2point)
        return (p2d_world2point.x, p2d_world2point.y)

    def get_laser_values(self):
        lasersensor_path = "Device/SubDeviceList/Platform/LaserSensor/"
        device_list = ["Front/", "Left/", "Right/"]
        laser_values = [[(0,0)]*15, [(0,0)]*15, [(0,0)]*15]
        self.robot_pose = self.motion.getRobotPosition(True)
        self.robot_pose[0] = self.robot_pose[1] = 0.0

        position = [[(0,0)]*15, [(0,0)]*15, [(0,0)]*15]
        for j in range(0,3):
            device = device_list[j]
            for i in range(0, 15):
                segNo = 15 - i
                path = lasersensor_path + device
                x = self.memory.getData(path + "Horizontal/Seg%02d/X/Sensor/Value" % segNo)
                y = self.memory.getData(path + "Horizontal/Seg%02d/Y/Sensor/Value" % segNo)
                position[j][i] = self.convert_laser_position(j, x, y)
        return position

    def draw_robot(self, ax):
        robot_pose = almath.Pose2D(self.robot_pose)
        xn = robot_pose.x + 0.2 * math.cos(robot_pose.theta)
        yn = robot_pose.y + 0.2 * math.sin(robot_pose.theta)
        plt.plot([robot_pose.x, xn], [robot_pose.y, yn], c="black")
        c = patches.Circle(xy=(robot_pose.x, robot_pose.y), radius=0.2, fill=False, color="black")
        ax.add_patch(c)

    #  レーザーを描画するだけのスレッド
    def run(self):
        color = ["red", "blue", "green"]
        ax = plt.axes()

        try:
            while True:
                ax.cla()
                ax.set_xlim(-3,3)
                ax.set_ylim(-3,3)

                # draw robot
                self.draw_robot(ax)

                position = self.get_laser_values()
                for sensor_device in range(3):
                    for value in range(15):
                        laser = position[sensor_device][value]
                        try:
                            plt.scatter(laser[0], laser[1], s=20,c=color[sensor_device])
                        except Exception as errorMsg1:
                            print str(errorMsg1)
                ax.set_aspect("equal")
                ax.grid()
                plt.pause(0.02)
        except Exception as errorMsg:
            failure("running is failed: {}".format(str(errorMsg)))
            exit(-1)

def failure(e):
    exc_type, exc_obj, tb=sys.exc_info()
    lineno=tb.tb_lineno
    print(str(lineno) + ":" + e)
    exit(-1)

if __name__ == '__main__':
    ip = "127.0.0.1"
    if (len(sys.argv)) > 1:
        ip = sys.argv[1]
    laser_viz = LaserViz(ip)
    laser_viz.run()
