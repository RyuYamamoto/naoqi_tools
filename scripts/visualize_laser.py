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

EXTEND_AREA = 10.0

def atan_zero_to_twopi(y, x):
    angle = math.atan2(y, x)
    if angle < 0.0:
        angle += math.pi * 2.0
    return angle

def calc_grid_map_config(ox, oy, xyreso):
    minx = round(min(ox) - EXTEND_AREA / 2.0)
    miny = round(min(oy) - EXTEND_AREA / 2.0)
    maxx = round(max(ox) + EXTEND_AREA / 2.0)
    maxy = round(max(oy) + EXTEND_AREA / 2.0)
    xw = int(round((maxx - minx) / xyreso))
    yw = int(round((maxy - miny) / xyreso))

    return minx, miny, maxx, maxy, xw, yw

class precastDB:

    def __init__(self):
        self.px = 0.0
        self.py = 0.0
        self.d = 0.0
        self.angle = 0.0
        self.ix = 0
        self.iy = 0

    def __str__(self):
        return str(self.px) + "," + str(self.py) + "," + str(self.d) + "," + str(self.angle)

class LaserViz:
    def __init__(self, robot_ip):
        self.position = [[(0,0)]*15, [(0,0)]*15, [(0,0)]*15]

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

    def precasting(self, minx, miny, xw, yw, xyreso, yawreso):

        precast = [[] for i in range(int(round((math.pi * 2.0) / yawreso)) + 1)]

        for ix in range(xw):
            for iy in range(yw):
                px = ix * xyreso + minx
                py = iy * xyreso + miny

                d = math.sqrt(px**2 + py**2)
                angle = atan_zero_to_twopi(py, px)
                angleid = int(math.floor(angle / yawreso))

                pc = precastDB()

                pc.px = px
                pc.py = py
                pc.d = d
                pc.ix = ix
                pc.iy = iy
                pc.angle = angle

                precast[angleid].append(pc)

        return precast

    def generate_ray_casting_grid_map(self, ox, oy, xyreso, yawreso):

        minx, miny, maxx, maxy, xw, yw = calc_grid_map_config(ox, oy, xyreso)

        pmap = [[0.0 for i in range(yw)] for i in range(xw)]

        precast = self.precasting(minx, miny, xw, yw, xyreso, yawreso)

        for (x, y) in zip(ox, oy):

            d = math.sqrt(x**2 + y**2)
            angle = atan_zero_to_twopi(y, x)
            angleid = int(math.floor(angle / yawreso))

            gridlist = precast[angleid]

            ix = int(round((x - minx) / xyreso))
            iy = int(round((y - miny) / xyreso))

            for grid in gridlist:
                if grid.d > d:
                    pmap[grid.ix][grid.iy] = 0.5

            pmap[ix][iy] = 1.0

        return pmap, minx, maxx, miny, maxy, xyreso


    def draw_heatmap(self, data, minx, maxx, miny, maxy, xyreso):
        x, y = np.mgrid[slice(minx - xyreso / 2.0, maxx + xyreso / 2.0, xyreso),
                        slice(miny - xyreso / 2.0, maxy + xyreso / 2.0, xyreso)]
        print(len(x), len(y))
        plt.pcolor(x, y, data, vmax=1.0, cmap=plt.cm.Blues)
        plt.axis("equal")

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

        for j in range(0,3):
            device = device_list[j]
            for i in range(0, 15):
                segNo = 15 - i
                path = lasersensor_path + device
                x = self.memory.getData(path + "Horizontal/Seg%02d/X/Sensor/Value" % segNo)
                y = self.memory.getData(path + "Horizontal/Seg%02d/Y/Sensor/Value" % segNo)
                self.position[j][i] = self.convert_laser_position(j, x, y)

    #  レーザーを描画するだけのスレッド
    '''
    def run(self):c
        ax = plt.axes()

        try:
            while True:
                ax.cla()
                ax.set_xlim(-3,3)
                ax.set_ylim(-3,3)
                # draw robot
                robot_pose = almath.Pose2D(self.robot_pose)
                xn = robot_pose.x + 0.2 * math.cos(robot_pose.theta)
                yn = robot_pose.y + 0.2 * math.sin(robot_pose.theta)
                plt.plot([robot_pose.x, xn], [robot_pose.y, yn], c="black")
                c = patches.Circle(xy=(robot_pose.x, robot_pose.y), radius=0.2, fill=False, color="black")
                ax.add_patch(c)

                #laser_values = self.convert_laser_position(self.get_laser_values())
                self.get_laser_values()
                for sensor_device in range(3):
                    print(self.position[sensor_device])
                    for value in range(15):
                        laser = self.position[sensor_device][value]
                        try:
                            plt.scatter(laser[0], laser[1], s=20,c=color[sensor_device])
                        except Exception as errorMsg1:
                            failure(errorMsg1)
                ax.set_aspect("equal")
                ax.grid()
                plt.pause(0.02)
        except Exception as errorMsg:
            failure("running is failed: {}".format(str(errorMsg)))
            exit(-1)
    '''

    # レーザー値を元にグリッドマップを描画するスレッド
    def run(self):
        color = ["red", "blue", "green"]
        ax = plt.axes()
        xyreso = 0.15
        yawreso = np.deg2rad(10.0)

        try:
            while True:
                ax.cla()

                # draw robot
                robot_pose = almath.Pose2D(self.robot_pose)
                xn = robot_pose.x + 0.2 * math.cos(robot_pose.theta)
                yn = robot_pose.y + 0.2 * math.sin(robot_pose.theta)
                plt.plot([robot_pose.x ,xn], [robot_pose.y, yn], c="black")
                c = patches.Circle(xy=(robot_pose.x, robot_pose.y), radius=0.2, fill=False, color="black")
                ax.add_patch(c)

                ox = list()
                oy = list()
                self.get_laser_values()
                for sensor_device in range(1):
                    for value in range(15):
                        laser = self.position[sensor_device][value]
                        plt.scatter(laser[0], laser[1], s=20,c=color[sensor_device])
                        ox.append(laser[0])
                        oy.append(laser[1])
                pmap, minx, maxx, miny, maxy, xyreso = self.generate_ray_casting_grid_map(ox, oy, xyreso, yawreso)
                self.draw_heatmap(pmap, minx, maxx, miny, maxy, xyreso)
                ax.set_xlim(-10,10)
                ax.set_ylim(-10,10)
                ax.set_aspect("equal")
                plt.pause(0.05)
        except Exception as message:
            failure("running is failed: {}".format(str(message)))
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
