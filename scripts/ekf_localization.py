import os
import sys
import time
import csv

sys.path.insert(0,"/home/nao/scripts/_naoqios")
import qi
import numpy as np

import matplotlib.pyplot as plt
import matplotlib.patches as patches

import almath
import math

import signal

from camera_config import *

class Authenticator:

    def __init__(self, user, pswd):
        self.user = user
        self.pswd = pswd

    def initialAuthData(self):
        cm = {'user': self.user, 'token': self.pswd}
        return cm

class ClientFactory:

    def __init__(self, user, pswd):
        self.user = user
        self.pswd = pswd

    def newAuthenticator(self):
        return Authenticator(self.user, self.pswd)

class Visualizer:
    def __init__(self):
        self.r = 0.2
        self.pos = almath.Pose2D(0,0,0)
        fig = plt.figure(figsize=(8,8))
        self.ax = plt.axes()

    def config_screen(self):
        self.ax.cla()
        self.ax.set_aspect('equal')
        self.ax.set_xlim(-1,5)
        self.ax.set_ylim(-3,3)

    def set_marker_pos(self, pos):
        self.pos = pos

    def draw_robot(self):
        x, y, theta = almath.Pose2D().toVector()

        xn = x + self.r * math.cos(theta)
        yn = y + self.r * math.sin(theta)
        self.ax.plot([x,xn], [y,yn], color="black")
        c = patches.Circle(xy=(x,y), radius=self.r, fill=False, color="black")
        self.ax.add_patch(c)

    def draw_marker(self):
        x, y, theta = self.pos.toVector()

        ux = 0.3 * math.cos(theta)
        vx = 0.3 * math.sin(theta)
        self.ax.quiver(x, y, ux, vx, angles="xy", scale_units='xy', alpha=1, width=0.003, scale=1)

        uy = - 0.3 * math.sin(theta)
        vy =   0.3 * math.cos(theta)
        self.ax.quiver(x, y, uy, vy, angles="xy", scale_units='xy', alpha=1, width=0.003, scale=1)

class KalmanFilter:
    def __init__(self):
        print("www")
        
class EKFLocalization:
    def __init__(self, ip):
        self.ip = ip
        self.aruco = cv2.aruco
        self.dictionary = self.aruco.getPredefinedDictionary(self.aruco.DICT_4X4_1000)
        self.image_remote = None
        self.subscriber_id = None

        self.viz = Visualizer()

        signal.signal(signal.SIGINT, self.handler)

        self.connect()

    def handler(self, signal, frame):
        self.unsubscribe()

    def connect(self):
        try:
            try:
                self.session = qi.Session()
                port = 9559
                self.session.connect("tcp://" + self.ip + ":" + str(port))
            except Exception as errorMsg:
                try:
                    self.session = qi.Session()
                    factory = ClientFactory("nao", "nao")
                    self.session.setClientAuthenticatorFactory(factory)
                    self.session.connect('tcps://{ip}:9503'.format(ip=self.ip))
                    print("ok connection")
                except Exception as errorMsg2:
                    print(errorMsg2)
            self.camera = self.session.service("ALVideoDevice")
            self.motion = self.session.service("ALMotion")
        except Exception as errorMsg3:
            print("Error when creating proxy: => " + str(errorMsg3))

        self.subscribe()
        self.run()

    def subscribe(self, params=DEFAULT_PARAMS):
        if self.subscriber_id is None:
            self.params = dict(DEFAULT_PARAMS) # copy default params
            self.params.update(params)
            # subscribe
            camera = self.params["camera"]
            resolution = self.params["resolution"]
            color_space, channels = self.params["color_space_and_channels"]
            self.params["dictionary"] = cv2.aruco.getPredefinedDictionary(self.params["dictionary"])
            fps = CAMERA_DATAS_AT_RESOLUTION[resolution]["fps"]
            self.subscriber_id = self.camera.subscribeCamera(SUBSCRIBER_ID,
                                                              camera,
                                                              resolution,
                                                              color_space,
                                                              fps)
            if self.params["exposure"]:
                self.camera.setParameter(self.params["camera"], CAMERA_PARAMETERS["AutoExposition"], 0)
                self.camera.setParameter(self.params["camera"], CAMERA_PARAMETERS["Exposure"], 400)
            resolution = self.params["resolution"]
            fps = CAMERA_DATAS_AT_RESOLUTION[resolution]["fps"]
            # self.periodic_tasks.setUsPeriod(1000000 / fps)
            # self.periodic_tasks.start(True)
            print("subscribe done")
        else:
            raise Exception("DXAruco is already running")

    def unsubscribe(self):
        print("unsubscribe...")
        if self.subscriber_id is not None:
            self.camera.unsubscribe(self.subscriber_id)
            # self.periodic_tasks.stop()
            self.subscriber_id = None
            print("unsubscribe done")
            exit(1)
        else:
            raise Exception("DXAruco is not running")

    def calc_marker(self):
        start_time = time.time()

        try:
            self.image_remote = self.camera.getImageRemote(self.subscriber_id)
        except Exception as message:
            self.unsubscribe()
            print(str(message))
        if not self.image_remote:
            self.unsubscribe()
            raise Exception("No data in image")
        camera = self.params["camera"]
        camera_name = CAMERAS[camera]
        seconds = self.image_remote[4]
        micro_seconds = self.image_remote[5]
        t_world2camera = almath.Transform(self.motion._getSensorTransformAtTime(camera_name, seconds*10e8+micro_seconds*10e2))
        t_robot2camera = almath.Transform(self.motion.getTransform(camera_name, 2, True))
        resolution = self.params["resolution"]
        x, y = CAMERA_DATAS_AT_RESOLUTION[resolution]["image_size"]
        color_space, channels = self.params["color_space_and_channels"]
        image = numpy.frombuffer(self.image_remote[6], dtype = numpy.uint8).reshape(y, x, channels)
        if self.params["color"] and color_space == vd.kBGRColorSpace:
            print("Thresholding image...")
            lower_b = tuple([int(val) for val in self.params["color"]])
            upper_b = (255, 255, 255)
            image = cv2.inRange(image, lower_b, upper_b)
            print("Thresholding image done")

        p6Ds = dict()
        corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(image, self.params["dictionary"])

        result = False
        marker_pos = list()
        if ids is not None:
            try:
                if [965] in ids:
                    count = 0
                    for _id in ids:
                        if _id == [965]:
                            break
                        count = count + 1

                    rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, \
                                                                          self.params["size"], \
                                                                          CAMERA_DATAS_AT_RESOLUTION[resolution]["matrix"], \
                                                                          CAMERA_DISTORTION_COEFF)

                    tvec = tvecs[count][0]
                    x, y, z = tvec[2], -tvec[0], -tvec[1]
                    p3d_camera2target = almath.Position3D(x, y, z)

                    rvec = rvecs[count][0]
                    wx, wy, wz = rvec[2], -rvec[0], -rvec[1]
                    proj_rvec, _ = cv2.Rodrigues(numpy.array([wx, wy, wz]))

                    r_camera2target = almath.Rotation(proj_rvec.flatten())
                    t_camera2target = almath.transformFromRotationPosition3D(r_camera2target, p3d_camera2target)

                    r3d_correction = almath.Rotation3D(0., 3*math.pi/2, 0)

                    t_corretion = almath.transformFromRotation3D(r3d_correction)
                    t_world2target = t_world2camera * t_camera2target * t_corretion
                    t_robot2target = t_robot2camera * t_camera2target * t_corretion

                    p6D_world2target = almath.position6DFromTransform(t_world2target)
                    p6D_robot2target = almath.position6DFromTransform(t_robot2target)
                    
                    print("[x,y,theta] = [{},{},{}]".format(p6D_robot2target.x, p6D_robot2target.y, math.degrees(p6D_robot2target.wz)))

                    marker_pos = list(p6D_robot2target.toVector())
                    result = {"result": True, "p6D_robot2target": marker_pos}
                else:
                    result = {"result": False, "p6D_robot2target": marker_pos}
            except Exception as message:
                print("failed: {}".format(str(message)))
                self.unsubscribe()
        else:
            result = {"result": False, "p6D_robot2target": marker_pos}
            print("No Marker")
        delta_time = time.time() - start_time
        return result

    def save_csv(self, marker_pos):
        f = open("plot.csv", "a")
        writer = csv.writer(f, lineterminator="\n")
        marker_data = list()
        marker_data.append(marker_pos[0])
        marker_data.append(marker_pos[1])
        marker_data.append(marker_pos[5])
        writer.writerow(marker_data)
        f.close()

    def run(self):
        try:
            while True:
                self.viz.config_screen()
                self.viz.draw_robot()
                result = self.calc_marker()
                if result["result"]:
                    marker_pos = result["p6D_robot2target"]
                    self.viz.set_marker_pos(almath.Pose2D(marker_pos[0], marker_pos[1], marker_pos[5]))
                    self.viz.draw_marker()
                plt.pause(0.01)
        except Exception as message:
            print("running is failed : "+str(message))
            self.unsubscribe()

if __name__ == "__main__":
    ip = "127.0.0.1"
    if(len(sys.argv)) > 1:
        ip = sys.argv[1]
    print("IP:" + ip)
    win = EKFLocalization(ip)
