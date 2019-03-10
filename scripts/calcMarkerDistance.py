import sys
import time

import qi
import numpy as np
import cv2
import cv2.aruco
import almath
import math

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

class CameraImage:
    def __init__(self, ip):
        self.ip = ip
        self.aruco = cv2.aruco
        self.dictionary = self.aruco.getPredefinedDictionary(self.aruco.DICT_4X4_1000)
        self.image_remote = None
        self.subscriber_id = None

        self.connect()

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
        else:
            raise Exception("DXAruco is not running")

    def calc_marker(self):
        print("calculation marker information")
        start_time = time.time()

        self.image_remote = self.camera.getImageRemote(self.subscriber_id)
        if not self.image_remote:
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
        print(ids)

        result = False
        if ids:
            if ids == [965]:
                rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, \
                                                                      self.params["size"], \
                                                                      CAMERA_DATAS_AT_RESOLUTION[resolution]["matrix"], \
                                                                      CAMERA_DISTORTION_COEFF)

                tvec = tvecs[0][0]
                x, y, z = tvec[2], -tvec[0], -tvec[1]
                p3d_camera2target = almath.Position3D(x, y, z)

                rvec = rvecs[0][0]
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

                p6Ds[ids] = {
                    "robot2target": list(p6D_robot2target.toVector()),
                    "world2target": list(p6D_world2target.toVector())
                }
                result = True
            print("ID:" + str(ids))
        else:
            result = False
            print("No Marker")
        delta_time = time.time() - start_time
        #print("task done in %s seconds" & delta_time)
        return result

    def run(self):
        
        while True:
            self.calc_marker()
            '''
            width = self.image_remote[0]
            height = self.image_remote[1]
            
            image = np.zeros((height, width, 3), np.uint8)

            if self.image_remote == None:
                print('cannot capture.')
            elif self.image_remote[6] == None:
                print('no image data string')
            else:
                values = map(ord, str(self.image_remote[6]))
                i = 0
                for y in range(0, height):
                    for x in range(0, width):
                        image.itemset((y, x, 0), values[i + 0])
                        image.itemset((y, x, 1), values[i + 1])
                        image.itemset((y, x, 2), values[i + 2])
                        i += 3

                corners, ids, rejectedImgPoints = self.aruco.detectMarkers(image, self.dictionary)
                self.aruco.drawDetectedMarkers(image, corners, ids, (0, 255, 0))
                cv2.imshow("test", image)

            if cv2.waitKey(33) == 27:
                self.camera.unsubscribe(NAME)
                break
            '''

if __name__ == "__main__":
    ip = "127.0.0.1"
    if(len(sys.argv)) > 1:
        ip = sys.argv[1]
    print("IP:" + ip)
    win = CameraImage(ip)
