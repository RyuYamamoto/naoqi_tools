import sys
import qi
import numpy as np
import cv2

NAME = "camera_image"
CAMERA_ID = 2
RESOLUTION = 1
FRAME_RATE = 15
COLOR_SPACE = 17

FX = 525.0 / 2
FY = 525.0 / 2
# Optical center
CX = 319.5 / 2
CY = 239.5 / 2

# meter to millimeter
UNIT_SCALING = 0.001

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

class DepthImage:
    def __init__(self, ip):
        self.ip = ip

        self.connect()

    def connect(self):
        try:
            try:
                self.session = qi.Session()
                port = 9559
                self.session.connect("tcp://" + self.ip + ":" + str(port))
            except Exception,errorMsg:
                try:
                    self.session = qi.Session()
                    factory = ClientFactory("nao", "nao")
                    self.session.setClientAuthenticatorFactory(factory)
                    self.session.connect('tcps://{ip}:9503'.format(ip=self.ip))
                    print "ok connection"
                except Exception,errorMsg2:
                    print errorMsg2
                self.camera = self.session.service("ALVideoDevice")
                self.captureDevice = self.camera.subscribeCamera(NAME, CAMERA_ID, RESOLUTION, COLOR_SPACE, FRAME_RATE)
        except Exception,errorMsg:
            print "Error when creating proxy: "
            print str(errorMsg)

        self.run()

    def run(self):
        result = self.camera.getImageRemote(self.captureDevice)

        if result is None:
            print 'cannnot obtain depth image.'
            exit()

        width   = result[0]
        height  = result[1]
        array   = result[6]

        cloud = []
        for v in range(height):
            for u in range(width):
                offset = (v * width + u) * 2
                depth = int(array[offset]) + int(array[offset+1]) * 256

                x = (u - CX) * depth * UNIT_SCALING / FX
                y = (v - CY) * depth * UNIT_SCALING / FY
                z = depth * UNIT_SCALING

                cloud.append((x, y, z))

        self.camera.unsubscribe(NAME)

        header = '''# .PCD v0.7 - Point Cloud Data file format
	VERSION 0.7
	FIELDS x y z
	SIZE 4 4 4
	TYPE F F F
	COUNT 1 1 1
	WIDTH %d
	HEIGHT 1
	VIEWPOINT 0 0 0 1 0 0 0
	POINTS %d
	DATA ascii'''

	f = open("test.pcd", 'w')
	num = len(cloud)
	f.write(header % (num, num))
	f.write("\n")
	for c in cloud:
	    f.write('%f %f %f' % (c[0], c[1], c[2]))
	    f.write("\n")
	f.close()

if __name__ == "__main__":
    ip = "127.0.0.1"
    if(len(sys.argv))>1:
        ip = sys.argv[1]
    print "IP: " + ip
    win = DepthImage(ip)
