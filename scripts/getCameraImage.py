import sys
import qi
import numpy as np
import cv2

NAME = "camera_image"
CAMERA_ID = 1
RESOLUTION = 2
FRAME_RATE = 15
COLOR_SPACE = 13

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

        width   = result[0]
        height  = result[1]
        array   = result[6]

        image = np.zeros((height, width, 3), np.uint8)

        while True:
            if result == None:
                print 'cannot capture.'
            elif result[6] == None:
                print 'no image data string'
            else:
                values = map(ord, str((array)))
                i = 0
                for y in range(0, height):
                    for x in range(0, width):
                        image.itemset((y,x,0),values[i + 0])
                        image.itemset((y,x,1),values[i + 1])
                        image.itemset((y,x,2),values[i + 2])
                        i += 3

                cv2.imshow("test",image)

            if cv2.waitKey(33) == 27:
                self.camera.unsubscribe(NAME)
                break


if __name__ == "__main__":
    ip = "127.0.0.1"
    if(len(sys.argv))>1:
        ip = sys.argv[1]
    print "IP: " + ip
    win = CameraImage(ip)
