#!/usr/bin/env python

import windowDetection_new
import rospy
import yaml
import cv2 as cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

mainUIName = "Main UI"

class TestNode:

    def __init__(self, paramsFile):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/quadrotor_1/front/image_raw",Image,self.imageCallback)

        self._detector = windowDetection_new.ColorBasedWindowDetector()
        self._detector.setThresholds(0, 0, 0, 179, 255, 255, 30)

        self._paramsFile = paramsFile
        with open(paramsFile) as f:

            data = yaml.load(f, Loader=yaml.FullLoader)
            self.lowH = 0 if not data.has_key("lowH") else data["lowH"]
            self.lowS = 0 if not data.has_key("lowS") else data["lowS"]
            self.lowV = 0 if not data.has_key("lowV") else data["lowV"]
            self.highH = 179 if not data.has_key("highH") else data["highH"]
            self.highS = 255 if not data.has_key("highS") else data["highS"]
            self.highV = 255 if not data.has_key("highV") else data["highV"]
            self.thresh = 30 if not data.has_key("thresh") else data["thresh"]
            self._detector.setThresholds(self.lowH, self.lowS, self.lowV, self.highH, self.highS, self.highV, self.thresh)

        self._windowCreated = False

    def onSlidersChange(self):
        self._detector.setThresholds(self.lowH, self.lowS, self.lowV, self.highH, self.highS, self.highV, self.thresh)

        with open(self._paramsFile, "r") as f:
            dict = yaml.load(f, Loader=yaml.FullLoader)

        dict["lowH"] = self.lowH
        dict["lowS"] = self.lowS
        dict["lowV"] = self.lowV
        dict["highH"] = self.highH
        dict["highS"] = self.highS
        dict["highV"] = self.highV
        dict["thresh"] = self.thresh

        with open(self._paramsFile, "w") as f:
            yaml.dump(dict, f)

    def lowHChanged(self, value):
        self.lowH = value
        self.onSlidersChange()
    def lowSChanged(self, value):
        self.lowS = value
        self.onSlidersChange()
    def lowVChanged(self, value):
        self.lowV = value
        self.onSlidersChange()
    def highHChanged(self, value):
        self.highH = value
        self.onSlidersChange()
    def highSChanged(self, value):
        self.highS = value
        self.onSlidersChange()
    def highVChanged(self, value):
        self.highV = value
        self.onSlidersChange()
    def threshChanged(self, value):
        self.thresh = value
        self.onSlidersChange()

    def imageCallback(self,data):

        image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        output = self._detector.detect(image)

        if ~self._windowCreated:
            cv2.namedWindow(mainUIName)
            cv2.createTrackbar("Low H", mainUIName , self.lowH, 180, self.lowHChanged)
            cv2.createTrackbar("High H", mainUIName , self.highH, 180, self.highHChanged)
            cv2.createTrackbar("Low S", mainUIName , self.lowS, 255, self.lowSChanged)
            cv2.createTrackbar("High S", mainUIName , self.highS, 255, self.highSChanged)
            cv2.createTrackbar("Low V", mainUIName , self.lowV, 255, self.lowVChanged)
            cv2.createTrackbar("High V", mainUIName , self.highV, 255, self.highVChanged)
            cv2.createTrackbar("Thresh", mainUIName , self.thresh, 255, self.threshChanged)

        output = cv2.cvtColor(output, cv2.COLOR_GRAY2RGB)
        cv2.imshow(mainUIName, cv2.hconcat([output, image]))
        cv2.waitKey(1)


rospy.init_node('test_node', anonymous=True)
testNode = TestNode("../params/params.yaml")

rospy.spin()
