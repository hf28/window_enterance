#!/usr/bin/env python

import cv2 as cv2
import time

class ColorBasedWindowDetector:
    "This class detects windows of a building with known wall colors, in an input image."

    def __init__(self):

        self._lastEndTime = None
        return


    def setThresholds(self, lowH, lowS, lowV, highH, highS, highV):

        self._lowH = lowH
        self._lowS = lowS
        self._lowV = lowV
        self._highH = highH
        self._highS = highS
        self._highV = highV


    def detect(self, image):

        startTime = time.time()

        preProcessed = self.preProcess(image)

        # FPS and calculation time
        endTime = time.time()
        if self._lastEndTime != None:

            FPS = 1. / (endTime - self._lastEndTime)
            dt = endTime - startTime
            cv2.putText(image,"Calculation Time: {:.03f} ms   FPS: {:.0f}".format(dt, FPS) ,(20,20), cv2.FONT_HERSHEY_PLAIN, 1, [0,0,0], 2)

        self._lastEndTime = endTime

        return preProcessed

    def preProcess(self, image):

        imageHSV = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        imageThresholded = cv2.inRange(imageHSV, (self._lowH, self._lowS, self._lowV),
                                                (self._highH, self._highS, self._highV))
        return cv2.bitwise_not(imageThresholded)
