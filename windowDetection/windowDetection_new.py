#!/usr/bin/env python

import cv2 as cv2
import time
import math
import numpy as np

class MovingAverageFilter:

    def __init__(self, windowSize):
        self._windowSize = windowSize
        self._window = []

    def update(self, newValue):

        if len(self._window) >= self._windowSize:
            self._window.remove(self._window[0])

        self._window.append(newValue)

        return sum(self._window) / len(self._window)



class ColorBasedWindowDetector:
    "This class detects windows of a building with known wall colors, in an input image."

    def __init__(self):

        self._lastEndTime = None
        self._FPSMAFilter = MovingAverageFilter(10)
        self._DTMAFilter = MovingAverageFilter(10)

        self.colors = list(255*np.random.rand(100, 3))

        return


    def setThresholds(self, lowH, lowS, lowV, highH, highS, highV, thresh):

        self._lowH = lowH
        self._lowS = lowS
        self._lowV = lowV
        self._highH = highH
        self._highS = highS
        self._highV = highV
        self._thresh = thresh

    def detect(self, image):

        startTime = time.time()

        preProcessed = self.preProcess(image)
        # self.extractContours(preProcessed, image)
        self.extraxtLines(image)

        # FPS and calculation time
        endTime = time.time()
        if self._lastEndTime != None:

            FPS = self._FPSMAFilter.update( 1. / (endTime - self._lastEndTime) )
            dt = self._DTMAFilter.update( endTime - startTime )
            cv2.putText(image,"Calculation Time: {:.03f} ms   FPS: {:.0f}".format(dt, FPS) ,(20,20), cv2.FONT_HERSHEY_PLAIN, 1, [0,0,0], 2)
            print "Calculation Time: {:.03f} ms   FPS: {:.0f}".format(dt, FPS)

        self._lastEndTime = endTime

        return preProcessed

    def preProcess(self, image):

        imageHSV = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        imageThresholded = cv2.inRange(imageHSV, (self._lowH, self._lowS, self._lowV),
                                                (self._highH, self._highS, self._highV))
        imageInversed = cv2.bitwise_not(imageThresholded)


        erosion_type = cv2.MORPH_RECT
        erosion_size1 = 3
        erosion_size2 = 2
        element1 = cv2.getStructuringElement(erosion_type, (2*erosion_size1 + 1, 2*erosion_size1+1), (erosion_size1, erosion_size1))
        element2 = cv2.getStructuringElement(erosion_type, (2*erosion_size2 + 1, 2*erosion_size2+1), (erosion_size2, erosion_size2))

        imageInversed = cv2.GaussianBlur(imageInversed,(3,3),1.2)
        imageInversed = cv2.erode(imageInversed, element1)
        _,imageInversed = cv2.threshold(imageInversed,self._thresh,255,cv2.THRESH_BINARY)
        imageInversed = cv2.dilate(imageInversed, element2)

        return imageInversed

    def extractContours(self, imageBin, image):
        _, contours, _ = cv2.findContours(imageBin, cv2.RETR_TREE, cv2.CHAIN_APPROX_TC89_KCOS)
        convexes = []
        polies = []
        # windows = []

        i=0
        for cnt in contours:
            if cv2.contourArea(cnt)<100: continue
            polies.append(cv2.approxPolyDP(cnt,5, True))
            # windows.append(reconstructRect(cnt))
            convexes.append(cv2.convexHull(cnt))
        # cv2.drawContours(image, contours, -1, (0,0,255), 1)
        cv2.drawContours(image, convexes, -1, (0,255,0), 2)
        cv2.drawContours(image, polies, -1, (255,0,0), 2)
        # return polies, windows

    def visualizeLines(self, image, lines, color):

        if lines is not None:
            for i in range(0, len(lines)):
                rho = lines[i][0][0]
                theta = lines[i][0][1]
                a = math.cos(theta)
                b = math.sin(theta)
                x0 = a * rho
                y0 = b * rho
                pt1 = (int(x0 + 1000*(-b)), int(y0 + 1000*(a)))
                pt2 = (int(x0 - 1000*(-b)), int(y0 - 1000*(a)))
                cv2.line(image, pt1, pt2, color, 1, cv2.LINE_AA)


    def mergeRelatedLines(self, lines):

        notRelatedIdxs = range(len(lines))

        for i in range(len(lines)):
            r = lines[i][0][0]
            t = lines[i][0][1]

            for j in range(i+1,len(lines)):
                if abs(lines[j][0][0] - r) < 10 and abs(lines[j][0][1] - t) < math.pi / 18 :
                    try:
                        notRelatedIdxs.remove(j)
                    except:
                        pass

        return [lines[idx] for idx in notRelatedIdxs]

    def classifyLines(self, inputLines):

        lines = list(inputLines)
        classifieds = []

        for idx, line in enumerate(lines):

            if line[0][1] == None: continue

            parallels = []
            parallels.append([[line[0][0],line[0][1]]])
            for idx1, line1 in enumerate(lines):
                if line1[0][1] == None or idx == idx1: continue
                if abs(line[0][1] - line1[0][1]) < math.pi / 18:
                    parallels.append([[line1[0][0],line1[0][1]]])
                    lines[idx1][0][1] = None

            if len(parallels) > 1:
                # print "\n\n\n\n"
                # print parallels
                classifieds.append(parallels)

            lines[idx][0][1] = None

        return classifieds

    def findIntersection(self, line1, line2):

        rho1, theta1 = line1[0]
        rho2, theta2 = line2[0]
        A = np.array([
            [np.cos(theta1), np.sin(theta1)],
            [np.cos(theta2), np.sin(theta2)]
        ])
        b = np.array([[rho1], [rho2]])
        x0, y0 = np.linalg.solve(A, b)
        x0, y0 = int(np.round(x0)), int(np.round(y0))
        return [x0, y0]

    def findSegmentedIntersection(self, lines):
        segmentedIntersections = []
        for i, group in enumerate(lines[:-1]):
            for next_group in lines[i+1:]:

                intersections = []
                for line1 in group:
                    for line2 in next_group:
                        intersections.append(self.findIntersection(line1, line2))

                if len(intersections) > 3:
                    segmentedIntersections.append(intersections)

        return segmentedIntersections

    def findRectPattern(self, intersections, shape, divisionFactor):
        erosion_type = cv2.MORPH_RECT
        erosion_size1 = 1
        erosion_size2 = 2
        element1 = cv2.getStructuringElement(erosion_type, (2*erosion_size1 + 1, 2*erosion_size1+1), (erosion_size1, erosion_size1))
        element2 = cv2.getStructuringElement(erosion_type, (2*erosion_size2 + 1, 2*erosion_size2+1), (erosion_size2, erosion_size2))

        rects = []

        for idx, group in enumerate(intersections):
            bin = np.zeros((shape[0]/divisionFactor, shape[1]/divisionFactor, 1), dtype = "uint8")
            for isc in group:
                cv2.circle(bin, (isc[0]/divisionFactor,isc[1]/divisionFactor), 1, 255, -1)

            bin = cv2.erode(bin, element1)
            # bin = cv2.dilate(bin, element1)
            # bin = cv2.erode(bin, element2)
            # bin = cv2.erode(bin, element2)
            # bin = cv2.dilate(bin, element2)
            # bin = cv2.erode(bin, element2)

            cv2.imwrite("bin_{:.0f}.jpg".format(idx), bin)

            # if idx == 0:
            #     cv2.imshow("image {:.0f}".format(idx), bin)

            _, contours, _ = cv2.findContours(bin, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

            for cnt in contours:
                if cv2.contourArea(cnt)<bin.shape[0]*bin.shape[1]/20: continue
                polygon = cv2.approxPolyDP(cnt,5, True)
                # print "poly"
                # if len(polygon) == 4:
                rects.append(polygon)

        for contour in rects:
            contour[:, :, 0] = contour[:, :, 0] * divisionFactor
            contour[:, :, 1] = contour[:, :,  1] * divisionFactor

        return rects

    def extraxtLines(self, image):
        gray = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray,70,100,apertureSize = 3)
        # kernel = np.ones((3,3),np.uint8)
        # edges = cv2.dilate(edges,kernel,iterations = 1)
        # kernel = np.ones((5,5),np.uint8)
        # edges = cv2.erode(edges,kernel,iterations = 1)

        # cv2.imshow("gray", gray)
        # cv2.imshow("edges", edges)

        lines = cv2.HoughLines(edges,1,np.pi/180,90)
        notRelatedLines = self.mergeRelatedLines(lines)
        classifiedLines = self.classifyLines(notRelatedLines)

        intersections = self.findSegmentedIntersection(classifiedLines)
        buildings = self.findRectPattern(intersections,image.shape,10)

        # print len(buildings), len(buildings[0]), len(buildings[0][0][0])

        # for rect in buildings:
            # cv2.rectangle(image, tuple(20*rect[0][0]), tuple(20*rect[2][0]), (255,0,0), 2)
        cv2.drawContours(image, buildings, -1, (255,0,0), 2)


        for idx, group in enumerate(intersections):
            for isc in group:
                cv2.drawMarker(image, tuple(isc),self.colors[idx%100], markerType=cv2.MARKER_STAR,markerSize=5, thickness=1, line_type=cv2.LINE_AA)

        # for idx, prallelLines in enumerate(classifiedLines):
        #     self.visualizeLines(image, prallelLines, self.colors[idx%100])
