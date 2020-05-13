
#!/usr/bin/env python
from __future__ import print_function

import roslib
roslib.load_manifest('py_simulate')
import sys
import rospy
import cv2 as cv
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import math
from simulate.msg import imtodyn

it =0
cv_image = np.zeros((1, 1, 3), np.uint8)
frame_ = np.zeros((1, 1, 3), np.uint8)
window_frame = np.zeros((1, 1, 3), np.uint8)
imdata = np.zeros((1, 1, 3), np.uint8)
max_canny1, max_canny2 = 800, 800
max_GuKernelSize = 50
window_capture_name = "Video Capture"
max_value_H, max_value = 360/2, 255
low_H, low_S, low_V, thresh = 0, 57, 37, 30
high_H, high_S, high_V = 22, 108, 113
canny1, canny2, GuKernelSize, GuSigma = 131, 0, 7, 1.2
ity, itz = 0, 0
shapeAR=1000
euc, eucFilterIt, eucFIt, areaIt, iter = 0, 0, 0, 0, 0
vecy, vecz, tempvecy, tempvecz, tempArea, xc, yc = 0,0,0,0,0, 0, 0
flag, long_distance, enterance = False, False, False
oldP = [[0,0]]
newP = [[0,0]]
window_points = []

# tracker = cv.TrackerBoosting_create()
# tracker = cv.TrackerTLD_create()
tr_it = 0

index = []
index_before = []
index_change_x = False
index_change_y = False
frpl2 = False
frpl1 = False
fcpl2 = False
fcpl1 = False

seed = []
is_set = False

camera_matrix = np.float64([[376.744103, 0, 319.513089],
                [0,  376.572581,178.056011],
                [0.0,0.0,      1.000000]])
distortion_matrix = [-0.000545, 0.000835, -0.000038, -0.000143, 0.000000]
distortion_matrix = np.array(distortion_matrix)
distortion_matrix = distortion_matrix.reshape((5,1))
x1, y1 = 1.5, 1
real_rect_info = np.float32([[0, 0, 0], [x1, 0, 0], [x1, y1, 0], [0, y1, 0]])
#
pub = rospy.Publisher('visual_info', imtodyn, queue_size=1)
msg = imtodyn()

def on_low_H_thresh_trackbar(val):
    global low_H
    global high_H
    low_H = val
    low_H = min(high_H-1, low_H)
    cv.setTrackbarPos("Low H", window_capture_name, low_H)
def on_high_H_thresh_trackbar(val):
    global low_H
    global high_H
    high_H = val
    high_H = max(high_H, low_H+1)
    cv.setTrackbarPos("High H", window_capture_name, high_H)
def on_low_S_thresh_trackbar(val):
    global low_S
    global high_S
    low_S = val
    low_S = min(high_S-1, low_S)
    cv.setTrackbarPos("Low S", window_capture_name, low_S)
def on_high_S_thresh_trackbar(val):
    global low_S
    global high_S
    high_S = val
    high_S = max(high_S, low_S+1)
    cv.setTrackbarPos("High S", window_capture_name, high_S)
def on_low_V_thresh_trackbar(val):
    global low_V
    global high_V
    low_V = val
    low_V = min(high_V-1, low_V)
    cv.setTrackbarPos("Low V", window_capture_name, low_V)
def on_high_V_thresh_trackbar(val):
    global low_V
    global high_V
    high_V = val
    high_V = max(high_V, low_V+1)
    cv.setTrackbarPos("High V", window_capture_name, high_V)
def on_thresh_thresh_trackbar(val):
    global thresh
    # global high_V
    thresh = val
    # high_V = max(high_V, low_V+1)
    cv.setTrackbarPos("thresh", window_capture_name, thresh)

def onMouse(event, x, y, flags, param):
    global seed, is_set, tempArea, tempvecy, tempvecz, frame_, cv_image, index, index_before
    if event == cv.EVENT_LBUTTONDOWN:
        seed = (x,y)
        # print(seed)
        frame_ = cv_image.copy()
        im = cv_image.copy()
        pic = preProcessing(im)
        wins = contourExtraction(pic, im)
        minDist=1e7
        dist=-1
        choiceIndex=-1
        i = 0
        for win in wins:
            if cv.pointPolygonTest(win, seed, False)!=-1:
                dist = cv.pointPolygonTest(win, seed, True)
                if minDist>dist:
                    minDist = dist
                    choiceIndex = i
            i = i+1
        tempArea = cv.contourArea(wins[choiceIndex])
        index = windowIndex(wins, wins[choiceIndex])
        index_before = index
        print(index)
        cv.drawContours( frame_, wins, choiceIndex, (255,0,0), 2);
        is_set = True

def setWindow(img):
    if is_set: return
    global frame_
    cv.namedWindow("operator desicion")
    cv.setMouseCallback("operator desicion", onMouse)
    pic = preProcessing(img)
    wins = contourExtraction(pic, img)

    cv.imshow("operator desicion", img)
    # print(1)
    cv.waitKey(1)
    # cv.destroyAllWindows()

def preProcessing(img):
    # cv.namedWindow(window_capture_name)
    # cv.createTrackbar("Low H", window_capture_name , low_H, max_value_H, on_low_H_thresh_trackbar)
    # cv.createTrackbar("High H", window_capture_name , high_H, max_value_H, on_high_H_thresh_trackbar)
    # cv.createTrackbar("Low S", window_capture_name , low_S, max_value, on_low_S_thresh_trackbar)
    # cv.createTrackbar("High S", window_capture_name , high_S, max_value, on_high_S_thresh_trackbar)
    # cv.createTrackbar("Low V", window_capture_name , low_V, max_value, on_low_V_thresh_trackbar)
    # cv.createTrackbar("High V", window_capture_name , high_V, max_value, on_high_V_thresh_trackbar)
    # cv.createTrackbar("thresh", window_capture_name , thresh, max_value, on_thresh_thresh_trackbar)
#
    erosion_type = cv.MORPH_RECT
    erosion_size1 = 3
    erosion_size2 = 2
    element1 = cv.getStructuringElement(erosion_type, (2*erosion_size1 + 1, 2*erosion_size1+1), (erosion_size1, erosion_size1))
    element2 = cv.getStructuringElement(erosion_type, (2*erosion_size2 + 1, 2*erosion_size2+1), (erosion_size2, erosion_size2))


    frame_HSV = cv.cvtColor(img, cv.COLOR_BGR2HSV)
    frame_threshold = cv.inRange(frame_HSV, (low_H, low_S, low_V), (high_H, high_S, high_V))
    frame_inverse = cv.bitwise_not(frame_threshold)

    frame_inverse = cv.GaussianBlur(frame_inverse,(3,3),1.2)
    frame_inverse = cv.erode(frame_inverse, element1)
    _,frame_inverse = cv.threshold(frame_inverse,thresh,255,cv.THRESH_BINARY)
    frame_inverse = cv.dilate(frame_inverse, element2)
#
    # cv.imshow(window_capture_name,frame_inverse)
    # cv.waitKey(1)
    return frame_inverse

def euclideanDist(p, q):
    p1 = np.array(p)
    p2 = np.array(q)
    p1 = p1.ravel()
    p2 = p2.ravel()
    diff = [p1[0]-p2[0], p1[1]-p2[1]]
    return math.sqrt((diff[0]*diff[0])+(diff[1]*diff[1]))

def reconstructRect(contour):
    if len(contour)==2:
        win = [[[contour[0][0][0], contour[0][0][1]]],
              [[contour[0][0][0], contour[1][0][1]]],
              [[contour[1][0][0], contour[1][0][1]]],
              [[contour[1][0][0], contour[0][0][1]]]]
        win = np.array(win)
        return win
    elif len(contour)==3:
        diff = -1
        for i in range(3):
            for j in range(3):
                if i==j: continue
                if diff<(abs(contour[i][0][0]-contour[j][0][0])+abs(contour[i][0][1]-contour[j][0][1])):
                    diff = abs(contour[i][0][0]-contour[j][0][0])+abs(contour[i][0][1]-contour[j][0][1])
                    diam1_1 = [[contour[i][0][0], contour[i][0][1]]]
                    diam1_2 = [[contour[j][0][0], contour[j][0][1]]]
        for k in range(3):
            di = [[contour[k][0][0], contour[k][0][1]]]
            if di!=diam1_1 and di!=diam1_2:
                diam2_1 = di
        # print(diam1_1)
        # print(diam1_2)
        # print(diam2_1)
        diam2_2 = [[diam1_2[0][i] + diam1_1[0][i] - diam2_1[0][i] for i in range(2)]]
        win = [diam1_1,diam2_1,diam1_2,diam2_2]
        win = np.array(win)
        return win
    elif len(contour)==4:
        return contour
    else:
        p1, p2, p3, p4 = [], [],[],[]
        min_xpy=1e6
        max_xpy=-1
        min_xmy=1e6
        max_xmy=-1e6
        for cnt in contour:
            xpy = cnt[0][0] + cnt[0][1]
            xmy = cnt[0][0] - cnt[0][1]
            if min_xpy>=xpy:
                min_xpy = xpy
                p1 = cnt
            if min_xmy>=xmy:
                min_xmy = xmy
                p2 = cnt
            if max_xpy<=xpy:
                max_xpy = xpy
                p3 = cnt
            if max_xmy<=xmy:
                max_xmy = xmy
                p4 = cnt
        win = [p1,p2,p3,p4]
        win = np.array(win)
        return win

def rectangleGeometric(points):
    global cv_image
    max = may = dx = xc = dy = yc = 0
    rows,cols,_ = cv_image.shape
    xpc = cols/2 - 1
    ypc = rows/2 - 1
    mix = miy = 1e6
    for point in points:
        if max<point[0][0]: max = point[0][0]
        if may<point[0][1]: may = point[0][1]
        if mix>point[0][0]: mix = point[0][0]
        if miy>point[0][1]: miy = point[0][1]
    xc = (max+mix)/2 + 1;
    yc = (may+miy)/2 + 1;
    dx = xc - xpc;
    dy = yc - ypc;
    return dx, dy;

def fourPSort(cont):
    success = False
    res = []
    if len(cont)==0: return res, success;
    xm = ym = 0
    for point in cont:
        xm = xm + point[0][0]
        ym = ym + point[0][1]
    xm = xm / len(cont)
    ym = ym / len(cont)
    c1 = []
    c2 = []
    c3 = []
    c4 = []
    for point in cont:
        if (point[0][0]<=xm)and(point[0][1]>=ym):
            c1.append(point)
        elif (point[0][0]>xm)and(point[0][1]>ym):
            c2.append(point)
        elif (point[0][0]>=xm)and(point[0][1]<=ym):
            c3.append(point)
        elif (point[0][0]<xm)and(point[0][1]<ym):
            c4.append(point)
    if (len(c1)==0) or (len(c2)==0) or (len(c3)==0) or (len(c4)==0): return res, success;
    c = c1 + c2 + c3 + c4
    success = True
    return success, c;

def contourExtraction(img, img0):
    _, contours, _ = cv.findContours(img, cv.RETR_TREE, cv.CHAIN_APPROX_TC89_KCOS)
    # polies = []
    windows = []
    # print("num of contours:  ", len(contours))
    rows,cols,_ = img0.shape
    i=0
    for cnt in contours:
        if cv.contourArea(cnt)<100: continue

        # polies.append(cv.approxPolyDP(cnt,13, True))
        # windows.append(reconstructRect(cnt))
        poly = reconstructRect(cv.approxPolyDP(cnt,13, True))
        cornerPoints = 0
        for pnt in poly:
            if (abs(pnt[0][0]-cols)<5 or pnt[0][0]<5) and (abs(pnt[0][1]-rows)<5 or pnt[0][1]<5):
                cornerPoints = cornerPoints + 1
        if cornerPoints>=2 : continue

        windows.append(poly)
    cv.drawContours(img0, windows, -1, (0,0,255), 2)
    return windows;

def boundingBox(contour):
    global cv_image
    rows, cols, _ = cv_image.shape
    max = may = 0
    mix = miy = 1e6
    for point in contour:
        if max<point[0][0]: max = point[0][0]
        if may<point[0][1]: may = point[0][1]
        if mix>point[0][0]: mix = point[0][0]
        if miy>point[0][1]: miy = point[0][1]
    box = (mix, miy, (max-mix), (may-miy))
    return box;

def contourCenter(contour):
    x, y = 0, 0
    # print(len(contour))
    for pt in contour:
        x = x + pt[0][0]
        y = y + pt[0][1]
    x = x/len(contour)
    y = y/len(contour)
    return x, y;

def findNearest(windowsInfo, win, xlim, ylim):
    # Each element of windowsInfo, consists of index, center x and y and the contour of the window
    minx = miny = 1e7
    nearestX_y = nearestY_x = tempX_y = tempY_x = 1e7
    farest_x = farest_y = -1
    nearestX = []
    nearestY = []
    tempX = []
    tempY = []
    wins = windowsInfo

    if xlim==0 or ylim==0:
        _,_, a, b = boundingBox(win[3])
        if xlim==0: xlim = (a*b)/math.sqrt((4*b*b)+(a*a))
        if ylim==0: ylim = (a*b)/math.sqrt((b*b)+(4*a*a))

    typicalSide = 0
    win_area = cv.contourArea(win[3])
    for wn in wins:
        area = cv.contourArea(wn[3])
        typicalSide = typicalSide + math.sqrt(area)
    typicalSide = typicalSide/len(wins)
    for wn in wins:
        distX = abs(win[1]-wn[1])
        distY = abs(win[2]-wn[2])
        if distX==0 and distY==0:
            continue
        if distX<xlim:
            if nearestX_y>distY:
                nearestX = wn
                nearestX_y = distY
            if farest_y<distY:
                farest_y = distY
        if distY<ylim:
            if nearestY_x>distX:
                nearestY = wn
                nearestY_x = distX
            if farest_x<distX:
                farest_x = distX
        if minx>distX and distX>=xlim:       # for single columns
            minx = distX
            tempX = wn
            tempX_y = distY
        if miny>distY and distY>=ylim:       # for single rows
            miny = distY
            tempY = wn
            tempY_x = distX
    if nearestY_x==1e7:
        farest_x = nearestY_x = tempY_x
        nearestY = tempY
    if nearestX_y==1e7:
        farest_y = nearestX_y = tempX_y
        nearestX = tempX
    if nearestX_y<typicalSide:
        nearestX_y = typicalSide
    if nearestY_x<typicalSide:
        nearestY_x = typicalSide
    if len(nearestX)==0: nearestX = [[1e6],[1e6],[1e6]]
    if len(nearestY)==0: nearestY = [[1e6],[1e6],[1e6]]
    return nearestX, nearestY, nearestX_y, nearestY_x, farest_x, farest_y;


def windowIndex(windows, input):
    global it, cv_image, tempArea, index, index_before, index_change_x, index_change_y, imdata, frpl2, frpl1, fcpl2, fcpl1, xc,yc

    # print('windowIndex entered ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~')
    it = it + 1
    index_in = len(input)==2
    window_in = len(input)==4

    if not (index_in or window_in):
        print("Wrong input to windowIndex!")
        return [0]

    if window_in:
        print('w input:', input)
    elif index_in:
        print('i input:', input)

    winTable = []
    for i, poly in enumerate(windows):
        area = cv.contourArea(poly)
        if abs(area-tempArea)>tempArea or area<0.2*tempArea:
            continue
        x, y = contourCenter(poly)
        winTable.append([i, x, y, np.int32(poly)])

    b = a = 1e7
    hors = []
    vers = []
    removeList = []
    for k, wn in enumerate(winTable):
        _, _, bl, al, _, _ = findNearest(winTable, wn, 0, 0)
        if b>bl: b = bl
        if a>al: a = al
        hors.append([al, wn])
        vers.append([bl, wn])

    for k, el in enumerate(hors):
        if abs(el[0]-a)>(2*a):
            if el[1] in winTable:
                winTable.remove(hors[k][1])
                el[0] = -1

    for k, el in enumerate(vers):
        if abs(el[0]-b)>(2*b):
            if el[1] in winTable:
                winTable.remove(vers[k][1])
                el[0] = -1

    maxHors = max(el[0] for el in hors)
    maxVers = max(el[0] for el in vers)

    numWin = len(winTable)
    if numWin<=2:
        print('Low number of contours to windowIndex!')
        return [1]

    ylim = (a*b)/math.sqrt((b*b)+(4*a*a))
    xlim = (a*b)/math.sqrt((4*b*b)+(a*a))
    # print(a,b,ylim, xlim)

    classifyTable = winTable
    indexBox = []
    k = 0
    windowMatrix = []
    winRow = []
    classifyTable.sort(key=lambda x: x[1])
    while k<numWin:
        if len(indexBox)>=numWin or k>=len(classifyTable):
            break
        if classifyTable[k][0] in indexBox:
            k = k+1
            continue
        else:
            indexBox.append(classifyTable[k][0])
        winRow.append(classifyTable[k])
        go = True
        ii = 0
        tempWin = classifyTable[k]
        # sum_nyx = -xlim
        while go:
            ii = ii + 1
            _, wn, _, nyx, fx, _ = findNearest(classifyTable, tempWin, xlim, ylim)
            if abs(wn[2]-tempWin[2])<ylim and (ii==1 or sign==((wn[1]-tempWin[1])>0)):
            # if abs(wn[2]-tempWin[2])<ylim and (nyx<sum_nyx or ii<=2):
                if tempWin in classifyTable: classifyTable.remove(tempWin)
                if ii==1: sign = (wn[1]-tempWin[1])>0
                tempWin = wn
                winRow.append(wn)
                indexBox.append(wn[0])
                # sum_nyx = sum_nyx + nyx
            else:
                go = False
                # winRow.sort(key=lambda x: x[1])
                windowMatrix.append(winRow)
                winRow = []

    classifyTable = []
    winTable = []
    windowMatrix.sort(key=lambda x: x[0][2])
    # numRow = len(windowMatrix)
    firstRow = windowMatrix[0]
    for i, m in enumerate(windowMatrix):
        for j, n in enumerate(m):
            n.append(i+1)
            # n.append(i-(int(numRow/2)))
            winTable.append(n)

    indexBox = []
    windowMatrix = []
    winCol = []
    classifyTable = winTable
    classifyTable.sort(key=lambda x: x[2])
    k = 0
    while k<numWin:
        if (len(indexBox)>=numWin) or k>=len(classifyTable):
            break
        if classifyTable[k][0] in indexBox:
            k = k+1
            continue
        else:
            indexBox.append(classifyTable[k][0])
        winCol.append(classifyTable[k])
        go = True
        ii = 0
        tempWin = classifyTable[k]
        # sum_nxy = -ylim
        while go:
            ii = ii + 1
            wn, _, nxy, _, _, fy = findNearest(classifyTable, tempWin, xlim, ylim)
            # if abs(wn[1]-tempWin[1])<xlim and (nxy<sum_nxy or ii<=2):
            if abs(wn[1]-tempWin[1])<xlim and (ii==1 or sign==((wn[2]-tempWin[2])>0)):
                if ii==1: sign = (wn[2]-tempWin[2])>0
                if tempWin in classifyTable: classifyTable.remove(tempWin)
                tempWin = wn
                winCol.append(wn)
                indexBox.append(wn[0])
                # sum_nxy = sum_nxy + nxy
            else:
                go = False
                # winCol.sort(key=lambda x: x[])
                windowMatrix.append(winCol)
                winCol = []
        if len(indexBox)==numWin:
            break

    winTable = []
    windowMatrix.sort(key=lambda x: x[0][1])
    numCol = len(windowMatrix)
    firstCol = windowMatrix[0]
    for j, m in enumerate(windowMatrix):
        for i, n in enumerate(m):
            n.append(j+1)
            # n.append(j-int(numCol/2))
            winTable.append(n)


    FRPL1 = False
    FRPL2 = False
    for wn in firstRow:
        if wn[2]<(ylim/2):
            FRPL2 = True
        if wn[2]<(1.75*(ylim/2)):
            FRPL1 = True

    if FRPL2:
        for wn in firstRow:
            winTable.remove(wn)
        for wn in winTable:
            wn[4] = wn[4]-1
        if frpl1 and not frpl2:
            print(3)
            if len(index)!=0: index[0] = index[0]-1
    elif frpl2 and not FRPL2:
        print(2)
        if FRPL1:
            print(4)
            if len(index)!=0: index[0] = index[0]+1

    FCPL1 = False
    FCPL2 = False
    for wn in firstCol:
        if wn[1]<(xlim/2):
            FCPL2 = True
        if wn[1]<(1.75*(xlim/2)):
            FCPL1 = True
    if FCPL2:
        for wn in firstCol:
            winTable.remove(wn)
        for wn in winTable:
            wn[5] = wn[5]-1
        if fcpl1 and not fcpl2:
            if len(index)!=0: index[1] = index[1]-1
    elif fcpl2 and not FCPL2:
        if FCPL1:
            if len(index)!=0: index[1] = index[1]+1

    frpl2 = FRPL2
    frpl1 = FRPL1
    fcpl2 = FCPL2
    fcpl1 = FCPL1

    for k, wn in enumerate(winTable):
        x = cv.putText(imdata, str([wn[4], wn[5]]),(wn[1]-10,wn[2]), cv.FONT_HERSHEY_SIMPLEX, 0.25, (0,255,0), lineType=cv.LINE_AA)
    cv.putText(imdata, str(it),(10,10), cv.FONT_HERSHEY_SIMPLEX, 0.25, (0,0,0), lineType=cv.LINE_AA)


    output = []
    if index_in:
        for wn in winTable:
            if wn[4]==input[0] and wn[5]==input[1]:
                # print('w output:', wn[3])
                output = wn
                # return wn[3]
        if len(output)==0:
            print('Wrong index to windowIndex!')
            return [1]
    elif window_in:
        for wn in winTable:
            if (wn[3]==input).all():
                print('i output:', wn[4], wn[5])
                xc = wn[1]
                yc = wn[2]
                return [wn[4], wn[5]]
        print('Wrong contour to windowIndex!')
        return [2]

    if len(output)>3:
        _,_, w, h = boundingBox(output[3])
        if h<50 : limh = 1.65*h
        else: limh = 1.25*h
        if w<50 : limw = 1.65*w
        else: limw = 1.25*w
        if abs(output[2]-yc)>(limh):
            print('row fix------------', h, output[2], yc, output[2]-yc)
            if (output[2]-yc)>0:
                index[0] = index[0]-1
            else:
                index[0] = index[0]+1
        if abs(output[1]-xc)>(limw):
            print('col fix------------', w, output[1], xc, output[1]-xc)
            if (output[1]-xc)>0:
                index[1] = index[1]-1
            else:
                index[1] = index[1]+1
        for wn in winTable:
            if wn[4]==index[0] and wn[5]==index[1]:
                output = wn

    return output[3]

def scoreContours(contours):
    print('scoreContours entered ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~')
    global tempArea, enterance, cv_image,  xc, yc
    rows, cols, _ = cv_image.shape
    goodIndex = ind = -1
    eucFIt = 0
    enteranceArea = 0.4*rows*cols
    polyInfo = []
    win = []
    for i, poly in enumerate(contours):
        area = cv.contourArea(poly)
        if len(poly)==1 or area<0.25*tempArea:
            continue
        case_x,case_y = contourCenter(poly)
        centerDist = euclideanDist([[xc,yc]], [[case_x,case_y]])
        polyInfo.append([i, centerDist, area, abs(area-tempArea)])
        # case_y, case_z = rectangleGeometric(poly)
        # polyScore = abs(area-tempArea)/tempArea+(2*centerDist/math.sqrt(tempArea))+(arDiff/shapeAR)
        # polyScore = centerDist
        # polyScore = centerDist/math.sqrt(tempArea)
        # polyInfo.append([i, polyScore, area, abs(area-tempArea), centerDist, case_y, case_z])
    m = len(polyInfo)
    polyInfo.sort(key=lambda x: x[1])
    print("polyInfo size:  ", len(polyInfo))
    # print("sorted polies:")
    # for p in polyInfo:
    #     # if p[1]>100: continue
    #     polyInfo_r = [float("{:.2f}".format(p[i])) for i in range(4)]
    #     print(polyInfo_r)
    print("area data:   before:", tempArea)

    if tempArea>=enteranceArea/3: maxAreaDiff = (4*tempArea)/10
    elif tempArea >= enteranceArea/9: maxAreaDiff = (3.5*tempArea)/10
    else: maxAreaDiff = (3*tempArea)/10
    i = 0
    while i<m :
        print("areaDiff", polyInfo[i][3])

        is_the_ca_const = (polyInfo[i][1]<(math.sqrt(tempArea)))
        is_the_area_const = polyInfo[i][3]<maxAreaDiff
        is_it_close_enough = (tempArea>=(0.05*enteranceArea))#and(tempArea<enteranceArea)
        is_it_the_full_frame = (polyInfo[i][2]>(cv_image.size-1e4))

        # print("is the ca cnst:", is_the_area_const, polyInfo[i][4], (math.sqrt(tempArea)))
        print("filters report :   ", is_the_ca_const, is_the_area_const, is_it_close_enough, is_it_the_full_frame)

        if (is_the_area_const or is_it_close_enough) and (not is_it_the_full_frame) and is_the_ca_const:
            goodIndex = polyInfo[i][0]
            ind = i
            print("---good poly info:   ", polyInfo[ind])
            print("area data:    before:", tempArea, "   new:   ", polyInfo[ind][2])
            # print(polies[goodIndex])
            win = contours[goodIndex]
            break
        i = i+1

    # if not is_the_ca_const:
        # eucFIt = eucFIt + 1
        # if goodIndex<m-1 : goodIndex = goodIndex + 1
    # else: eucFIt = 0
    if (goodIndex==-1) or (polyInfo[ind][1]>=100):
        # print("return -1")
        return [];
    print("the goodIndex:   ", goodIndex)

    if (is_the_area_const or is_it_close_enough):
        print("iter:\t", iter, "\tdata before:\t", tempvecy, tempvecz, "\tnew data:\t", vecy, vecz, "\teuc:\t", euc)
        if polyInfo[i][1]>(cols/5): return drawing, [];

        win = contours[goodIndex]
        # window_frame = cv_image.copy()
    return win;

def contourManagement(polies):
    global tempArea, flag, vecy, vecz, cv_image, window_points, index, xc, yc
    win = []
    enteranceArea = 2*cv_image.size/5
    scoring_approach = flag = draw_on = False
    img = cv_image.copy()
    rows, cols, _ = cv_image.shape
    areaRatio = tempArea/(rows*cols)
    # print(areaRatio)
    if len(index)==0 or areaRatio>=0.02:
        scoring_approach = True
    else:
        wi = windowIndex(polies, index)
        if len(wi)==4:
            is_sorted, wn = fourPSort(wi)
            if is_sorted:
                window_points = wn
                xc, yc = contourCenter(window_points)
                # tempvecy, tempvecz = contourCenter(window_points)
                # print('wi', window_points)
                window_points = np.array(window_points)
                tempArea = cv.contourArea(window_points)
                flag = draw_on = True
            else:
                scoring_approach = True
        # elif len(wi)==3:
            # The approximate case
            # move the last true window (window_points) to the approximate position and return it
            # approximate = True
        else:
            scoring_approach = True

    if scoring_approach:
        window_points = scoreContours(polies)
        if len(window_points)==4: flag = draw_on = True

    if draw_on:
        vecy, vecz = rectangleGeometric(window_points)
        if cv.contourArea(window_points)>(2*enteranceArea/3):
            print("enter!!")
            enterance = True
        cv.drawContours(img, [window_points], -1, (0,255,0), 2)
        for point in window_points:
            # point = [[x,y]]
            xp, yp = point[0][0], point[0][1]
            center = (xp,yp)
            cv.circle(img, center, 10, (0,0,0), 3)
        return img, window_points;
    # elif scoring_approach:
        # scoreContour(polies)
    return img, win

# def trackWindow(contours):
#     global window_points, window_frame, tr_it, cv_image
#     # print("---trackWindow entered---")
#     first_box = boundingBox(window_points)
#     # print("window_points", window_points)
#     # print("first_box", first_box)
#     # print("window_frame size", window_frame.size)
#     # if tr_it==0:
#     #     tracker.init(window_frame, first_box)
#     tracker.init(window_frame, first_box)
#     # tr_it = tr_it+1
#     success, box_new = tracker.update(cv_image)
#     if not success:
#         # print("---trackWindow failed---")
#         return cv_image, [];
#     xc = box_new[0]+(box_new[2]/2)
#     yc = box_new[1]+(box_new[3]/2)
#     bc = [[xc,yc]]
#     minDist = 1e7
#     for cnt in contours:
#         case_xc = 0
#         case_yc = 0
#         for pt in cnt:
#             case_xc = case_xc + pt[0][0]
#             case_yc = case_yc + pt[0][1]
#         case_xc = case_xc/len(cnt)
#         case_yc = case_yc/len(cnt)
#         case_c = [[case_xc, case_yc]]
#         box_dist = euclideanDist(bc,case_c)
#         if minDist>box_dist:
#             minDist = box_dist
#             win = cnt
#     tempvecy, tempvecz = rectangleGeometric(win)
#     wins = [win]
#     img = cv_image.copy()
#     cv.drawContours(img, wins, -1, (0,255,0), 2)
#     window_points = win
#     window_frame = img
#     flag = True
#     # print("---trackWindow succeeded---")
#     return img, win;

def perception3D(img, win):
    print("flag---p3d: ", flag)
    if not flag:
        return img
    global real_rect_info, translation_vec, rotation_vec
    axis_3d = np.float32([[0, 0, 0], [0.5, 0, 0], [0, 0.5, 0], [0, 0, 0.5]])
    is_sorted, rect = fourPSort(reconstructRect(win))
    if not is_sorted: return img
    found_rect_info = np.float32([rect[0][0], rect[1][0], rect[2][0], rect[3][0]])
    pic = img.copy()
    _, rotation_vec, translation_vec = cv.solvePnP(real_rect_info, found_rect_info, camera_matrix, distortion_matrix)
    axis_2d, j = cv.projectPoints(axis_3d, rotation_vec, translation_vec, camera_matrix, distortion_matrix)
    # print("a2d", axis_2d)
    start = (axis_2d[0][0][0], axis_2d[0][0][1])
    # print("start:  ", start)
    cv.line(pic, start, (axis_2d[1][0][0],axis_2d[1][0][1]), (0,255,255), 1)
    cv.line(pic, start, (axis_2d[2][0][0],axis_2d[2][0][1]), (255,0,255), 1)
    cv.line(pic, start, (axis_2d[3][0][0],axis_2d[3][0][1]), (255,255,0), 1)
    return pic

def windowDetection(imin):
    # print("\n~~~~~\nWD is called\n")
    global imdata
    imdata = imin.copy()
    imout = preProcessing(imin)
    poly = contourExtraction(imout, imdata)
    # windowIndex(poly,[1,1])
    # print(np.int32(poly))
    drawing, window = contourManagement(poly)
    # if not flaros
    #     drawing, window = trackWindow(poly)
    # else: tr_it = 0
    result = perception3D(drawing, window)
    cv.imshow("window detection result",result)
    # cv.imwrite('prob1.jpg', imdata)
    cv.imshow("window detection t",imdata)
    cv.waitKey(30)
    # print("\n~~~~~\nWD is out\n")
    return imout

class image_converter:

  def __init__(self):
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/quadrotor_1/front/image_raw",Image,self.imageCallback)
    # self.image_sub = rospy.Subscriber("/image_raw",Image,self.imageCallback)

  def imageCallback(self,data):
    global cv_image, is_set, frame_, msg, pub
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    # print("Iteration Begin\n\n\n")
    frame = cv_image.copy()
    # cv.imwrite('oopl.jpg', cv_image)

    # frame = windowDetection(cv_image)

    if is_set == False:
        # print("is_set",is_set)
        setWindow(frame)

    else:
        # print("is_set",is_set)
        cv.destroyWindow("operator desicion")
        cv.imshow("operator dsicion", frame_)
        frame = windowDetection(cv_image)
    # print("flag", flag)
    if flag:
        msg.y = vecy
        msg.z = vecz
        msg.enterance = enterance
        pub.publish(msg)
        # print("\n\nimpro published data:::\nmsg.y:",vecy,"\tmsg.z:\t",vecz)
        # print("translation_vec: ",translation_vec)
        # print("rotation_vec: ",rotation_vec)
    # else:
        # print("nothing published\n")
    # print("Iteration end\n---------------------------------------------------------------------------------\n")



def main(args):

  # print("Node Started ...\n")
  ic = image_converter()
  # ic = window_detection()
  pub = rospy.Publisher('chatter', String, queue_size=10)
  rospy.init_node('impro_p', anonymous=True)

  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
