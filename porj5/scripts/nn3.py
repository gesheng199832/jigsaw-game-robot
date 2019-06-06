from __future__ import print_function
import numpy as np
import warnings
warnings.simplefilter('ignore', np.RankWarning)

from math import sin, cos, pi
import cv2
import pyrealsense2 as rs 
import copy
import time
# setup realsense

points = rs.points()
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
profile = pipeline.start(config)
depth_sensor = profile.get_device().first_depth_sensor()
depth_scale = depth_sensor.get_depth_scale()
align_to = rs.stream.color
align = rs.align(align_to)

for _ in range(10):
    frames = pipeline.wait_for_frames()

kernel = np.ones((7,7),np.uint8)
train_list = []
label_list = []
count = 0
while count < 1000:

    frames = pipeline.wait_for_frames()
    aligned_frames = align.process(frames)
    color_frame = aligned_frames.get_color_frame()
    img = np.asanyarray(color_frame.get_data())
    #print(img.shape) (720, 1280, 3)
    img = img[30:720/2+100, 1280/3:1280*2/3]
    raw_img = copy.copy(img)
    #cv2.imshow('img', img)
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    
    # green
    upper_green = np.array([60+10, 255, 255])
    lower_green = np.array([60-10, 60, 60])

    mask = cv2.inRange(hsv, lower_green, upper_green)

    mask = cv2.bitwise_not(mask, mask)

    closing = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    # cv2.imshow('closing', closing)

    contours, hierarchy = cv2.findContours(closing,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    # select base board
    # select each piece
    pice_cnt = []
    base_cnt = []
    for i in range(len(contours)):
        cnt = contours[i]
        area = cv2.contourArea(cnt)
        if area < 1500 and area > 15000: # is piece or base ?
            continue
        elif 1700 < area < 2500: # pice
            pice_cnt = pice_cnt+[cnt]
            M = cv2.moments(cnt)
            if M['m00'] != 0 :
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])
                # cv2.circle(img,(cx,cy), 10, (0,0,255), 2)
            # cv2.putText(img,str(area),(cx,cy), cv2.FONT_HERSHEY_SIMPLEX, 1,(255,0,255),2, 2)
            rect = cv2.minAreaRect(cnt)
            lx, ly = rect[0]
            roi = raw_img[cy-35:cy+35, cx-35:cx+35]
            try:
                cv2.imshow('roi', roi)
            except:
                pass

        elif 10000 < area < 15000: # base
            base_cnt = base_cnt+[cnt]
            M = cv2.moments(cnt)
            if M['m00'] != 0 :
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])
                
            #cv2.putText(img,str(area),(cx,cy), cv2.FONT_HERSHEY_SIMPLEX, 1,(255,0,255),2, 2)
            rect = cv2.minAreaRect(cnt)
            #cv2.circle(img,(int(rect[0][0]),int(rect[0][1])), 10, (0,0,255), 2)
            #cv2.putText(img,str(rect[2]),(cx,cy), cv2.FONT_HERSHEY_SIMPLEX, 0.5,(255,0,255),1, 2)
            rect = cv2.minAreaRect(cnt)
            lx, ly = rect[0]
            w, h = rect[1]
            r = rect[2]
            m = np.array([[cos(r*pi/180),-sin(r*pi/180)],[sin(r*pi/180),cos(r*pi/180)]])
            a = np.array([0-43,0-43])
            b = np.array([0+43,0+43])
            c = np.array([0+43,0-43])
            d = np.array([0-43,0+43])
            a = m.dot(a.T) + np.array([cx, cy])
            b = m.dot(b.T) + np.array([cx, cy])
            c = m.dot(c.T) + np.array([cx, cy])
            d = m.dot(d.T) + np.array([cx, cy])
            cv2.line(img,(int(a[0]), int(a[1])),(int(b[0]), int(b[1])),(0,0,255),2)
            cv2.line(img,(int(c[0]), int(c[1])),(int(d[0]), int(d[1])),(0,0,255),2)
            cv2.circle(img,(int(a[0]), int(a[1])), 2, (255,0,0), 2)
            cv2.putText(img,str(1),(int(a[0]), int(a[1])), cv2.FONT_HERSHEY_SIMPLEX, 0.5,(255,0,255),1, 2)
            cv2.circle(img,(int(b[0]), int(b[1])), 2, (255,0,0), 2)
            cv2.putText(img,str(2),(int(b[0]), int(b[1])), cv2.FONT_HERSHEY_SIMPLEX, 0.5,(255,0,255),1, 2)
            cv2.circle(img,(int(c[0]), int(c[1])), 2, (255,0,0), 2)
            cv2.putText(img,str(3),(int(c[0]), int(c[1])), cv2.FONT_HERSHEY_SIMPLEX, 0.5,(255,0,255),1, 2)
            cv2.circle(img,(int(d[0]), int(d[1])), 2, (255,0,0), 2)
            cv2.putText(img,str(4),(int(d[0]), int(d[1])), cv2.FONT_HERSHEY_SIMPLEX, 0.5,(255,0,255),1, 2)
    
    
    cv2.drawContours(img, pice_cnt, -1, (255,0,0), 1)
    cv2.drawContours(img, base_cnt, -1, (0,0,255), 2)
    cv2.circle(img,(200, 200), 2, (0,0,255), 2)
    cv2.imshow('img', img)
    #time.sleep(float(1/24))
    
    if cv2.waitKey(1) & 0xff == ord('q'):
        break
print(roi.shape)
np.load('1.npy', roi)