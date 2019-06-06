#!/usr/bin/env python2
#coding:utf-8　
from __future__ import print_function
import numpy as np
import warnings
warnings.simplefilter('ignore', np.RankWarning)
import tensorflow as tf
from math import sin, cos, pi
import cv2
import pyrealsense2 as rs
import copy
import time
import rospy

from porj5.srv import get_state
from porj5.srv import get_stateResponse
# define functions

def res2ang(res):
    if res == 0 or res == 4 or res == 8 or res == 12:
        ang_bias = 360
    elif res == 1 or res == 5 or res == 9 or res == 13:
        ang_bias = 90
    elif res == 2 or res == 6 or res == 10 or res == 14:
        ang_bias = 270
    elif res == 3 or res == 7 or res == 11 or res == 15:
        ang_bias = 180
    
    if res == 0 or res == 1 or res == 2 or res == 3:
        index = 0
    elif res == 4 or res == 5 or res == 6 or res == 7:
        index = 1
    elif res == 8 or res == 9 or res == 10 or res == 11:
        index = 2
    elif res == 12 or res == 13 or res == 14 or res == 15:
        index = 3
    return index, ang_bias

def handle_get_state(req):
    req = req
    global pub_list_
    return get_stateResponse(pub_list_)

# define nn models
model1 = tf.keras.Sequential([
    tf.keras.layers.Flatten(input_shape=(60, 60, 3)),
    tf.keras.layers.Dense(512, activation=tf.nn.relu),
    tf.keras.layers.Dropout(0.2),
    tf.keras.layers.Dense(128, activation=tf.nn.relu),
    tf.keras.layers.Dropout(0.2),
    tf.keras.layers.Dense(2, activation=tf.nn.softmax)
])
model1.compile(optimizer='adam',
                loss='sparse_categorical_crossentropy',
                metrics=['accuracy'])

model1.load_weights('weights')

model2 = tf.keras.Sequential([
    tf.keras.layers.Flatten(input_shape=(60, 60, 3)),
    tf.keras.layers.Dense(512, activation=tf.nn.relu),
    tf.keras.layers.Dropout(0.2),
    tf.keras.layers.Dense(128, activation=tf.nn.relu),
    tf.keras.layers.Dropout(0.2),
    tf.keras.layers.Dense(16, activation=tf.nn.softmax)
])
model2.compile(optimizer='adam',
                loss='sparse_categorical_crossentropy',
                metrics=['accuracy'])

model2.load_weights('0522_weights')

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

# ros settings
rospy.init_node('get_state_server')
s = rospy.Service('get_state', get_state, handle_get_state)

# dont touch !
kernel = np.ones((7,7),np.uint8)
img_list = []
label_list = []
l = 32
piece_cx_buff = np.zeros((4, 10))
piece_cy_buff = np.zeros((4, 10))
piece_ang_buff = np.zeros((4, 10))
base_cx_buff = np.zeros(10)
base_cy_buff = np.zeros(10)
while True:

    frames = pipeline.wait_for_frames()
    aligned_frames = align.process(frames)
    color_frame = aligned_frames.get_color_frame()
    img = np.asanyarray(color_frame.get_data())

    img = img[30:720/2+100, 1280/3:(1280*2/3 + 3)] # (430, 430) 30,426
    raw_img = copy.copy(img)

    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    
    # green
    upper_green = np.array([60+10, 255, 255])
    lower_green = np.array([60-10, 60, 60])

    mask = cv2.inRange(hsv, lower_green, upper_green)

    mask = cv2.bitwise_not(mask, mask)

    closing = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    contours, hierarchy = cv2.findContours(closing,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

    pice_cnt = []
    base_cnt = []
    my_pice_cornors = np.zeros((4,3))
    base_pice_cornors = np.zeros((4,3))
    base_pice_point = np.zeros((2, 2))
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
                        
            rect = cv2.minAreaRect(cnt)
            lx, ly = rect[0]
            w, h = rect[1]
            r = rect[2]
            m = np.array([[cos(r*pi/180),-sin(r*pi/180)],[sin(r*pi/180),cos(r*pi/180)]])
            inv = np.array([[cos((r+45)*pi/180),-sin((r+45)*pi/180)],[sin((r+45)*pi/180),cos((r+45)*pi/180)]])
            a = np.array([0-30,0-30])
            b = np.array([0+30,0+30])
            c = np.array([0+30,0-30])
            d = np.array([0-30,0+30])
            a = m.dot(a.T) + np.array([cx, cy])
            b = m.dot(b.T) + np.array([cx, cy])
            c = m.dot(c.T) + np.array([cx, cy])
            d = m.dot(d.T) + np.array([cx, cy])

            pts = np.array([[int(a[0]),int(a[1])],[int(c[0]),int(c[1])],[int(b[0]),int(b[1])],[int(d[0]),int(d[1])]], np.int32)
            pts = pts.reshape((-1,1,2))
            cv2.polylines(img,[pts],True,(0,255,255))
            
            rot_M = cv2.getRotationMatrix2D((cx,cy),r,1)
            dst = cv2.warpAffine(raw_img,rot_M,(1280,720))
            roi_r = dst[cy-30:cy+30, cx-30:cx+30]

            try:
                res = model1.predict(np.array([roi_r]))
            except:
                res = [[1, 0]]
                pass
            
            if res[0][1] == 1:
                clr = (0,0,255)
                res = model2.predict(np.array([roi_r]))
                res = np.argmax(res)
                index, angle_bias = res2ang(res)
                # cv2.putText(img,str(index),(cx,cy), cv2.FONT_HERSHEY_SIMPLEX, 0.5,(255,0,255),1, 2)
                l_sin = l*sin((r+angle_bias-135)*pi/180)
                l_cos = l*cos((r+angle_bias-135)*pi/180)
                cv2.line(img, (int(cx+l*cos((r+angle_bias-135)*pi/180)),int(cy+l*sin((r+angle_bias-135)*pi/180))),
                    (int(cx+l*cos((r+angle_bias-45)*pi/180)),int(cy+l*sin((r+angle_bias-45)*pi/180))),(0, 0, 255), 3)
                cv2.line(img, (int(cx+l*cos((r+angle_bias-135)*pi/180)),int(cy+l*sin((r+angle_bias-135)*pi/180))),
                    (int(cx+l*cos((r+angle_bias-225)*pi/180)),int(cy+l*sin((r+angle_bias-225)*pi/180))),(255, 0, 0), 3)
                buff1 = piece_cx_buff[index]
                buff2 = piece_cy_buff[index]
                buff1 = buff1.tolist()
                buff2 = buff2.tolist()
                buff1.pop(0)
                buff1 = buff1 + [cx]
                buff2.pop(0)
                buff2 = buff2 + [cy]
                buff1 = np.array(buff1)
                buff2 = np.array(buff2)
                piece_cx_buff[index] = buff1
                piece_cy_buff[index] = buff2
                my_pice_cornors[index, 0] = buff1.mean()# int(cx+l*cos((r+angle_bias-135)*pi/180))
                my_pice_cornors[index, 1] = buff2.mean()# int(cy+l*sin((r+angle_bias-135)*pi/180))

                buff1 = piece_ang_buff[index]
                buff1 = buff1.tolist()
                buff1.pop(0)
                buff1 = buff1 + [r+angle_bias]
                buff1 = np.array(buff1)
                piece_ang_buff[index] = buff1

                my_pice_cornors[index, 2] = buff1.mean()

                
            else:
                clr = (255,0,0)
            cv2.polylines(img,[pts],True,clr)
            
        elif 10000 < area < 15000: # base
            base_cnt = base_cnt+[cnt]
            M = cv2.moments(cnt)
            if M['m00'] != 0 :
                base_cx = int(M['m10']/M['m00'])
                base_cy = int(M['m01']/M['m00'])
            rect = cv2.minAreaRect(cnt)
            lx, ly = rect[0]
            w, h = rect[1]
            r = rect[2]
            if abs(w - h) < 5: 
                m = np.array([[cos(r*pi/180),-sin(r*pi/180)],[sin(r*pi/180),cos(r*pi/180)]])
                a = np.array([0-50/2,0-50/2])
                b = np.array([0+50/2,0+50/2])
                c = np.array([0+50/2,0-50/2])
                d = np.array([0-50/2,0+50/2])
                a = m.dot(a.T) + np.array([base_cx, base_cy])
                b = m.dot(b.T) + np.array([base_cx, base_cy])
                c = m.dot(c.T) + np.array([base_cx, base_cy])
                d = m.dot(d.T) + np.array([base_cx, base_cy])
                base_pice_point = np.array([(c+b)/2, (b+d)/2])
                cv2.line(img,(int(a[0]), int(a[1])),(int(b[0]), int(b[1])),(0,0,255),2)
                cv2.line(img,(int(c[0]), int(c[1])),(int(d[0]), int(d[1])),(0,0,255),2)

                cv2.circle(img,(int(a[0]), int(a[1])), 2, (255,0,0), 2)
                #cv2.putText(img,str(0),(int(a[0]), int(a[1])), cv2.FONT_HERSHEY_SIMPLEX, 0.5,(255,0,255),1, 2)
                base_pice_cornors[0, 0] = int(a[0])
                base_pice_cornors[0, 1] = int(a[1])
                base_pice_cornors[0, 2] = r

                cv2.circle(img,(int(b[0]), int(b[1])), 2, (255,0,0), 2)
                #cv2.putText(img,str(3),(int(b[0]), int(b[1])), cv2.FONT_HERSHEY_SIMPLEX, 0.5,(255,0,255),1, 2)
                base_pice_cornors[3, 0] = int(b[0])
                base_pice_cornors[3, 1] = int(b[1])
                base_pice_cornors[3, 2] = r

                cv2.circle(img,(int(c[0]), int(c[1])), 2, (255,0,0), 2)
                #cv2.putText(img,str(1),(int(c[0]), int(c[1])), cv2.FONT_HERSHEY_SIMPLEX, 0.5,(255,0,255),1, 2)
                base_pice_cornors[1, 0] = int(c[0])
                base_pice_cornors[1, 1] = int(c[1])
                base_pice_cornors[1, 2] = r

                cv2.circle(img,(int(d[0]), int(d[1])), 2, (255,0,0), 2)
                #cv2.putText(img,str(2),(int(d[0]), int(d[1])), cv2.FONT_HERSHEY_SIMPLEX, 0.5,(255,0,255),1, 2)
                base_pice_cornors[2, 0] = int(d[0])
                base_pice_cornors[2, 1] = int(d[1])
                base_pice_cornors[2, 2] = r
                #cv2.putText(img,str(int(r)),(cx, cy), cv2.FONT_HERSHEY_SIMPLEX, 0.5,(255,0,255),1, 2)
    
    pub_list = [0]*24
    for i in range(4):
        if my_pice_cornors[i,0] != 0 and my_pice_cornors[i,1] != 0 and base_pice_cornors[i,0]!=0 and base_pice_cornors[i,1]!=0:
            cv2.line(img, (int(my_pice_cornors[i,0]), int(my_pice_cornors[i,1])), (int(base_pice_cornors[i,0]), int(base_pice_cornors[i,1])), (255, 0, 0), 2)
            if i == 0: 
                ang = my_pice_cornors[i,2] - base_pice_cornors[i, 2]
                base_pice_cornors[i,0] = base_cx
                base_pice_cornors[i,1] = base_cy
            if i == 1: 
                ang = my_pice_cornors[i,2] - base_pice_cornors[i, 2] + 270
                loc = base_pice_point[0]
                base_pice_cornors[i,0] = loc[0]
                base_pice_cornors[i,1] = loc[1]
            if i == 2: 
                ang = my_pice_cornors[i,2] - base_pice_cornors[i, 2] - 270
                d = np.array([0-30,0+30])
                d = m.dot(d.T) + np.array([base_cx, base_cy])
                base_pice_cornors[i, 0] = d[0] 
                base_pice_cornors[i, 1] = d[1] 
            if i == 3: 
                ang = my_pice_cornors[i,2] - base_pice_cornors[i, 2] - 180
                loc = base_pice_point[1]
                base_pice_cornors[i,0] = loc[0]
                base_pice_cornors[i,1] = loc[1]
            while ang > 360 : ang = ang - 360
            while ang < 0: ang = ang + 360
            cv2.putText(img,str(int(ang)),(int(my_pice_cornors[i,0]), int(my_pice_cornors[i,1])), cv2.FONT_HERSHEY_SIMPLEX, 0.5,(255,0,255),1, 2)
            pub_list[i*6] = my_pice_cornors[i,0]
            pub_list[i*6+1] = my_pice_cornors[i,1]
            pub_list[i*6+2] = base_pice_cornors[i,0]
            pub_list[i*6+3] = base_pice_cornors[i,1]
            pub_list[i*6+4] = my_pice_cornors[i, 2]
            pub_list[i*6+5] = base_pice_cornors[i, 2]
    pub_list_ = pub_list
    cv2.imshow('img', img)
    if cv2.waitKey(1) & 0xff == ord('q'):
        break
while not rospy.is_shutdown():
    rospy.spin()

