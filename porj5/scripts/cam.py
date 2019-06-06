#!/usr/bin/env python2
#coding:utf-8　

from __future__ import print_function
import cv2
import numpy as np
import math
from math import pi
from math import sin
from math import cos
from math import tan
import copy
import pyrealsense2 as rs 

defaultwidth = 1280
defaultheight = 720 

# Setup:
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

for iter in range(100):
    #get frame
    for _ in range(20):
        frames = pipeline.wait_for_frames()

    frames = pipeline.wait_for_frames()
    aligned_frames = align.process(frames)
    aligned_depth_frame = aligned_frames.get_depth_frame()
    color_frame = aligned_frames.get_color_frame()

    depth_image = np.asanyarray(aligned_depth_frame.get_data())
    img = np.asanyarray(color_frame.get_data())

    #cv2.imwrite("yzy/2_Color.png", img)

    cv2.imshow("org",img)

    cv2.waitKey(0)
    
    cv2.imwrite('yzy/color_image_%02d.png' % iter, img)    
