import cv2
import pyrealsense2 as rs
import numpy as np
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

fourcc = cv2.cv.CV_FOURCC(*'XVID')
out = cv2.VideoWriter('output.avi',fourcc, 20.0, (1280,720))
while True:
    frames = pipeline.wait_for_frames()
    aligned_frames = align.process(frames)
    color_frame = aligned_frames.get_color_frame()
    img = np.asanyarray(color_frame.get_data())
    cv2.imshow('fig', img)
    out.write(img)
    if cv2.waitKey(1) & 0xff == ord('q'):
        break
out.release()