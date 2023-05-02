"""
Author: Mohsen Azimi
Date: September 01 2021
"""
import time
import cv2
from camera import L515
import pyrealsense2 as rs
import numpy as np
import open3d as o3d


camera = L515(read_bag=0, record_bag=0)
while True:
    f = camera.get_frame()
    color_image = f.color_image
    depth_image = f.depth_image
    ir_image = f.ir_image
    accel = f.accel
    gyro = f.gyro

    # print(gyro)
    pcd = f.point_cloud



    o3d.visualization.draw_geometries([pcd])

    if cv2.waitKey(1) & 0xff == 27:  # 27 = ESC
        break

# Stop streaming
camera.pipeline.stop()
