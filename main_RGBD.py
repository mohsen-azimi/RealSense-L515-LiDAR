"""
Author: Mohsen Azimi
Date: September 01 2021
"""
import time
import cv2 as cv
from camera import L515
import numpy as np
import open3d as o3d


point = (400, 300)
def mouse_coord(event, x, y, args, params):
    global point
    point = (x, y)


# Streaming loop
frame_count = 0

try:
    # Initialize Camera
    camera = L515(enable_imu=False)

    frame_count = 0
    start_time = time.time()
    frame_time = start_time

    while True:

        # maintain frame timing
        frame_count += 14
        last_time = frame_time
        frame_time = time.time()

        # read camera data
        frame = camera.get_frame()
        color_image = frame.color_image
        depth_image = frame.depth_image
        ir_image = frame.ir_image
        pc = frame.point_cloud

        # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
        depth_image_colorised = cv.applyColorMap(cv.convertScaleAbs(depth_image, alpha=.03), cv.COLORMAP_JET)

        # # Flip image ?
        color_image = cv.flip(color_image, 1)
        depth_image_colorised = cv.flip(depth_image_colorised, 1)
        infrared = cv.flip(infrared, 1)

        # # depth_frame = decimate.process(depth_frame)
        # # Grab new intrinsics (may be changed by decimation)
        # depth_intrinsics = rs.video_stream_profile(
        #     depth_frame.profile).get_intrinsics()
        # w, h = depth_intrinsics.width, depth_intrinsics.height
        #
        # depth_scale = camera.get_depth_scale()


        # if enable_imu and not profile_frames:
        #     print("imu frame {} in {} seconds: \n\taccel = {}, \n\tgyro = {}".format(
        #         str(frame_count),
        #         str(frame_time - last_time),
        #         str((acceleration_x, acceleration_y, acceleration_z)),
        #         str((gyroscope_x, gyroscope_y, gyroscope_z))))

        if camera.enable_rgbd:

            cv.namedWindow('Color', cv.WINDOW_AUTOSIZE)

            cv.namedWindow('Depth', cv.WINDOW_AUTOSIZE)
            cv.namedWindow('IR', cv.WINDOW_AUTOSIZE)
            cv.imshow('IR', infrared)


            cv.setMouseCallback('Depth', mouse_coord)  # to show distance on mouse
            # Show distance for a specific point
            cv.circle(depth_image_colorised, point, 5, (0, 0, 255))
            distance = depth_image[point[1], point[0]] * camera.depth_scale

            cv.putText(depth_image_colorised, f'{distance:.3f} m', (point[0], point[1] - 20), cv.FONT_HERSHEY_PLAIN, 2,
                       (0, 255, 255), 4)

            cv.imshow('Color', color_image)
            cv.imshow('Depth', depth_image_colorised)

            # # show pc


        if cv.waitKey(1) & 0xff == 27:  # 27 = ESC
            break

finally:
    camera.pipeline.stop()
