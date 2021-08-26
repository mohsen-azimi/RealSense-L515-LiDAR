import numpy as np
import pyrealsense2 as rs
import cv2

i = 0
try:
    config = rs.config()
    rs.config.enable_device_from_file(config, "2.bag", repeat_playback=False)
    pipeline = rs.pipeline()
    profile = pipeline.start(config)
    playback = profile.get_device().as_playback()
    playback.set_real_time(False)

    while True:
        frames = pipeline.wait_for_frames()
        # playback.pause()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        if not depth_frame:
            continue
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
        depth_color_map_2D = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
        cv2.imshow('Depth', depth_color_map_2D)
        # cv2.imshow('Color', color_image)

        # cv2.imwrite('Output/Depth/Depthimage' + str(i) + '.png', depth_image, [2 | 4])
        # cv2.imwrite('Output/Color/Colorimage' + str(i) + '.png', color_image)

        i += 1
        key = cv2.waitKey(30)
except RuntimeError:
    print("There are no more frames left in the .bag file!")

finally:
    pass