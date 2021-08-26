"""
Author: Mohsen Azimi
Date: September 01 2021
The pyrealsense2 API can be found in https://intelrealsense.github.io/librealsense/python_docs/_generated/pyrealsense2.html.

"""
import time
import logging
import numpy as np
import pyrealsense2 as rs
import yaml


class Frame(object):
    def __init__(self, color_frame, color_image, depth_frame, depth_image, ir_frame, ir_image, point_cloud, acceleration, gyroscope):
        self.color_frame = color_frame
        self.color_image = color_image

        self.depth_frame = depth_frame
        self.depth_image = depth_image

        self.ir_frame = ir_frame
        self.ir_image = ir_image

        self.point_cloud = point_cloud
        self.acceleration = acceleration
        self.gyroscope = gyroscope


class L515:
    def __init__(self, enable_rgbd=True, enable_imu=False,
                 record_bag=None, read_bag=None):
        print("connecting to camera...")

        self.enable_imu = enable_imu
        self.enable_rgbd = enable_rgbd

        self.record_bag = record_bag
        self.read_bag = read_bag

        self._cfg = yaml.load(open('config\\camera_rs.yaml', 'r'), Loader=yaml.FullLoader)
        self.imu_fps = self._cfg['imu']['fps']
        self.rgb_fps = self._cfg['rgb']['fps']
        self.rgb_res = (self._cfg['rgb']['width'], self._cfg['rgb']['height'])
        self.depth_ir_res = (self._cfg['depth_infrared']['width'], self._cfg['depth_infrared']['height'])

        # Configure streams
        self.points = rs.points()
        self.pipeline = rs.pipeline()
        config = rs.config()

        # Get device product line for setting a supporting resolution
        pipeline_profile = config.resolve(rs.pipeline_wrapper(self.pipeline))
        device = pipeline_profile.get_device()
        # device_product_line = str(device.get_info(rs.camera_info.product_line))
        for i, s in enumerate(device.sensors):
            print(f"Sensor({i}): ", s.get_info(rs.camera_info.name))

        if self.enable_rgbd:
            config.enable_stream(rs.stream.color, self.rgb_res[0], self.rgb_res[1], rs.format.bgr8, self.rgb_fps)
            config.enable_stream(rs.stream.depth, self.depth_ir_res[0], self.depth_ir_res[1], rs.format.z16, self.rgb_fps)
            config.enable_stream(rs.stream.infrared, self.depth_ir_res[0], self.depth_ir_res[1], rs.format.y8, self.rgb_fps)
        #     self.config.enable_stream(rs.stream.confidence, CAM_WIDTH, CAM_HEIGHT, rs.format.raw8, CAM_FPS)

        if self.enable_imu:
            config.enable_stream(rs.stream.accel, rs.format.motion_xyz32f, self.imu_fps)  # acceleration
            config.enable_stream(rs.stream.gyro, rs.format.motion_xyz32f, self.imu_fps)  # gyroscope
            config.enable_stream(rs.stream.pose, rs.format.six_dof, self.imu_fps)  #

        if self.read_bag:
            print("Reading a bag file...")
            # config.enable_device_from_file(self.read_bag)
            config.enable_device_from_file(self.read_bag, repeat_playback=False)
        elif self.record_bag:
            print("Recording a bag file...")
            config.enable_record_to_file(self.record_bag)
        else:
            pass

        # Start streaming
        self.profile = self.pipeline.start(config)

        # Getting the depth sensor's depth scale (see rs-align example for explanation)
        if self.enable_rgbd:
            depth_sensor = self.profile.get_device().first_depth_sensor()
            self.depth_scale = depth_sensor.get_depth_scale()
            # Create an align object
            # rs.align allows us to perform alignment of depth frames to others frames
            # The "align_to" is the stream type to which we plan to align depth frames.
            align_to = rs.stream.color
            self.align = rs.align(align_to)

        # # eat some frames to allow auto-exposure to settle
        # for i in range(0, 5):
        #     self.pipeline.wait_for_frames()


    def get_frame(self):
        # Get frameset of color and depth
        frames = self.pipeline.wait_for_frames()
        # frames.get_depth_frame() is a 640x360 depth image

        # Align the depth frame to color frame
        aligned_frames = self.align.process(frames)

        # Get aligned frames
        aligned_depth_frame = aligned_frames.get_depth_frame() # aligned_depth_frame is a 640x480 depth image

        color_frame = aligned_frames.get_color_frame()
        depth_frame = aligned_frames.get_depth_frame()
        ir_frame = aligned_frames.get_infrared_frame()

        color_image = np.asanyarray(color_frame.get_data())
        depth_image = np.asanyarray(depth_frame.get_data())
        ir_image = np.asanyarray(ir_frame.get_data())

        # depth_image = np.asanyarray(aligned_depth_frame.get_data())  # , dtype=np.uint16)
        # depth_image = depth_image * self.get_depth_scale()
        # color_image = np.asanyarray(color_frame.get_data())   # , dtype=np.uint8)

        pc = rs.pointcloud()
        point_cloud = pc.calculate(aligned_depth_frame)

        if self.enable_imu:
            # for frame in frames:
            #     accel_data = frame.as_motion_frame().get_motion_data()
            #     gyro_data = frame.as_motion_frame().get_motion_data()
            #     accel = np.asarray([accel_data.x, accel_data.y, accel_data.z])
            #     gyro = np.asarray([gyro_data.x, gyro_data.y, gyro_data.z])

            accel = aligned_frames.first_or_default(rs.stream.accel, rs.format.motion_xyz32f).as_motion_frame().get_motion_data()
            acceleration = [accel.x, accel.y, accel.z]

            gyro = aligned_frames.first_or_default(rs.stream.gyro, rs.format.motion_xyz32f).as_motion_frame().get_motion_data()
            gyroscope = [gyro.x, gyro.y, gyro.z]


            # pose = aligned_frames.get_pose_frame()  # for tracking
            # if pose:
            #     # Print some of the pose data to the terminal
            #     data = pose.get_pose_data()
            #     print("Frame #{}".format(pose.frame_number))
            #     print("Position: {}".format(data.translation))
            #     global x, y
            #     x = (x + data.translation.z) / 1000
            #     y = (y + (-data.translation.x)) / 1000
            #     x = round(x, 4)
            #     y = round(y, 4)

            # {{logging.debug("imu frame {} in {} seconds: \n\taccel = {}, \n\tgyro = {}".format(
            #     str(self.frame_count),
            #     str(self.frame_time - last_time),
            #     str((self.acceleration_x, self.acceleration_y, self.acceleration_z)),
            # str((self.gyroscope_x, self.gyroscope_y, self.gyroscope_z))))}}
            return Frame(color_frame, color_image, depth_frame, depth_image, ir_frame, ir_image, point_cloud,  acceleration, gyroscope)

        else:
            return Frame(color_frame, color_image, depth_frame, depth_image, ir_frame, ir_image, point_cloud,  None, None)

    """
    @Description: get intrinsics attributes of a camera
    @parameters[in]: None
    @return:
        intrinsics.fx: focal length of the image in width(columns)
        intrinsics.fy: focal length of the image in height(rows)
        intrinsics.ppx: the pixel coordinates of the principal point (center of projection) in width
        intrinsics.ppy: the pixel coordinates of the principal point (center of projection) in height
    """

    def clip_distance(self, depth_image, distance, depth):
        clipping_distance_in_meters = distance  # 1 meter
        depth_scale = self.get_depth_scale()
        clipping_distance = clipping_distance_in_meters / depth_scale

        grey_color = 0
        depth_image_3d = np.dstack((depth_image, depth_image, depth_image))
        bg_removed = np.where((depth_image_3d > clipping_distance) | (depth_image_3d <= 0), grey_color, color_image)

        return bg_removed

    def get_w_h(self):
        # Get stream profile and camera intrinsics
        profile = self.pipeline.get_active_profile()
        depth_profile = rs.video_stream_profile(profile.get_stream(rs.stream.depth))
        depth_intrinsics = depth_profile.get_intrinsics()
        w, h = depth_intrinsics.width, depth_intrinsics.height

        return w, h

    # def get_gyro_data(gyro):
    #     return np.asarray([gyro.x, gyro.y, gyro.z])
    #
    # def get_accel_data(accel):
    #     return np.asarray([accel.x, accel.y, accel.z])

# self test
#
# if __name__ == "__main__":
#
#     show_opencv_window = True  # True to show images in opencv window: note that default donkeycar environment is not configured for this.
#     if show_opencv_window:
#         import cv2
#
#     enable_rgb = True
#     enable_depth = True
#     enable_imu = True
#     device_id = None
#
#     width = 212
#     height = 120
#     channels = 3
#
#     profile_frames = 0  # set to non-zero to calculate the max frame rate using given number of frames
#
#     try:
#         #
#         # for D435i, enable_imu can be True, for D435 enable_imu should be false
#         #
#         camera = L515(
#             width=width, height=height, channels=channels,
#             enable_rgb=enable_rgb, enable_depth=enable_depth, enable_imu=enable_imu, device_id=device_id)
#
#         frame_count = 0
#         start_time = time.time()
#         frame_time = start_time
#         while True:
#             #
#             # read data from camera
#             #
#             color_image, depth_image, depth_frame, acceleration_x, acceleration_y, acceleration_z, gyroscope_x, gyroscope_y, gyroscope_z = camera.run()
#
#             # maintain frame timing
#             frame_count += 1
#             last_time = frame_time
#             frame_time = time.time()
#
#             if enable_imu and not profile_frames:
#                 print("imu frame {} in {} seconds: \n\taccel = {}, \n\tgyro = {}".format(
#                     str(frame_count),
#                     str(frame_time - last_time),
#                     str((acceleration_x, acceleration_y, acceleration_z)),
#                     str((gyroscope_x, gyroscope_y, gyroscope_z))))
#
#             # Show images
#             if show_opencv_window and not profile_frames:
#                 cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
#                 if enable_rgb or enable_depth:
#                     # make sure depth and color images have same number of channels so we can show them together in the window
#                     if 3 == channels:
#                         depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03),
#                                                            cv2.COLORMAP_JET) if enable_depth else None
#                     else:
#                         depth_colormap = cv2.cvtColor(
#                             cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET),
#                             cv2.COLOR_RGB2GRAY) if enable_depth else None
#
#                     # Stack both images horizontally
#                     images = None
#                     if enable_rgb:
#                         images = np.hstack((color_image, depth_colormap)) if enable_depth else color_image
#                     elif enable_depth:
#                         images = depth_colormap
#
#                     if images is not None:
#                         cv2.imshow('RealSense', images)
#
#                 # Press esc or 'q' to close the image window
#                 key = cv2.waitKey(1)
#                 if key & 0xFF == ord('q') or key == 27:
#                     cv2.destroyAllWindows()
#                     break
#             if profile_frames > 0:
#                 if frame_count == profile_frames:
#                     print("Aquired {} frames in {} seconds for {} fps".format(str(frame_count),
#                                                                               str(frame_time - start_time),
#                                                                               str(frame_count / (
#                                                                                       frame_time - start_time))))
#                     break
#             else:
#                 time.sleep(0.05)
#     finally:
#         camera.shutdown()
