import os
from PIL import Image

# Set the input and output directories
input_dir = r"R:\Dataset\DTU - Drone inspection images of wind turbine\Nordtank 2018"
output_dir = r"C:\Users\mohsenaz\Desktop\Instant-NGP-for-GTX-1000\data\nerf\wind\images"

# Create the output directory if it doesn't exist
if not os.path.exists(output_dir):
    os.makedirs(output_dir)

# Loop through all the files in the input directory
for filename in os.listdir(input_dir):
    if filename.endswith(".JPG"):
        # Open the image file using PIL
        image_path = os.path.join(input_dir, filename)
        with Image.open(image_path) as img:
            # Resize the image to 20% of its original size
            new_size = (int(img.size[0] * 0.2), int(img.size[1] * 0.2))
            resized_img = img.resize(new_size)
            # Save the resized image to the output directory as .jpg
            output_path = os.path.join(output_dir, os.path.splitext(filename)[0] + ".jpg")
            resized_img.save(output_path)


# import pyrealsense2 as rs
# import numpy as np
# import cv2
#
# # Create a RealSense pipeline and start streaming
# pipeline = rs.pipeline()
# config = rs.config()
# config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
# config.enable_stream(rs.stream.color,  640, 480, rs.format.bgr8, 30)
# profile = pipeline.start(config)
#
# # Set up the bilateral filter
# bilateral_filter = cv2.bilateralFilter
#
# # Set up the weighted median filter
# weighted_median_filter = cv2.ximgproc.guidedFilter
#
# # Loop over frames from the camera
# while True:
#     # Wait for a frame from the camera
#     frames = pipeline.wait_for_frames()
#     depth_frame = frames.get_depth_frame()
#     color_frame = frames.get_color_frame()
#
#     # Convert depth and color frames to numpy arrays
#     depth_image = np.asanyarray(depth_frame.get_data())
#     color_image = np.asanyarray(color_frame.get_data())
#
#     # Apply bilateral filter to color image
#     smoothed_color = bilateral_filter(color_image, 5, 75, 75)
#
#     # Compute the gradient of the smoothed color image
#     grad_x = cv2.Sobel(smoothed_color, cv2.CV_64F, 1, 0, ksize=3)
#     grad_y = cv2.Sobel(smoothed_color, cv2.CV_64F, 0, 1, ksize=3)
#     grad_mag = cv2.magnitude(grad_x, grad_y)
#
#     # Normalize gradient values to [0, 1]
#     grad_norm = cv2.normalize(grad_mag, None, 0, 1, cv2.NORM_MINMAX)
#
#     # Apply weighted median filter to depth image using the normalized gradient as weights
#     denoised_depth = weighted_median_filter(depth_image.astype(np.float32), grad_norm.astype(np.float32), 9, 0.1)
#
#     # Convert denoised depth image back to uint16 format
#     denoised_depth = (denoised_depth * 65535).astype(np.uint16)
#
#
#     colorizedDepth = cv2.applyColorMap(cv2.convertScaleAbs(denoised_depth, alpha=0.03), cv2.COLORMAP_JET)
#
#     # Display the RGB and depth frames
#     cv2.imshow('RGB', color_image)
#     cv2.imshow('Depth', colorizedDepth)
#
#     # Exit on ESC key
#     if cv2.waitKey(1) == 27:
#         break
#
# # Stop the pipeline and close all windows
# pipeline.stop()
# cv2.destroyAllWindows()
