"""
Author: Mohsen Azimi
Date: September 01 2021
"""
import cv2
import numpy as np

from camera import L515
from collections import deque
import pywt
def wavelet_filter(depth_buffer):
    depth_avg = np.mean(depth_buffer, axis=0)

    # Apply wavelet decomposition to the depth image
    coeffs2 = pywt.dwt2(depth_avg, 'haar')
    cA, (cH, cV, cD) = coeffs2

    # Threshold the detail coefficients
    cH[np.abs(cH) < 10] = 0
    cV[np.abs(cV) < 10] = 0
    cD[np.abs(cD) < 10] = 0

    # Reconstruct the filtered depth image
    coeffs2_filtered = (cA, (cH, cV, cD))
    depth_filtered = pywt.idwt2(coeffs2_filtered, 'haar')

    return depth_filtered

# Define the callback function for the sliders
def set_clipping_distance(val):
    global dist_min, dist_max
    dist_min = cv2.getTrackbarPos("Min Distance", "Clipping Distance Selector")
    dist_max = cv2.getTrackbarPos("Max Distance", "Clipping Distance Selector")



def mouse_over_callback(event, x, y, flags, param):
    global depth_value
    global x1, y1, x2, y2

    if event == cv2.EVENT_LBUTTONDOWN:
        x1, y1 = x, y
        depth_value = depth_image[y, x] * camera.depth_scale
        text = f"({y}, {x}, {depth_value:.2f})"
        cv2.putText(color_image, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
        cv2.imshow("Color", color_image)
        print(f"Distance: {text}")

    if event == cv2.EVENT_RBUTTONDOWN:
        x2, y2 = x, y
        depth_value = depth_image[y, x] * camera.depth_scale
        text = f"({y}, {x}, {depth_value:.2f})"
        cv2.putText(color_image, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
        cv2.imshow("Color", color_image)
        print(f"Distance: {text}")

        # crop image
        cropped_image = color_image[y1:y2, x1:x2]
        # crop mask
        cropped_mask = mask_clipped[y1:y2, x1:x2]
        # save image and mask
        cv2.imwrite(f"masks/cropped_image_{i}.png", cropped_image)
        cv2.imwrite(f"masks/cropped_mask_{i}.png", cropped_mask)
        print("Image and Mask Saved")





def update_min_max_distance(event, x, y, flags, params):
    global dist_min
    global dist_max


    if event == cv2.EVENT_LBUTTONDOWN:  # Left click: set min distance
        depth_value = depth_image[y, x] * camera.depth_scale

        dist_min = depth_value * 100
        # max = min + 2 cm
        dist_max = dist_min + 2
        print(f"Min Distance: {dist_min}")

    elif event == cv2.EVENT_RBUTTONDOWN:  # Right click: set max distance
        depth_value = depth_image[y, x] * camera.depth_scale + 0.001 #  offset
        dist_max = depth_value * 100
        print(f"Max Distance: {dist_max}")

    elif event == cv2.EVENT_MBUTTONDOWN:  # Middle click: reset distances to default
        dist_min = 0
        dist_max = 1000
        print("Distances reset to default values")











    # Initialize the clipping distances
dist_min = 79
dist_max = 80
# Create a window to display the sliders
# cv2.namedWindow("Clipping Distance Selector", cv2.WINDOW_NORMAL)

# cv2.createTrackbar("Min Distance", "Clipping Distance Selector", dist_min, 500, set_clipping_distance)
# cv2.createTrackbar("Max Distance", "Clipping Distance Selector", dist_max, 500, set_clipping_distance)

cv2.namedWindow("Clipped Depth Image")
cv2.setMouseCallback("Clipped Depth Image", update_min_max_distance)

cv2.namedWindow("Color")
cv2.setMouseCallback("Color", mouse_over_callback)



# Initialize Camera
camera = L515(read_bag=0, record_bag=0)
i = 0


# define buffer size
BUFFER_SIZE = 10
# create buffer using deque
depth_buffer = deque(maxlen=BUFFER_SIZE)


while True:
    # This call waits until a new coherent set of frames is available on a device maintain frame timing

    # read camera data
    f = camera.get_frame()
    color_image = f.color_image
    depth_image = f.depth_image


    # add current depth frame to buffer
    depth_buffer.append(depth_image)

    # compute moving average over last BUFFER_SIZE frames
    if len(depth_buffer) == BUFFER_SIZE:
        # depth_avg = sum(depth_buffer) / BUFFER_SIZE
        depth_avg = np.median(depth_buffer, axis=0)
        # depth_avg = np.mean(depth_buffer, axis=0)
        # depth_avg = wavelet_filter(depth_buffer)
        # depth_avg = depth_image



        # depth_avg = cv2.GaussianBlur(depth_avg, ksize=(5, 5), sigmaX=2, sigmaY=2)

    else:
        depth_avg = depth_image



    # Clip the depth image based on the selected minimum and maximum distances
    depth_clipped, mask_clipped = camera.clip_distance(depth_avg, color_image, dist_min/100, dist_max/100)


    # # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
    depth_image_ave_colorised = cv2.applyColorMap(cv2.convertScaleAbs(depth_avg, alpha=.03), cv2.COLORMAP_JET)

    # # Flip image ?
    # color_image = cv.flip(color_image, 1)
    # depth_image_colorised = cv.flip(depth_image_colorised, 1)
    # infrared = cv.flip(infrared, 1)

    if camera.enable_rgbd:
        # cv.namedWindow('Color', cv.WINDOW_AUTOSIZE)
        # cv.namedWindow('Depth', cv.WINDOW_AUTOSIZE)
        # cv.namedWindow('depth_clipped', cv.WINDOW_AUTOSIZE)
        # cv.namedWindow('mask_clipped', cv.WINDOW_AUTOSIZE)

        # cv.namedWindow('IR', cv.WINDOW_AUTOSIZE)
        # cv.imshow('IR', infrared)

        # cv.setMouseCallback('Depth', mouse_coord)  # to show distance on mouse
        # # Show distance for a specific point
        # cv.circle(depth_image_colorised, point, 5, (0, 0, 255))
        # distance = depth_image[point[1], point[0]] * camera.depth_scale
        #
        # cv.putText(depth_image_colorised, f'{distance:.3f} m', (point[0], point[1] - 20), cv.FONT_HERSHEY_PLAIN, 2,
        #            (0, 255, 255), 4)

        cv2.imshow('Color', color_image)
        cv2.imshow('Depth', depth_image_ave_colorised)
        cv2.imshow('Clipped Depth Image', depth_clipped)



        # update the min_dist and max_dist by clicking + dragging on the depth_clipped image (mouse callback)


        #




        # if i % 100 == 0:
        #     cv.imwrite(f'./masks/rgb{i}.jpg', color_image)
        #     cv.imwrite(f'./masks/depth_{i}.png', depth_image_colorised)
        #
        #     cv.imwrite(f'./masks/mask{i}_0.png', mask_clipped_0)
        #     cv.imwrite(f'./masks/mask{i}_1.png', mask_clipped_1)
        #     cv.imwrite(f'./masks/mask{i}_2.png', mask_clipped_2)
        #
        #     break # remove this
        i += 1

        # print(i)


    if cv2.waitKey(1) & 0xff == 27:  # 27 = ESC
        break

# Stop streaming
camera.pipeline.stop()
