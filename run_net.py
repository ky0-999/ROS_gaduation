#! /usr/bin/env python3

import time

import numpy as np
from numpy.lib.twodim_base import mask_indices
from scipy.ndimage import filters
import torch
import torchvision
from PIL import Image
from torch.nn import functional as F
from torch.utils.data import DataLoader, Dataset
from torchvision import transforms
from torchvision.datasets.utils import download_url
import os
import sys

sys.path.append('/home/ericlab/shiba/ros_ws/src')
from gr_convnet.inference.models.grconvnet_origin import GenerativeResnet
# from gr_convnet.inference.models.graduation import GenerativeResnet
# from gr_convnet.inference.models.graduation2 import GenerativeResnet
import cv2
import scipy.ndimage as ndimage
from skimage.draw import circle
from skimage.feature import peak_local_max

import rospy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Float32MultiArray

from timeit import TimeIt

bridge = CvBridge()

def get_device(use_gpu):
    if use_gpu and torch.cuda.is_available():
        return torch.device("cuda")
    else:
        return torch.device("cpu")

def normalize_depth(depth):
    depth_min = depth.min()
    depth_max = depth.max()
    max_val = 2
    if depth_max - depth_min > np.finfo("float").eps:
        out = max_val * (depth - depth_min) / (depth_max - depth_min) -1
    else:
        out = np.zeros(depth.shape, dtype=depth.type)
    return out

def process_depth_image(depth, crop_size, out_size=224, return_mask=False, crop_y_offset=0):
    imh, imw = depth.shape

    with TimeIt('1'):
        # Crop.
        depth_crop = depth[(imh - crop_size) // 2 - crop_y_offset:(imh - crop_size) // 2 + crop_size - crop_y_offset,
                           (imw - crop_size) // 2:(imw - crop_size) // 2 + crop_size]
        
    # depth_nan_mask = np.isnan(depth_crop).astype(np.uint8)

    # Inpaint
    # OpenCV inpainting does weird things at the border.
    with TimeIt('2'):
        depth_crop = cv2.copyMakeBorder(depth_crop, 1, 1, 1, 1, cv2.BORDER_DEFAULT)
        depth_nan_mask = np.isnan(depth_crop).astype(np.uint8)

    with TimeIt('3'):
        depth_crop[depth_nan_mask==1] = 0

    with TimeIt('4'):
        # Scale to keep as float, but has to be in bounds -1:1 to keep opencv happy.
        depth_scale = np.abs(depth_crop).max()
        depth_crop = depth_crop.astype(np.float32) / depth_scale  # Has to be float32, 64 not supported.

        with TimeIt('Inpainting'):
            depth_crop = cv2.inpaint(depth_crop, depth_nan_mask, 1, cv2.INPAINT_NS)

        # Back to original size and value range.
        depth_crop = depth_crop[1:-1, 1:-1]
        depth_crop = depth_crop * depth_scale

    with TimeIt('5'):
        # Resize
        depth_crop = cv2.resize(depth_crop, (out_size, out_size), cv2.INTER_AREA)

    if return_mask:
        with TimeIt('6'):
            depth_nan_mask = depth_nan_mask[1:-1, 1:-1]
            depth_nan_mask = cv2.resize(depth_nan_mask, (out_size, out_size), cv2.INTER_NEAREST)
        return depth_crop, depth_nan_mask
    else:
        return depth_crop

def predict(depth, process_depth=True, crop_size=224, out_size=224, depth_nan_mask=None, crop_y_offset=0, filters=(2.0, 1.0, 1.0)):
    if process_depth:
        depth, depth_nan_mask = process_depth_image(depth, crop_size, out_size=out_size, return_mask=True, crop_y_offset=crop_y_offset)

    # Inference
    depth = np.clip((depth - depth.mean()), -1, 1)
    depthT = torch.from_numpy(depth.reshape(1, 1, out_size, out_size).astype(np.float32)).to(device)
    with torch.no_grad():
        pred_out = model(depthT)

    points_out = pred_out[0].cpu().numpy().squeeze()
    points_out[depth_nan_mask] = 0

    # Calculate the angle map.
    cos_out = pred_out[1].cpu().numpy().squeeze()
    sin_out = pred_out[2].cpu().numpy().squeeze()
    ang_out = np.arctan2(sin_out, cos_out) / 2.0

    width_out = pred_out[3].cpu().numpy().squeeze() * 150.0  # Scaled 0-150:0-1

    # Filter the outputs.
    if filters[0]:
        points_out = ndimage.filters.gaussian_filter(points_out, filters[0])  # 3.0
    if filters[1]:
        ang_out = ndimage.filters.gaussian_filter(ang_out, filters[1])
    if filters[2]:
        width_out = ndimage.filters.gaussian_filter(width_out, filters[2])

    points_out = np.clip(points_out, 0.0, 1.0-1e-3)

    # SM
    # temp = 0.15
    # ep = np.exp(points_out / temp)
    # points_out = ep / ep.sum()

    # points_out = (points_out - points_out.min())/(points_out.max() - points_out.min())

    return points_out, ang_out, width_out, depth.squeeze()

device = get_device(use_gpu=True)
device2 = torch.device('cpu')
# Load the Network
# model = torch.jit.load("/home/ericlab/graduation_paper/src/ggcnn/hourglass_net_dense_OW_epoch99_scripted.pt").eval().cuda()
# model = GGCNN2()
# model.load_state_dict(torch.load('/home/ericlab/graduation_paper/src/ggcnn/output/models/211206_1356_ggcnn2_OW/epoch_57_iou_0.90_statedict.pt'))
# model = model.eval().cuda()


model = GenerativeResnet()
model.load_state_dict(torch.load('/home/ericlab/shiba/ros_ws/src/gr_convnet/logs/OW_gr-convenet_origin_100/epoch_96_iou_0.91.pt'))
# model.load_state_dict(torch.load('/home/ericlab/shiba/ros_ws/src/gr_convnet/logs/gra1/epoch_97_iou_0.91.pt'))
# model.load_state_dict(torch.load('/home/ericlab/shiba/ros_ws/src/gr_convnet/logs/grad2/epoch_85_iou_0.91.pt'))

model = model.eval().cuda()
print('Load the Network')



rospy.init_node('ggcnn_detection')

# Output publishers.
print('Output publishers.')
grasp_pub = rospy.Publisher('ggcnn/img/grasp', Image, queue_size=1)
grasp_plain_pub = rospy.Publisher('ggcnn/img/grasp_plain', Image, queue_size=1)
depth_pub = rospy.Publisher('ggcnn/img/depth', Image, queue_size=1)
ang_pub = rospy.Publisher('ggcnn/img/ang', Image, queue_size=1)
cmd_pub = rospy.Publisher('ggcnn/out/command', Float32MultiArray, queue_size=1)

# Inirialise some globals.
print('Initialise some globals.')
prev_mp = np.array([150, 150])

# Get the camera parameters
print('Get the camera parameters.')
camera_info_msg = rospy.wait_for_message('/camera/depth/camera_info', CameraInfo)
K = camera_info_msg.K
fx = K[0]
cx = K[2]
fy = K[4]
cy = K[5]

def depth_callback(depth_message):
    global prev_mp
    global fx, cx, fy, cy

    with TimeIt('Predict'):
        depth = bridge.imgmsg_to_cv2(depth_message)
        crop_size = 300
        crop_offset = 0
        out_size = 300
        # crop_size = 224
        # crop_offset = 0
        # out_size = 224

    # with TimeIt('pre'):
        points_out, ang_out, width_out, depth_crop = predict(depth, crop_size=crop_size, out_size=out_size, crop_y_offset=crop_offset, filters=(2.0, 2.0, 2.0))

    with TimeIt('Calculate Depth'):
        depth_center = depth_crop[100:141, 130:171].flatten()
        depth_center.sort()
        depth_center = depth_center[:10].mean() * 1000.0

    with TimeIt('Control'):
        # Calculate the best pose from the camera intrinsics.
        maxes = None
        ALWAYS_MAX = True  # Use ALWAYS_MAX = True for the open-loop solution.

        if ALWAYS_MAX:
            # Track the global max.
            max_pixel = np.array(np.unravel_index(np.argmax(points_out), points_out.shape))
            prev_mp = max_pixel.astype(np.int)
        else:
            # Calculate a set of local maxes.  Choose the one that is closes to the previous one.
            maxes = peak_local_max(points_out, min_distance=10, threshold_abs=0.1, num_peaks=3)
            if maxes.shape[0] == 0:
                rospy.logerr('No Local Maxes')
                return
            max_pixel = maxes[np.argmin(np.linalg.norm(maxes - prev_mp, axis=1))]

            # Keep a global copy for next iteration.
            prev_mp = (max_pixel * 0.25 + prev_mp * 0.75).astype(np.int)

        ang = ang_out[max_pixel[0], max_pixel[1]]
        width = width_out[max_pixel[0], max_pixel[1]]

        # Convert max_pixel back to uncropped/resized image coordinates in order to do the camera transform.
        max_pixel = ((np.array(max_pixel) / out_size * crop_size) + np.array([(480 - crop_size)//2 - crop_offset, (640 - crop_size) // 2]))
        max_pixel = np.round(max_pixel).astype(np.int)

        point_depth = depth[max_pixel[0], max_pixel[1]]
        # Compute the actual position.
        x = (max_pixel[1] - cx)/(fx) * point_depth
        y = (max_pixel[0] - cy)/(fy) * point_depth
        z = point_depth

        if np.isnan(z):
            return

    with TimeIt('Draw'):
        # Draw grasp markers on the points_out and publish it. (for visualisation)
        grasp_img = cv2.applyColorMap((points_out * 255).astype(np.uint8), cv2.COLORMAP_JET)
        grasp_img_plain = grasp_img.copy()

        rr, cc = circle(prev_mp[0], prev_mp[1], 5)
        grasp_img[rr, cc, 0] = 0
        grasp_img[rr, cc, 1] = 255
        grasp_img[rr, cc, 2] = 0

    with TimeIt('Publish'):
        # Publish the output images (not used for control, only visualisation)
        grasp_img = bridge.cv2_to_imgmsg(grasp_img, 'bgr8')
        grasp_img.header = depth_message.header
        grasp_pub.publish(grasp_img)

        grasp_img_plain = bridge.cv2_to_imgmsg(grasp_img_plain, 'bgr8')
        grasp_img_plain.header = depth_message.header
        grasp_plain_pub.publish(grasp_img_plain)

        depth_pub.publish(bridge.cv2_to_imgmsg(depth_crop))

        ang_pub.publish(bridge.cv2_to_imgmsg(ang_out))

        # Output the best grasp pose relative to camera.
        cmd_msg = Float32MultiArray()
        cmd_msg.data = [x, y, z, ang, width, depth_center]
        if not (x == 0 and y == 0 and z == 0):
            cmd_pub.publish(cmd_msg)

# depth_sub = rospy.Subscriber('/camera/depth/image_rect_raw', Image, depth_callback, queue_size = 1)
depth_sub = rospy.Subscriber('/camera/depth/image_meters', Image, depth_callback, queue_size = 1)




while not rospy.is_shutdown():
    rospy.spin()