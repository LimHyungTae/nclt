# !/usr/bin/python
#
# Demonstrates how to project velodyne points to camera imagery. Requires a binary
# velodyne sync file, undistorted image, and assumes that the calibration files are
# in the directory.
#
# To use:
#
#    python project_vel_to_cam.py vel img cam_num
#
#       vel:  The velodyne binary file (timestamp.bin)
#       img:  The undistorted image (timestamp.tiff)
#   cam_num:  The index (0 through 5) of the camera
#

import sys
import struct
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import numpy as np
import cv2
from nclt_tools.sensor_extrinsics import get_T_B_V, get_T_B_h30
# from undistort import *
cmap = plt.cm.jet
def colored_depthmap(depth, d_min=None, d_max=None):
    if d_min is None:
        d_min = np.min(depth)
    if d_max is None:
        d_max = np.max(depth)
    depth_relative = (depth - d_min) / (d_max - d_min)
    colored = 255 * cmap(depth_relative)[:, :, :3]  # H, W, C

    return colored

def convert(x_s, y_s, z_s):
    scaling = 0.005  # 5 mm
    offset = -100.0

    x = x_s * scaling + offset
    y = y_s * scaling + offset
    z = z_s * scaling + offset

    return x, y, z


def load_vel_hits(filename):
    f_bin = open(filename, "r")

    hits = []

    while True:

        x_str = f_bin.read(2)
        if x_str == '':  # eof
            break

        x = struct.unpack('<H', x_str)[0]
        y = struct.unpack('<H', f_bin.read(2))[0]
        z = struct.unpack('<H', f_bin.read(2))[0]
        i = struct.unpack('B', f_bin.read(1))[0]
        l = struct.unpack('B', f_bin.read(1))[0]

        x, y, z = convert(x, y, z)

        # Load in homogenous
        hits += [[x, y, z, 1]]

    f_bin.close()
    hits = np.asarray(hits)  # ($num, 4)
    return hits.transpose()  # (4, $num)


def ssc_to_homo(ssc):
    # Convert 6-DOF ssc coordinate transformation to 4x4 homogeneous matrix
    # transformation

    sr = np.sin(np.pi / 180.0 * ssc[3])
    cr = np.cos(np.pi / 180.0 * ssc[3])

    sp = np.sin(np.pi / 180.0 * ssc[4])
    cp = np.cos(np.pi / 180.0 * ssc[4])

    sh = np.sin(np.pi / 180.0 * ssc[5])
    ch = np.cos(np.pi / 180.0 * ssc[5])

    H = np.zeros((4, 4))

    H[0, 0] = ch * cp
    H[0, 1] = -sh * cr + ch * sp * sr
    H[0, 2] = sh * sr + ch * sp * cr
    H[1, 0] = sh * cp
    H[1, 1] = ch * cr + sh * sp * sr
    H[1, 2] = -ch * sr + sh * sp * cr
    H[2, 0] = -sp
    H[2, 1] = cp * sr
    H[2, 2] = cp * cr

    H[0, 3] = ssc[0]
    H[1, 3] = ssc[1]
    H[2, 3] = ssc[2]

    H[3, 3] = 1

    return H

def project_vel_to_cam(hits):
    T_lb3_c = ssc_to_homo(x_lb3_c)
    T_body_lb3 = ssc_to_homo(x_body_lb3)

    T_lb3_body = np.linalg.inv(T_body_lb3)
    T_c_lb3 = np.linalg.inv(T_lb3_c)

    T_c_body = np.matmul(T_c_lb3, T_lb3_body)

    hits_c = np.matmul(T_c_body, hits)
    hits_im = np.matmul(K, hits_c[0:3, :])

    return hits_im

def project_hokuyo_to_cam(hits):
    T_lb3_c = ssc_to_homo(x_lb3_c)
    T_body_lb3 = ssc_to_homo(x_body_lb3)

    T_lb3_body = np.linalg.inv(T_body_lb3)
    T_c_lb3 = np.linalg.inv(T_lb3_c)

    T_c_body = np.matmul(T_c_lb3, T_lb3_body)
    T_body_h = get_T_B_h30()
    T_c_h = np.matmul(T_c_body, T_body_h)
    hits_c = np.matmul(T_c_h, hits)
    hits_im = np.matmul(K, hits_c[0:3, :])

    return hits_im

def project_other_time_vel_to_cam(hits, t_vel, t_curr):
    T_lb3_c = ssc_to_homo(x_lb3_c)
    T_body_lb3 = ssc_to_homo(x_body_lb3)

    T_lb3_body = np.linalg.inv(T_body_lb3)
    T_c_lb3 = np.linalg.inv(T_lb3_c)

    T_c_body = np.matmul(T_c_lb3, T_lb3_body)

    # m: map
    T_m_c_curr = load_tf(t_curr)
    T_m_c = load_tf(t_curr)
    T_c_curr_c = np.matmul(np.linalg.inv(T_m_c_curr), T_m_c)
    T_c_curr_body=  np.matmul(T_c_curr_c, T_c_body)
    hits_c = np.matmul(T_c_curr_body, hits)
    # # #

    # hits_c = np.matmul(T_c_body, hits)
    hits_im = np.matmul(K, hits_c[0:3, :])

    return hits_im

def project_vels2cam(t_idx, viz=True):
    global CAMERA_IDX
    time = TIMES[t_idx]
    vel = get_name(time, "vel")
    hits_body = load_vel_hits(vel)
    hits_image = project_vel_to_cam(hits_body)

    criteria = [-3, -2, -1, 1, 2, 3]
    for criterion in criteria:
        t_prev = TIMES[t_idx + criterion]
        vel = get_name(t_prev, "vel")
        hits_body_prev = load_vel_hits(vel)
        hits_image_prev = project_other_time_vel_to_cam(hits_body_prev, t_prev, time)

        hits_image = np.concatenate((hits_image, hits_image_prev), axis=1)



    x_im = hits_image[0, :] / hits_image[2, :]
    y_im = hits_image[1, :] / hits_image[2, :]
    z_im = hits_image[2, :]

    idx_infront = z_im > 0
    x_im = x_im[idx_infront]
    y_im = y_im[idx_infront]
    z_im = z_im[idx_infront]

    img = get_name(time, "img")
    image = cv2.imread(img).astype(np.uint8)[:, :, ::-1]

    depth = np.zeros(image.shape[:2])
    for i in range(len(y_im)):

        y_pix = int(round(y_im[i]))
        x_pix = int(round(x_im[i]))
        if 0 <= y_pix and y_pix < depth.shape[0]:
            if 0 <= x_pix and x_pix < depth.shape[1]:
                depth[y_pix, x_pix] = z_im[i]
    viz = colored_depthmap(depth, 0, np.max(depth))
    # img_transpose = np.transpose(image, (1, 0, 2))
    # w = 912
    # h = 228
    # print(img_transpose.shape)
    # img_transpose = cv2.rectangle(img_transpose, (400, 200), (800, 400), (0, 0, 255), 2)
    # cv2.imwrite("result_img.png",  img_transpose)
    # cv2.imwrite("result_viz.png", np.transpose(viz, (1, 0, 2)))
    return 0

def project_hokuyo2cam(t_idx, viz=True):
    global CAMERA_IDX
    time = TIMES[t_idx]
    hits_body = load_hokuyo(time)
    hits_image = project_hokuyo_to_cam(hits_body)

    x_im = hits_image[0, :] / hits_image[2, :]
    y_im = hits_image[1, :] / hits_image[2, :]
    z_im = hits_image[2, :]

    idx_infront = z_im > 0
    x_im = x_im[idx_infront]
    y_im = y_im[idx_infront]
    z_im = z_im[idx_infront]

    img = get_name(time, "img")
    image = cv2.imread(img).astype(np.uint8)[:, :, ::-1]

    # depth = np.zeros(image.shape[:2])
    # for i in range(len(y_im)):
    #
    #     y_pix = int(round(y_im[i]))
    #     x_pix = int(round(x_im[i]))
    #     if 0 <= y_pix and y_pix < depth.shape[0]:
    #         if 0 <= x_pix and x_pix < depth.shape[1]:
    #             depth[y_pix, x_pix] = z_im[i]
    # viz = colored_depthmap(depth, 0, np.max(depth))
    plt.figure(1)
    plt.imshow(image)
    plt.hold(True)
    plt.scatter(x_im, y_im, c=z_im, s=5, linewidths=0)
    plt.xlim(0, 1616)
    plt.ylim(0, 1232)
    plt.show()
    # img_transpose = np.transpose(image, (1, 0, 2))
    # w = 912
    # h = 228
    # print(img_transpose.shape)
    # img_transpose = cv2.rectangle(img_transpose, (400, 200), (800, 400), (0, 0, 255), 2)
    # cv2.imwrite("result_img.png",  img_transpose)
    # cv2.imwrite("result_viz.png", np.transpose(viz, (1, 0, 2)))
    return 0

if __name__ == '__main__':
    from nclt_tools.CONFIG import *

    # project_vels2cam(1213)
    project_hokuyo2cam(2200)
    # project_vel2cam(3131)
    # project_vel2cam(20010)
