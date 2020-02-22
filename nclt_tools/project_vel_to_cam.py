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
from sensor_extrinsics import get_T_B_V
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
    hits = np.asarray(hits)

    return hits.transpose()


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


def project_vel_to_cam(hits, cam_num):
    # Load camera parameters
    K = np.loadtxt('../params/K_cam%d.csv' % (cam_num), delimiter=',')
    x_lb3_c = np.loadtxt('../params/x_lb3_c%d.csv' % (cam_num), delimiter=',')

    # Other coordinate transforms we need
    x_body_lb3 = [0.035, 0.002, -1.23, -179.93, -0.23, 0.50]

    # Now do the projection
    T_lb3_c = ssc_to_homo(x_lb3_c)
    T_body_lb3 = ssc_to_homo(x_body_lb3)

    T_lb3_body = np.linalg.inv(T_body_lb3)
    T_c_lb3 = np.linalg.inv(T_lb3_c)

    T_c_body = np.matmul(T_c_lb3, T_lb3_body)

    hits_c = np.matmul(T_c_body, hits)
    hits_im = np.matmul(K, hits_c[0:3, :])
    print(hits_im.shape)
    return hits_im

def project_vel2cam(vel, img, index):
    hits_body = load_vel_hits(vel)
    print("hehe", hits_body.shape)

    # Load image
    # image = mpimg.imread(img)

    from undistort import Undistort
    map_path = "/home/joohyun/git/nclt/U2D_Cam5_1616X1232.txt"
    undistort = Undistort(map_path)
    im = cv2.imread(cam)
    # cv2.namedWindow('Image', cv2.WINDOW_NORMAL)
    # cv2.imshow('Image', im)
    image = undistort.undistort(im)[:, :, ::-1]
    cam_num = int(index)

    hits_image = project_vel_to_cam(hits_body, cam_num)

    x_im = hits_image[0, :] / hits_image[2, :]
    y_im = hits_image[1, :] / hits_image[2, :]
    z_im = hits_image[2, :]

    idx_infront = z_im > 0
    x_im = x_im[idx_infront]
    y_im = y_im[idx_infront]
    z_im = z_im[idx_infront]

    plt.figure(1)
    plt.imshow(image)
    plt.hold(True)
    plt.scatter(x_im, y_im, c=z_im, s=5, linewidths=0)
    plt.xlim(0, 1616)
    plt.ylim(0, 1232)
    plt.show()

    return 0

def project_vel2cam_cv2(vel, img, index):
    hits_body = load_vel_hits(vel)
    print("hehe", hits_body.shape)

    # Load image
    # image = mpimg.imread(img)

    from undistort import Undistort
    map_path = "/home/joohyun/git/nclt/U2D_Cam5_1616X1232.txt"
    undistort = Undistort(map_path)
    im = cv2.imread(cam)
    # cv2.namedWindow('Image', cv2.WINDOW_NORMAL)
    # cv2.imshow('Image', im)
    image = undistort.undistort(im)[:, :, ::-1]
    cam_num = int(index)

    hits_image = project_vel_to_cam(hits_body, cam_num)

    x_im = hits_image[0, :] / hits_image[2, :]
    y_im = hits_image[1, :] / hits_image[2, :]
    z_im = hits_image[2, :]

    idx_infront = z_im > 0
    x_im = x_im[idx_infront]
    y_im = y_im[idx_infront]
    z_im = z_im[idx_infront]

    plt.figure(1)
    depth = np.zeros(image.shape[:2])
    for i in range(len(y_im)):

        y_pix = int(round(y_im[i]))
        x_pix = int(round(x_im[i]))
        if 0 <= y_pix and y_pix < depth.shape[0]:
            if 0 <= x_pix and x_pix < depth.shape[1]:
                depth[y_pix, x_pix] = z_im[i]
    viz = colored_depthmap(depth, np.min(depth), np.max(depth))
    # cv2.imshow("viz", viz)
    # cv2.waitKey(0)
    cv2.imwrite("viz_t.png", viz)
    # plt.imshow(image)
    # plt.hold(True)
    # plt.scatter(x_im, y_im, c=z_im, s=5, linewidths=0)
    # plt.xlim(0, 1616)
    # plt.ylim(0, 1232)
    # plt.show()

    return 0

if __name__ == '__main__':
    # vel = "/home/joohyun/git/velodyne_data/2012-05-11/velodyne_sync/1336759411637320.bin"
    vel = "/home/joohyun/git/velodyne_data/2012-05-11/velodyne_sync/1336759309638372.bin"
    cam = "/home/joohyun/git/images/2012-05-11/lb3/Cam5/1336759309638372.tiff"
    index = 5
    project_vel2cam_cv2(vel, cam, index)
