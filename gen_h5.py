import h5py
import os
import numpy as np
import cv2
import sys

def kaist_h5_loader(path):
    h5f = h5py.File(path, "r")
    rgb = np.array(h5f['rgb'])
    rgb = np.transpose(rgb, (1, 2, 0)) # height, width, 3
    depth = np.array(h5f['depth'])
    scan_projected = np.array(h5f['scan_projected'])
    scan_raw = np.array(h5f['scan_raw'])[::-1, :] # [::-1, :] change left-> to right
    scan_raw = np.cos(scan_raw[:, 0]) * scan_raw[:, 1] -0.0369

    # scan_raw = scan_raw
    return rgb, depth, scan_projected, scan_raw

def nclt_h5_loader(path):
    h5f = h5py.File(path, "r")
    rgb = np.array(h5f['rgb'])
    rgb = np.transpose(rgb, (1, 2, 0)) # height, width, 3
    depth = np.array(h5f['depth'])
    scan_projected = np.array(h5f['scan_projected'])


    return rgb, depth, scan_projected


def generate_nclt_h5(path, img, gt, scan_proj):

    hf = h5py.File(path, 'w')
    # Make it reverse is for mathing the format of sparse-to-depth prediction code
    hf.create_dataset("rgb", data=np.transpose(img, (2, 0, 1)))
    hf.create_dataset("scan_projected", data=scan_proj)
    hf.create_dataset("depth", data=gt)

    hf.close()

import matplotlib.pyplot as plt
cmap = plt.cm.jet
def colored_depthmap(depth, d_min=None, d_max=None):
    if d_min is None:
        d_min = np.min(depth)
    if d_max is None:
        d_max = np.max(depth)
    depth_relative = (depth - d_min) / (d_max - d_min)
    colored = 255 * cmap(depth_relative)[:, :, :3]  # H, W, C

    return colored
if __name__ == "__main__":
    # rgb, depth, scan_proj, _ = kaist_h5_loader("/home/joohyun/Downloads/0060.h5")
    rgb, depth, scan_proj = nclt_h5_loader("nclt_tools/results/02608.h5")

    cv2.imshow("test", rgb)
    cv2.imshow("test1", colored_depthmap(depth, 0, 30))
    cv2.imshow("test2", colored_depthmap(scan_proj, 0, 30))
    cv2.waitKey(0)
