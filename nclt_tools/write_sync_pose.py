import numpy
import os
import numpy as np
import re
import struct
import matplotlib.pyplot as plt
import copy
import scipy.interpolate
from nclt_tools.transformation_helpers import getTransformationFromEulerAnglesRollPitchYawRadXYZMeters
import csv


def read_gt(gt_path):
    gt = np.loadtxt(gt_path, delimiter=",")
    return gt


if __name__ == "__main__":
    from nclt_tools.CONFIG import *
    gt = read_gt(GT_PATH)
    print("On interpolating...")
    interp = scipy.interpolate.interp1d(gt[:, 0], gt[:, 1:], axis=0, fill_value="extrapolate")

    if len(TIMES) == 0:
        raise RuntimeError

    poses = interp(sorted(TIMES))
    print("Done")
    if not os.path.exists(TF_DIR):
        os.mkdir(TF_DIR)
    for i, time in enumerate(TIMES):
        x_m = poses[i, 0]
        y_m = poses[i, 1]
        z_m = poses[i, 2]
        roll_rad = poses[i, 3]
        pitch_rad = poses[i, 4]
        yaw_rad = poses[i, 5]
        tf = getTransformationFromEulerAnglesRollPitchYawRadXYZMeters(roll_rad, pitch_rad, yaw_rad,
                                                                          x_m, y_m, z_m)
        filename = str(time) + ".csv"
        with open(os.path.join(TF_DIR, filename), 'w') as f:
            writer = csv.writer(f)
            writer.writerow(tf[0, :])
            writer.writerow(tf[1, :])
            writer.writerow(tf[2, :])
