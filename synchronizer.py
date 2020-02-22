import os
import numpy as np
import re
import struct
import matplotlib.pyplot as plt
import copy
import scipy.interpolate

CUTOFF_EDGE = 30
WINDOW_SIZE = 9

def read_gt(gt_path):
    gt = np.loadtxt(gt_path, delimiter = ",")
    return gt

def convert(x_s):

    scaling = 0.005 # 5 mm
    offset = -100.0

    x = x_s * scaling + offset

    return x

def read_hokuyo(hokuyo_bin):
    # hokuyo_30m always has 1081 hits
    num_hits = 1081

    # angles for each range observation
    rad0 = -135 * (np.pi/180.0)
    radstep = 0.25 * (np.pi/180.0)
    angles = np.linspace(rad0, rad0 + (num_hits-1)*radstep, num_hits)

    f_bin = open(hokuyo_bin, "r")

    count = 0
    while True:
        # Read timestamp
        utime = struct.unpack('<Q', f_bin.read(8))[0]
        print 'Timestamp', utime
        r = np.zeros(num_hits)
        for i in range(num_hits):
            s = struct.unpack('<H', f_bin.read(2))[0]
            r[i] = convert(s)

        x = r * np.cos(angles)
        y = r * np.sin(angles)

    f_bin.close()


class Synchronizer(object):
    def __init__(self, gt_path, hokuyo_path, vel_dir, undistorted_dir):
        self.num_hits = 1081
        print("On preparing....")
        self.h30_bin = open(hokuyo_path, "r")
        self.gt = read_gt(gt_path)
        self.v_root = vel_dir
        self.img_root = undistorted_dir
        self.img_list = sorted(os.listdir(undistorted_dir))
        self.synced_times = []
        self.set_times()
        self.poses = []
        self.interp_pose()
        self.check_h30_time()
        # self.sync_vel_and_img()

    def set_times(self):
        img_names = copy.deepcopy(self.img_list[CUTOFF_EDGE:-CUTOFF_EDGE])
        for name in img_names:
            t_img = self.get_time(name)
            self.synced_times.append(t_img)
        self.synced_times = np.array(self.synced_times)

    def interp_pose(self):
        interp = scipy.interpolate.interp1d(gt[:, 0], gt[:, 1:], axis=0, fill_value="extrapolate")
        self.poses = interp(self.synced_times)

    def check_h30_time(self):
        for time in self.synced_times:
            self.set_h30(time)

    def set_h30(self, time):
        utime = struct.unpack('<Q', self.h30_bin.read(8))[0]
        if time > utime:
            while time > utime:
                utime = struct.unpack('<Q', self.h30_bin.read(8))[0]
            utime = struct.unpack('<Q', self.h30_bin.read(8))[0]
            print(time - utime)

    def fetch_synced_data(self, time):
        self.set_h30(self, time)



    def get_time(self, filename): # vel or img
        time = filename.split(".")[0]
        return int(time)

    def sync_vel_and_img(self):
        # For defensive programming....
        img_names = copy.deepcopy(self.img_list[CUTOFF_EDGE:-CUTOFF_EDGE])
        vel_names = copy.deepcopy(self.vel_list)
        count = 0
        while (len(img_names) != 0):
            count += 1
            if count % 1000 == 0:
                print(str(len(img_names)) + " left...")
            name = img_names.pop(0)
            t_img = self.get_time(name)
            for i, target in enumerate(vel_names):
                t_vel = self.get_time(target)
                if t_vel == t_img:
                    self.synced_times.append(t_img)
                    vel_names = vel_names[i + 1:]
                    break
        print("Total " + str(len(self.synced_times)) + " are synced!")

if __name__ == "__main__":
    root = "/home/joohyun/git"
    date = "2012-05-11"
    gt_path = os.path.join(root, "ground_truth", "groundtruth_" + date + ".csv")
    odom_cov_path = os.path.join(root, "ground_truth", "cov_" + date + ".csv")
    vel_dir = os.path.join(root, "velodyne_data", date, "velodyne_sync")
    undistorted_dir = os.path.join(root, "images", date, "lb3", "Undistorted-All", "Cam5")
    hokuyo_path = os.path.join(root, "hokuyo_data", date, "hokuyo_30m.bin")

    assert os.path.exists(gt_path)
    assert os.path.exists(odom_cov_path)
    assert os.path.exists(vel_dir)
    assert os.path.exists(undistorted_dir)
    assert os.path.exists(hokuyo_path)

    vel_list = os.listdir(vel_dir)
    img_list = os.listdir(undistorted_dir)

    gt = read_gt(gt_path) # time , x, y, z, roll, pitch, yaw?
    in_there = 0
    no_there = 0
    gt_times = gt[:-1, 0]
    gt_times2 = gt[1:, 0]
    print(gt_times.shape)

    sync = Synchronizer(gt_path, hokuyo_path, vel_dir, undistorted_dir)
