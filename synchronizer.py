import os
import numpy as np
import re
import struct
import matplotlib.pyplot as plt
import copy
import scipy.interpolate
from nclt_tools.transformation_helpers import getTransformationFromEulerAnglesRollPitchYawRadXYZMeters
from nclt_tools.project_vel_to_cam import *
from nclt_tools.CONFIG import *
CUTOFF_EDGE = 30
WINDOW_SIZE = 9

cmap = plt.cm.jet


def colored_depthmap(depth, d_min=None, d_max=None):
    if d_min is None:
        d_min = np.min(depth)
    if d_max is None:
        d_max = np.max(depth)
    depth_relative = (depth - d_min) / (d_max - d_min)
    colored = 255 * cmap(depth_relative)[:, :, :3]  # H, W, C

    return colored



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
    def __init__(self):
        self.num_hits = 1081
        cam_num = 5
        self.K = np.loadtxt('params/K_cam%d.csv' % (cam_num), delimiter=',')
        self.x_lb3_c = np.loadtxt('params/x_lb3_c%d.csv' % (cam_num), delimiter=',')
        self.x_body_lb3 = [0.035, 0.002, -1.23, -179.93, -0.23, 0.50]
        print("On preparing....")
        # self.h30_bin = open(hokuyo_path, "r")

    def project_vel2cam_cv2(vel, img):
        global CAMERA_IDX
        hits_body = load_vel_hits(vel)
        print("hehe", hits_body.shape)

        # Load image
        # image = mpimg.imread(img)

        image = cv2.imread(img).astype(np.uint8)
        # cv2.namedWindow('Image', cv2.WINDOW_NORMAL)
        # cv2.imshow('Image', im)
        # image = undistort.undistort(im)[:, :, ::-1]
        cam_num = CAMERA_IDX

        hits_image = project_vel_to_cam(hits_body, cam_num)

        x_im = hits_image[0, :] / hits_image[2, :]
        y_im = hits_image[1, :] / hits_image[2, :]
        z_im = hits_image[2, :]

        idx_infront = z_im > 0
        x_im = x_im[idx_infront]
        y_im = y_im[idx_infront]
        z_im = z_im[idx_infront]

        plt.figure(1)
        # depth = np.zeros(image.shape[:2])
        # for i in range(len(y_im)):
        #
        #     y_pix = int(round(y_im[i]))
        #     x_pix = int(round(x_im[i]))
        #     if 0 <= y_pix and y_pix < depth.shape[0]:
        #         if 0 <= x_pix and x_pix < depth.shape[1]:
        #             depth[y_pix, x_pix] = z_im[i]
        # viz = colored_depthmap(depth, 0, np.max(depth))
        # # cv2.imshow("viz", viz)
        # # cv2.waitKey(0)
        # v_cropped = viz[:, 600:800, :]
        # img_cropped= image[:, 600:800, :]
        # merged = cv2.hconcat([img_cropped, v_cropped])
        # cv2.imwrite("merged_t.png", v_cropped)
        plt.imshow(image)
        plt.hold(True)
        plt.scatter(x_im, y_im, c=z_im, s=5, linewidths=0)
        plt.xlim(0, 1616)
        plt.ylim(0, 1232)
        plt.show()

        return 0


    def _project_vels_to_cam(self, vel_names, idx):
        # vel_names: local vel name!
        is_initial = True
        # Other coordinate transforms we need
        T_poses = self._set_T_poses(idx)
        for i, vel_name in enumerate(vel_names):
            hits = load_vel_hits(os.path.join(self.v_root, vel_name))
            T_lb3_c_i = ssc_to_homo(self.x_lb3_c)
            T_body_lb3 = ssc_to_homo(self.x_body_lb3)

            T_lb3_body = np.linalg.inv(T_body_lb3)
            T_c_i_lb3 = np.linalg.inv(T_lb3_c_i)

            T_c_i_body = np.matmul(T_c_i_lb3, T_lb3_body)
            # # #
            # T_origin_c = T_poses[len(T_poses)//2]
            # T_c_origin = np.linalg.inv(T_origin_c)
            # T_c_c_i = T_c_origin * T_poses[i]
            # T_c_body_i = T_c_c_i * T_c_i_body
            # # #
            hits_c = np.matmul(T_c_i_body, hits)
            hits_im = np.matmul(self.K, hits_c[0:3, :])
            if is_initial:
                hits_merged = hits_im
                is_initial = False
            else:
                hits_merged = np.concatenate([hits_merged, hits_im], axis=1)

        return hits_merged

    def project_vels2cam(self, idx, viz=False):
        # image: undistorted!
        image = cv2.imread(os.path.join(self.img_root, self.img_list[idx]))
        vel_names = self._set_vel_names(idx)

        hits_image = self._project_vels_to_cam(vel_names, idx)

        x_im = hits_image[0, :] / hits_image[2, :]
        y_im = hits_image[1, :] / hits_image[2, :]
        z_im = hits_image[2, :]

        idx_infront = z_im > 0
        x_im = x_im[idx_infront]
        y_im = y_im[idx_infront]
        z_im = z_im[idx_infront]

        plt.scatter(x_im, y_im, c=z_im, s=5, linewidths=0)
        # plt.xlim(0, 1616)
        # plt.ylim(0, 1232)
        plt.xlim(0, 1616)
        plt.ylim(0, 1232)
        fig = plt.gcf()
        plt.show()
        fig.savefig("before.png")


        depth = np.zeros(image.shape[:2])
        ys = np.rint(y_im).astype(int)
        xs = np.rint(x_im).astype(int)
        print(depth.shape)
        print(ys[-1])
        print(xs[-1])

        # cond_l = 0 <= ys
        # cond_u = ys < depth.shape[0]
        # mask1 = np.bitwise_and(cond_l, cond_u)
        # print(np.count_nonzero(mask1))
        # cond_l = 0 <= xs
        # cond_u = xs < depth.shape[1]
        # mask2 = np.bitwise_and(cond_l, cond_u)
        # print(np.count_nonzero(mask2))
        # mask = np.matmul(mask1, mask2)
        # print(np.count_nonzero(mask))
        # y_pix = ys[mask]
        # x_pix = xs[mask]
        # z_pix = z_im[mask]
        # print(x_pix.shape)
        # print(y_pix.shape)
        # print(z_pix.shape)

        # for i in range(len(ys)):
        #     if 0 <= ys[i] and ys[i] < depth.shape[0]:
        #         if 0 <= xs[i] and xs[i] < depth.shape[1]:
        #             depth[ys[i], xs[i]] = z_im[i]
        #
        # viz_depth = colored_depthmap(depth, 0, 100)
        # if viz == True:
        #     cv2.imshow("viz", viz_depth)
        #     cv2.waitKey(0)
        #
        # plt.figure(1)
        # plt.imshow(image)
        # plt.hold(True)
        # plt.scatter(x_im, y_im, c=z_im, s=5, linewidths=0)
        # plt.xlim(0, 1616)
        # plt.ylim(0, 1232)
        # plt.show()
        #
        # return 0
    # def check_h30_time(self):
    #     for time in self.synced_times:
    #         self.set_h30(time)
    def _set_vel_names(self, idx):
        # idx indicates center idx of window
        vel_names = []
        for time in self.synced_times[idx - WINDOW_SIZE // 2: idx + WINDOW_SIZE // 2 + 1]:
            vel_name = str(time) + ".bin"
            vel_names.append(vel_name)
        return vel_names

    def _set_T_poses(self, idx):
        # idx indicates center idx of window
        T_poses = []
        for i in range(idx - WINDOW_SIZE // 2, idx + WINDOW_SIZE // 2 + 1):
            x_m = self.poses[i, 0]
            y_m = self.poses[i, 1]
            z_m = self.poses[i, 2]
            print(x_m, y_m, z_m)
            roll_rad = self.poses[i, 3]
            pitch_rad = self.poses[i, 4]
            yaw_rad = self.poses[i, 5]
            T_pose = getTransformationFromEulerAnglesRollPitchYawRadXYZMeters(roll_rad, pitch_rad, yaw_rad,
                                                                              x_m, y_m, z_m)

            T_poses.append(T_pose)

        p_origin_c = T_poses[0]
        p_origin_c_next = T_poses[1]
        p_c_c_next = np.matmul(np.linalg.inv(p_origin_c), p_origin_c_next)
        print("======")
        print(p_c_c_next)
        print("======")

        return T_poses

    def set_h30(self, time):
        utime = struct.unpack('<Q', self.h30_bin.read(8))[0]
        if time > utime:
            while time > utime:
                utime = struct.unpack('<Q', self.h30_bin.read(8))[0]
            utime = struct.unpack('<Q', self.h30_bin.read(8))[0]
            print(time - utime)


    def get_time(self, filename): # vel or img
        time = filename.split(".")[0]
        return int(time)

    def sync_vel_and_img(self):
        # For defensive programming....
        # Deprecated
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
    sync = Synchronizer()
    sync.project_vels2cam(idx=60, viz=True)
