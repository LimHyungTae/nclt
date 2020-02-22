import os
import numpy as np
ROOT = "/media/joohyun/My Passport/iros20"
ROOT = "/home/joohyun/git"
DATE = "2012-05-11"

CUTOFF_EDGE = 30

GT_PATH = os.path.join(ROOT, "ground_truth", "groundtruth_" + DATE + ".csv")
VEL_DIR = os.path.join(ROOT, "velodyne_data", DATE, "velodyne_sync")
IMG_DIR = os.path.join(ROOT, "images", DATE, "lb3", "Cam5")

CAMERA_IDX = 5


HOKUYO_PATH = os.path.join(ROOT, "hokuyo_data", DATE, "hokuyo_30m.bin")
# Will be generated
HOKUYO_TARGET_PATH = os.path.join(ROOT, "hokuyo_data", DATE, "hokuyo_sync")
UNDISTORTED_DIR = os.path.join(ROOT, "images", DATE, "lb3", "Undistorted-All", "Cam5")
TF_DIR = os.path.join(ROOT, "tf")
TIMES = []

if os.path.exists(UNDISTORTED_DIR):
    tmp = os.listdir(IMG_DIR)
    for name in tmp:
        time = int(name.split(".")[0])
        TIMES.append(time)
    sorted(TIMES)

''' parameters '''
K = np.loadtxt('../params/K_cam%d.csv' % (CAMERA_IDX), delimiter=',')
x_lb3_c = np.loadtxt('../params/x_lb3_c%d.csv' % (CAMERA_IDX), delimiter=',')

# Other coordinate transforms we need
x_body_lb3 = [0.035, 0.002, -1.23, -179.93, -0.23, 0.50]
'''============'''

def get_name(time, d_type):
    if d_type == "vel":
        return os.path.join(VEL_DIR, str(time) + ".bin")
    elif d_type == "img":
        return os.path.join(UNDISTORTED_DIR, str(time) + ".png")
    elif d_type == "tf":
        return os.path.join(TF_DIR, str(time) + ".csv")
    elif d_type == "hky":
        return os.path.join(HOKUYO_TARGET_PATH, str(time) + ".csv")

    else:
        raise RuntimeError("Wrong d-type!")


def load_tf(time):
    name = get_name(time, "tf")
    tf = np.loadtxt(name, delimiter=",")
    tf = np.concatenate((tf, np.array([[0., 0., 0., 1]])), axis=0)
    return tf

def load_hokuyo(time):
    name = get_name(time, "hky")
    xy = np.loadtxt(name, delimiter=",")
    row = np.zeros((1, 1081), dtype=np.float)
    row2 = np.full((1, 1081), 1)
    xyz_homo = np.concatenate((xy, row, row2), axis=0)
    return xyz_homo

"""
/root
---/ground_truth
---/hokuyo_data
    ---/$DATE 
---/images
    ---/$DATE
        ---/lb3 
            ---/Undistorted-All
                ---/Cam5
---/tf
---/velodyne_data
    ---/$DATE
        ---/velodyne_sync
---/%our_ws
        ---/params        
"""

if __name__ == "__main__":
    # tf = load_tf(TIMES[10])
    # print(tf.shape)
    # print(tf)

    xy = load_hokuyo(TIMES[10])
    print(xy.shape)
    print(xy)