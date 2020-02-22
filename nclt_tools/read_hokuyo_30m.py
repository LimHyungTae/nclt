# !/usr/bin/python
#
# Example code to go through the hokuyo_30m.bin file, read timestamps and the hits
# in each packet, and plot them.
#
# To call:
#
#   python read_hokuyo_30m.py hokuyo_30m.bin
#
from nclt_tools.CONFIG import *
import sys
import struct
import numpy as np
import matplotlib.pyplot as plt
import csv
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

    plt.ion()
    count = 0

    while True:

        # Read timestamp
        utime = struct.unpack('<Q', f_bin.read(8))[0]


        r = np.zeros(num_hits)

        for i in range(num_hits):

            s = struct.unpack('<H', f_bin.read(2))[0]
            r[i] = convert(s)

            #print s

        x = r * np.cos(angles)
        y = r * np.sin(angles)

    #     plt.clf()
    #     plt.plot(x, y, '.')
    #     plt.title(utime)
    #     plt.draw()
    #
    # plt.show()
    f_bin.close()

    return 0


def sync_hokuyo(hokuyo_bin):
    # hokuyo_30m always has 1081 hits
    num_hits = 1081

    # angles for each range observation
    rad0 = -135 * (np.pi / 180.0)
    radstep = 0.25 * (np.pi / 180.0)
    angles = np.linspace(rad0, rad0 + (num_hits - 1) * radstep, num_hits)

    f_bin = open(hokuyo_bin, "r")

    plt.ion()
    count = 0
    utime_prev = 0
    data_prev = None
    is_initial = True
    global TIMES, HOKUYO_TARGET_PATH
    TIMES = sorted(TIMES)
    try:
        while len(TIMES) > 0:

            utime = struct.unpack('<Q', f_bin.read(8))[0]
            print(utime, TIMES[0], TIMES[0]-utime)

            if is_initial:
                if utime >= TIMES[0]:
                    raise RuntimeError("Should pop TIMES!")
                else:
                    is_initial = False

            r = np.zeros(num_hits)

            for i in range(num_hits):
                s = struct.unpack('<H', f_bin.read(2))[0]
                r[i] = convert(s)

            x = r * np.cos(angles)
            y = r * np.sin(angles)

            x = x.reshape(1, -1)
            y = y.reshape(1, -1)
            data = np.concatenate((x, y), axis=0)
            print(data.shape)

            if utime >= TIMES[0] and utime_prev < TIMES[0]:

                diff_upper = abs(utime - TIMES[0])
                diff_lower = abs(TIMES[0] - utime_prev)

                if diff_upper <= diff_lower:
                    target = data
                elif diff_upper > diff_lower:
                    target = data_prev

                filename = str(TIMES.pop(0)) + ".csv"
                with open(os.path.join(HOKUYO_TARGET_PATH, filename), 'w') as f:
                    writer = csv.writer(f)
                    writer.writerow(target[0, :])
                    writer.writerow(target[1, :])

            data_prev = data
            utime_prev = utime

    except struct.error or IndexError:
        print("May be done?")
        f_bin.close()




    return 0

if __name__ == '__main__':
    if not os.path.exists(HOKUYO_TARGET_PATH):
        os.mkdir(HOKUYO_TARGET_PATH)
    sync_hokuyo(HOKUYO_PATH)

