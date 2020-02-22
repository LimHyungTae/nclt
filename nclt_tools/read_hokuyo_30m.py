# !/usr/bin/python
#
# Example code to go through the hokuyo_30m.bin file, read timestamps and the hits
# in each packet, and plot them.
#
# To call:
#
#   python read_hokuyo_30m.py hokuyo_30m.bin
#

import sys
import struct
import numpy as np
import matplotlib.pyplot as plt

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

        print 'Timestamp', utime

        r = np.zeros(num_hits)

        for i in range(num_hits):

            s = struct.unpack('<H', f_bin.read(2))[0]
            r[i] = convert(s)

            #print s

        x = r * np.cos(angles)
        y = r * np.sin(angles)

        plt.clf()
        plt.plot(x, y, '.')
        plt.title(utime)
        plt.draw()

    plt.show()
    f_bin.close()

    return 0

if __name__ == '__main__':
    read_hokuyo("/home/joohyun/git/hokuyo_data/2012-05-11/hokuyo_30m.bin")
    # sys.exit(main(sys.argv))
