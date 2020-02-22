# import minkindr as mk
import numpy as np
# import IPython
# import quaternion
from pyquaternion import Quaternion
np.set_printoptions(precision=16)

def getRotationMatrixFromEulerAnglesRollPitchYaw(roll_rad, pitch_rad, yaw_rad):
    R_yaw = np.array([[np.cos(yaw_rad), -np.sin(yaw_rad), 0.0],
                     [np.sin(yaw_rad), np.cos(yaw_rad), 0.0],
                     [0.0, 0.0, 1.0]])

    R_pitch = np.array([[np.cos(pitch_rad), 0.0, np.sin(pitch_rad)],
                     [0.0, 1.0, 0.0],
                     [-np.sin(pitch_rad), 0.0, np.cos(pitch_rad)]])

    R_roll = np.array([[1.0, 0.0, 0.0],
                    [0.0, np.cos(roll_rad), -np.sin(roll_rad)],
                    [0.0, np.sin(roll_rad), np.cos(roll_rad)]])

    R = np.dot(np.dot(R_yaw, R_pitch), R_roll)
    return R

def getQuaternionFromFromEulerAnglesRollPitchYawRad(roll_rad, pitch_rad, yaw_rad):
    R = getRotationMatrixFromEulerAnglesRollPitchYaw(roll_rad, pitch_rad, yaw_rad)
    #assert(np.linalg.det(R) == 1.0)
    q = Quaternion(R)

    return q

def getTransformationFromEulerAnglesRollPitchYawRadXYZMeters(roll_rad, pitch_rad, yaw_rad, x_m, y_m, z_m):
    R = getRotationMatrixFromEulerAnglesRollPitchYaw(roll_rad, pitch_rad, yaw_rad)
    T = np.identity(4)
    T[:3, :3] = R
    T[0, 3] = x_m
    T[1, 3] = y_m
    T[2, 3] = z_m
    return T

def getTransformation(roll_rad, pitch_rad, yaw_rad, x_m, y_m, z_m):
    R = getRotationMatrixFromEulerAnglesRollPitchYaw(roll_rad, pitch_rad, yaw_rad)
    T = np.identity(4)
    T[:3, :3] = R
    T[0, 3] = x_m
    T[1, 3] = y_m
    T[2, 3] = z_m
    return T

# def dumpForYaml(T):
#   M = T.getTransformationMatrix()
#
#   FILENAME = 'transformation.yaml'
#   f = open(FILENAME, 'w')
#   for i in range(16):
#     row_idx = int(i) / int(4)
#     col_idx = np.mod(i, 4)
#     v = M[row_idx, col_idx]
#     f.writelines('- ' + "%.16f" % v + '\n')
#
#   f.close()
