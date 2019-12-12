'''
Basic geo transformations.
'''

import numpy as np
import copy
import cv2


def form_T(R, p):
    T = np.identity(4)
    T[0:3, 0:3] = R
    T[0:3, 3:4] = np.array(p).reshape((3, 1))
    return T


def get_Rp_from_T(T):
    R = T[0:3, 0:3]
    p = T[0:3, 3:4]
    return (R, p)


def inv_R_p(R, p):
    T = form_T(R, p)
    T = np.linalg.inv(T)
    R_inv, p_inv = get_Rp_from_T(T)
    return R_inv, p_inv


def xyz_to_T(x=None, y=None, z=None):
    ''' Get 4x4 Transformation matrix from
        the translation (x, y, z)
    '''
    T = np.identity(4)
    data = [x, y, z]
    for i in range(3):
        if data[i] is not None:
            T[i, 3] = data[i]
    return T


def rot3x3_to_4x4(R):
    T = np.identity(4)
    T[0:3, 0:3] = R
    return T


def rot(axis, angle, matrix_len=3):
    R_vec = np.array(axis).astype(float)*angle
    R, _ = cv2.Rodrigues(R_vec)
    if matrix_len == 4:
        R = rot3x3_to_4x4(R)
    return R


def rotx(angle, matrix_len=3):
    return rot([1, 0, 0], angle, matrix_len)


def roty(angle, matrix_len=3):
    return rot([0, 1, 0], angle, matrix_len)


def rotz(angle, matrix_len=3):
    return rot([0, 0, 1], angle, matrix_len)


def euler2matrix(x, y, z, order='rxyz'):
    return rotx(x).dot(roty(y)).dot(rotz(z))

