'''
From the whole dataset, select the images for unit test.
Besides, merge two test images into one image -- 
    one on the left side, the other on the right side, 
    so that there are two people in the output image,
    in order to test whether the program supports multiple people.
'''


import cv2
import numpy as np
import glob
import os
import sys

if True:  # Add project root
    import sys
    import os
    ROOT = os.path.dirname(os.path.abspath(__file__))+'/../'
    sys.path.append(ROOT)
    from utils.lib_plot import show

SRC_FOLDER = ROOT + "data/all_recorded_images/"
DST_FOLDER_COLOR = ROOT + "output/color/"
DST_FOLDER_DEPTH = ROOT + "output/depth/"
RIGHT_SIDE_INDICES = [54, 334]
LEFT_SIDE_INDICES = [879, 1192]
COLUMN_RATIO = 0.55


def read_ith_image(i):
    color = cv2.imread(SRC_FOLDER + "color_{:05d}.png".format(i))
    depth = cv2.imread(
        SRC_FOLDER + "depth_{:05d}.png".format(i), cv2.IMREAD_UNCHANGED)
    return color, depth


def merge_img(I1, I2, column_ratio=0.5):
    column = I1.shape[1]
    assert(I1.shape == I2.shape)
    c = int(column*column_ratio)
    if len(I1.shape) == 2:
        I12 = np.hstack((I1[:, :c], I2[:, c:]))
    else:
        I12 = np.hstack((I1[:, :c, :], I2[:, c:, :]))
    return I12


def makedir(folder):
    if not os.path.exists(folder):
        os.makedirs(folder)


if __name__ == '__main__':

    makedir(DST_FOLDER_DEPTH)
    makedir(DST_FOLDER_COLOR)

    L1 = RIGHT_SIDE_INDICES[1] - RIGHT_SIDE_INDICES[0] + 1
    L2 = LEFT_SIDE_INDICES[1] - LEFT_SIDE_INDICES[0] + 1
    L = min(L1, L2)

    for i in range(L):
        print("Processing the {}/{}th image".format(i+1, L))
        lc, ld = read_ith_image(LEFT_SIDE_INDICES[0] + i)
        rc, rd = read_ith_image(RIGHT_SIDE_INDICES[0] + i)
        color = merge_img(lc, rc, COLUMN_RATIO)
        depth = merge_img(ld, rd, COLUMN_RATIO)

        filename = "{:05d}.png".format(i)
        fcolor = DST_FOLDER_COLOR+filename
        fdepth = DST_FOLDER_DEPTH+filename
        cv2.imwrite(fcolor, color)
        cv2.imwrite(fdepth, depth)
        if 0:
            show([color, depth], figsize=(10, 5))
            show([lc, rc], figsize=(10, 5))
        print("  Write color image to: " + fcolor)
        print("  Write depth image to: " + fdepth)
