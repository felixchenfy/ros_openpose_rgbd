import sys
import cv2
import os
from sys import platform
import argparse
import time
import numpy as np
from tempfile import TemporaryFile

if True:  # Import openpose
    OPENPOSE_PYTHONPATH = os.environ['OPENPOSE_PYTHONPATH']
    sys.path.append(OPENPOSE_PYTHONPATH)
    from openpose import pyopenpose as op
    ROOT = os.path.dirname(os.path.abspath(__file__))+'/'

''' -------------------------------------- Settings -------------------------------------- '''

OPENPOSE_HOME = os.environ['OPENPOSE_HOME'] + "/"
MODEL_PATH = OPENPOSE_HOME + "models/"

''' ------------------------------- Command line arguments ------------------------------- '''


class OpenposeDetector(object):
    def __init__(self):
        self._params = self._set_openpose_params()
        self._opWrapper = op.WrapperPython()
        self._opWrapper.configure(self._params)
        self._opWrapper.start()  # Start Openpose.

    def detect(self, color_image):
        '''
        Arguments:
            color_image {image}: shape=[rows, cols, 3]; type=np.uint8.
        Return:
            datum {op.Datum()}: Detect results are stored here.
                datum.poseKeypoints {np.ndarray}:
                    shape = [P, N, 3]; type = np.float32.
                    P: Number of detected human bodies. 
                    N: N body joints defined by the detector model, such as COCO and MPI.
                    3: [pixel_column, pixel_row, confidence]. 
                        The not-detected joint has a [0., 0., 0.] value.
                datum.poseKeypoints {list}:
                    List length is 2, which are the left hand and the right hand.
                    Each element is a np.ndarray of shape = [P, M, 3]
                    P and 3 have the same meaning as `poseKeypoints`.
                    M: M body joints.
        For model definition, please see:
            https://github.com/CMU-Perceptual-Computing-Lab/openpose/blob/master/doc/output.md
        '''
        datum = op.Datum()
        datum.cvInputData = color_image
        self._opWrapper.emplaceAndPop([datum])
        # print("Body keypoints: \n" + str(datum.poseKeypoints))
        # print("Hand keypoints: \n" + str(datum.handKeypoints))
        return datum

    def save_result(self, datum, pose_filename, hand_filename):
        ''' Save body and hand joints to two binary files. '''

        def save_binary(filename, data):
            np.save(filename, np.array(data))

        def save_txt(filename, data):
            np.savetxt(filename, np.array(data), delimiter=",")

        save_binary(pose_filename, datum.poseKeypoints)
        save_binary(hand_filename, datum.handKeypoints)
        # To load the data, use:
        # bodies_joints = np.load(pose_filename)
        # hands_joints = np.load(hand_filename)

    def _set_openpose_params(self, command_line_args=[]):
        ''' Custom openpose params.
        (Refer to $OPENPOSE_HOME/include/openpose/flags.hpp for more parameters.)
        '''
        params = dict()
        params["model_folder"] = MODEL_PATH
        params["face"] = False  # I haven't done this.
        params["hand"] = True
        params["net_resolution"] = "320x240"  # e.g.: "240x160"
        params["model_pose"] = "COCO"  # Please use "COCO".

        # Add others settings from command line arguments to `params`
        if command_line_args:
            args = command_line_args
            for i in range(0, len(args[1])):
                curr_item = args[1][i]
                if i != len(args[1])-1:
                    next_item = args[1][i+1]
                else:
                    next_item = "1"
                if "--" in curr_item and "--" in next_item:
                    key = curr_item.replace('-', '')
                    if key not in params:
                        params[key] = "1"
                elif "--" in curr_item and "--" not in next_item:
                    key = curr_item.replace('-', '')
                    if key not in params:
                        params[key] = next_item
        return params


def test_on_images():
    ''' -------------------------------------- Settings -------------------------------------- '''
    DST_FOLDER = ROOT + "output/"
    DST_pose_filename = DST_FOLDER + "body_joints.npy"
    DST_hand_filename = DST_FOLDER + "hand_joints.npy"
    if not os.path.exists(DST_FOLDER):
        os.makedirs(DST_FOLDER)

    ''' -------------------------------------- Command line arguments -------------------------------------- '''
    def parse_args():
        parser = argparse.ArgumentParser()
        parser.add_argument("--image_dir",
                            # default=OPENPOSE_HOME+"examples/media/",
                            default="data/two_images/",
                            help="Process a directory of images. Read all standard formats (jpg, png, bmp, etc.).")
        parser.add_argument("--no_display", default=False,
                            help="Enable to disable the visual display.")
        args = parser.parse_known_args()
        return args

    args = parse_args()

    ''' -------------------------------------- Read; Detect; Save. -------------------------------------- '''
    detector = OpenposeDetector()

    # Read frames on directory
    imagePaths = op.get_images_on_directory(args[0].image_dir)
    start = time.time()

    # Process and display images
    for i, imagePath in enumerate(imagePaths):
        datum = op.Datum()
        color_image = cv2.imread(imagePath)

        datum = detector.detect(color_image)

        detector.save_result(datum, DST_pose_filename, DST_hand_filename)

        if not args[0].no_display:
            cv2.imshow("OpenPose 1.5.1 - Tutorial Python API",
                       datum.cvOutputData)
            # key = cv2.waitKey(15)
            key = cv2.waitKey(0)
            if key == 27:
                break

    end = time.time()
    print("OpenPose demo successfully finished. Total time: " +
          str(end - start) + " seconds")
    print("Total images: {}".format(len(imagePaths)))


if __name__ == '__main__':
    test_on_images()
