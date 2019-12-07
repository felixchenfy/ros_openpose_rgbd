import sys
import cv2
import os
from sys import platform
import argparse
import time

if True:  # Import openpose
    OPENPOSE_PYTHONPATH = os.environ['OPENPOSE_PYTHONPATH']
    sys.path.append(OPENPOSE_PYTHONPATH)
    from openpose import pyopenpose as op

''' -------------------------------------- Settings -------------------------------------- '''

OPENPOSE_HOME = os.environ['OPENPOSE_HOME'] + "/"
MODEL_PATH = OPENPOSE_HOME + "models/"

''' ------------------------------- Command line arguments ------------------------------- '''


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

def set_openpose_params():
    ''' Custom openpose params.
    (Refer to $OPENPOSE_HOME/include/openpose/flags.hpp for more parameters.)
    '''
    params = dict()
    params["model_folder"] = MODEL_PATH
    params["face"] = False # I haven't done this.
    params["hand"] = True
    params["net_resolution"] = "320x240" # e.g.: "240x160"
    params["model_pose"] = "BODY_25" # Choices: [BODY_25, COCO, MPI, MPI_4_layers]

    # Add others settings from command line arguments to `params`
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
params = set_openpose_params()


# Starting OpenPose
opWrapper = op.WrapperPython()
opWrapper.configure(params)
opWrapper.start()

# Read frames on directory
imagePaths = op.get_images_on_directory(args[0].image_dir)
start = time.time()

# Process and display images
for i, imagePath in enumerate(imagePaths):
    datum = op.Datum()
    imageToProcess = cv2.imread(imagePath)
    datum.cvInputData = imageToProcess
    opWrapper.emplaceAndPop([datum])

    print("Body keypoints: \n" + str(datum.poseKeypoints))

    if not args[0].no_display:
        cv2.imshow("OpenPose 1.5.1 - Tutorial Python API", datum.cvOutputData)
        # key = cv2.waitKey(15)
        key = cv2.waitKey(0)
        if key == 27:
            break

end = time.time()
print("OpenPose demo successfully finished. Total time: " +
      str(end - start) + " seconds")
print("Total images: {}".format(len(imagePaths)))
