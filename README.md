ros_openpose_rgbd
==================================
Combine Openpose 2D detection results and depth image to obtain 3D joint positions, and draw in ROS rviz.

**Demo** (Body only; 12 fps; Kind of good.)
![](doc/video_demo/demo_no_hand.gif)

**Demo** (Body+hand; 3 fps; Inaccurate hands' 3D positions.)
![](doc/video_demo/demo_with_hand.gif)

**Contents**:
TODO

# 1. Introduction

**Algorithm:** The workflow is:
1. Detect **2D human joints** from *color image* by Openpose.
2. Compute joints' **3D positions** by getting the depth from *depth image*. 
4. Visualize them in rviz by ROS markers.

**Code:** The main script is [detect_and_draw_joints.py](detect_and_draw_joints.py), which imports two library scripts [lib_openpose_detector.py](lib_openpose_detector.py) and [lib_draw_3d_joints.py](lib_draw_3d_joints.py).

# 2. Installation

**Environment:**    
Ubuntu 18.04, ROS melodic, python2.

**Openpose**:
First, install [Openpose](https://github.com/CMU-Perceptual-Computing-Lab/openpose) by following its very detailed and very long official tutorial.  
Please also compile its code into Python2 libraries. Tutorial is [here](https://github.com/CMU-Perceptual-Computing-Lab/openpose/blob/master/doc/modules/python_module.md).

The major (not complete) steps are listed below:
``` 
cd ~/githubs
git clone https://github.com/CMU-Perceptual-Computing-Lab/openpose; cd openpose
mkdir -p build; cd build
cmake  -DBUILD_PYTHON=ON \
    -DPYTHON_EXECUTABLE=/usr/bin/python2.7 \
    -DPYTHON_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython2.7m.so ..
make -j11
sudo make install # Install c++/python libraries to `/usr/local/python`.
```

After installation, check the environment variable `OPENPOSE_HOME`. We will need to read model from `${OPENPOSE_HOME}/models/`.
```
echo "'${OPENPOSE_HOME}' should be the folder of your openpose."
```

Make sure you can run its example 
```
cd ${OPENPOSE_HOME}/build/examples/tutorial_api_python
python 04_keypoints_from_images.py
```

Set the environment variable `OPENPOSE_PYTHONPATH` as the installation directory of openpose python libraries. It's probably `/usr/local/python`.
```
export OPENPOSE_PYTHONPATH="/usr/local/python"
```

# 3. Usage

## 3.1. Unittest


## 3.1.1. lib_openpose_detector.py
```
python lib_openpose_detector.py
```
The test case reads images from [data/image_i1/](data/image_i1/) and ouputs the results to [output/](output/).


## Test on realsense

**Bug:** I used a different coordinate direction than Realsense. (1) For me, I use X-Right, Y-Down, Z-Forward, which is the convention for camera. (2) For Realsense ROS package, it's X-Forward, Y-Left, Z-Up. So the point cloud published by Realsense doesn't match the 3D skeletons drawn by me.

# Program structure

lib_openpose_detector.py: test detection
lib_rviz_drawer: test read detection result and display.
detect_imgs_from_rostopic.py # from rostopic
detect_imgs_from_disk.py  # from local disk

明天先把ros_record_rgbd给更新一下, 把color/depth分开保存.
然后打扫一下卫生, 采集一些数据.
洗一下鞋子.
采集一下数据

11点去食堂吃饭.
晚上吃睡觉.

明年去超市.

# Format

I'm using `Pose Output Format (COCO)` and `Hand Output Format` from this [page](https://github.com/CMU-Perceptual-Computing-Lab/openpose/blob/master/doc/output.md).

# Speed

Running speed of `detect_and_draw_joints.py`:
* Settings: RTX 2070; Image resize to 320x240.
* Results:
    * body + hand: 3 fps.
    * Only body: 12 fps.

