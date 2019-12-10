

# Install

First, install [openpose](https://github.com/CMU-Perceptual-Computing-Lab/openpose) by following its very detailed and very long official tutorial.  
Please also compile its python code. Either python2 or python3 is fine. Tutorial is [here](https://github.com/CMU-Perceptual-Computing-Lab/openpose/blob/master/doc/modules/python_module.md).

The major (not complete) steps are listed below:
``` 
cd ~/githubs
git clone https://github.com/CMU-Perceptual-Computing-Lab/openpose; cd openpose
mkdir -p build; cd build
cmake -DBUILD_PYTHON=ON .. # Compile to create python libraries.
make -j11
sudo make install # Install c++/python libraries to `/usr/local/python`.
```

After installation, check the environment variable `OPENPOSE_HOME`. We will need to read model from `${OPENPOSE_HOME}/models/`.
```
echo "'${OPENPOSE_HOME}' should be the folder of your openpose."
```

Make sure you can run its example 
```
cd ${OPENPOSE_HOME} /build/examples/tutorial_api_python
python3 04_keypoints_from_images.py # Both python2 and python3 are fine. This is not running in ROS. 
```

Set the environment variable `OPENPOSE_PYTHONPATH` as the installation directory of openpose python libraries. It's probably `/usr/local/python`.
```
export OPENPOSE_PYTHONPATH="/usr/local/python"
```

# Usage


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


