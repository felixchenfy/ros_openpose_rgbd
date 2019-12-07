

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

