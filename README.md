# Data-Platform

Collecting data using Qt.

## Dependencies

- [OpenCV 3](https://opencv.org/)\
`apt install libopencv-dev`

- [Qt 5](https://www.qt.io/)\
`apt install libeigen3-dev`

- [ROS Melodic Morenia](https://wiki.ros.org/cn/melodic/Installation/Ubuntu)

- [PCL 1.8](http://pointclouds.org/)\
`apt install libpcl-dev`

- [Ceres Solver](http://ceres-solver.org/)\
`apt install libceres-dev`

- [Eigen 3](https://eigen.tuxfamily.org/)\
`apt install libeigen3-dev`

- [Intel RealSense SDK](https://github.com/IntelRealSense/librealsense/blob/master/doc/installation.md)\
从 Download/Clone librealsense github repository 开始，只看自己对应版本的安装。\
安装过程中，为读取图像曝光时间等 image.metadata，cmake 时请执行：\
`cmake ../ -DFORCE_RSUSB_BACKEND=true -DCMAKE_BUILD_TYPE=release`

## Build

Tested on `Ubuntu 18.04`.

```sh
$ mkdir build & cd build
$ cmake ..
$ make
```

## 已实现功能

- 响应函数标定数据采集
- 响应函数标定
- 晕影函数标定数据采集
- 晕影函数标定
- 相机和激光雷达间外参标定数据采集
- 相机和激光雷达外参标定
