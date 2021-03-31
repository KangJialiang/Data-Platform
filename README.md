# Data-Platform
Code for collecting data using Qt.

## 1.IntelRealSense/librealsense 
librealsense 安装请参照这里[执行](https://github.com/IntelRealSense/librealsense/blob/master/doc/installation.md).
(从 Download/Clone librealsense github repository 开始，只看自己对应版本的安装)

安装过程中，为读取图像曝光时间等image.metadata，cmake时请执行：
```shell
cmake ../ -DFORCE_RSUSB_BACKEND=true -DCMAKE_BUILD_TYPE=release
```

## 2.my_realsense_lab
该文件夹下代码是为满足特定需求而编写的代码，在每一个子文件夹下（如 rs-gamma）执行以下命令可在build目录下的bin文件夹中生成对应的可执行文件
```shell
mkdir build && cd build && cmake .. && make 
```
执行生成的可执行文件“rs-xxx”：
```shell
bin/rs-xxx
```
