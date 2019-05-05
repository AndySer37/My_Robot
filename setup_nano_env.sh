#!/bin/bash

### pre

sudo usermod -aG docker $USER

### install dependency

sudo apt update && sudo apt install -y python-pip cmake gcc g++ qt4-qmake libqt4-dev libjpeg-dev
pip install future numpy image

### install ROS

mkdir install && cd install
git clone https://github.com/JetsonHacksNano/installROS && cd installROS && ./installROS.sh
cd ..
sudo apt install -y ros-melodic-cv-bridge ros-melodic-image-transport ros-melodic-tf \
	     ros-melodic-diagnostic-updater ros-melodic-rgbd-launch ros-melodic-rviz \
	     ros-melodic-pcl-conversions ros-melodic-pcl-ros ros-melodic-tf2-sensor-msgs \
	     ros-melodic-tf-conversions ros-melodic-roslint 

### install pytorch

mkdir dl && cd dl
wget https://nvidia.box.com/shared/static/m6vy0c7rs8t1alrt9dqf7yt1z587d1jk.whl -O torch-1.1.0a0+b457266-cp27-cp27mu-linux_aarch64.whl
pip install torch-1.1.0a0+b457266-cp27-cp27mu-linux_aarch64.whl

git clone https://github.com/pytorch/vision && cd vision
sudo python setup.py install

### install librealsense

git clone https://github.com/JetsonHacksNano/installLibrealsense && cd installLibrealsense
./patchUbuntu.sh
./installLibrealsense.sh
cd ..

### install 