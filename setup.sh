#!/bin/bash

#Install python dependency 
pip install --upgrade pip

#Install apriltag_detector_pywrapper
cd apriltag_detector_pywrapper/
pip install -r requirements.txt  --user
python setup.py install --user
cd ..

source /opt/ros/kinetic/setup.bash
echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
echo "source /home/catkin_ws/devel/setup.bash" >> ~/.bashrc

cd /home/catkin_ws
catkin_make -DCMAKE_BUILD_TYPE=Release -Wno-dev
