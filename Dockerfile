FROM osrf/ros:kinetic-desktop-full

ENV CATKIN_WORKSPACE /home/catkin_ws

RUN apt-get update && apt-get install -y \
    python-setuptools \
	python-rosinstall \
	ipython \
	libeigen3-dev \
	libboost-all-dev \
	doxygen \
	libopencv-dev \
	ros-kinetic-vision-opencv \
	ros-kinetic-image-transport-plugins \
	ros-kinetic-cmake-modules \
	python-software-properties \
	software-properties-common \
	libpoco-dev \
	python-matplotlib \
	python-scipy \
	python-git \
	python-pip \
	libtbb-dev \
	libblas-dev \
	liblapack-dev \
	python-catkin-tools \
	libv4l-dev \
	wget \
	autoconf automake 

RUN python -m pip install --upgrade pip; python -m pip install python-igraph --upgrade
RUN mkdir -p $CATKIN_WORKSPACE/src && \
	cd $CATKIN_WORKSPACE && \
	catkin init && \
	catkin config --extend /opt/ros/kinetic && \
	catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release

WORKDIR /app
COPY . /app
RUN cp -a /app $CATKIN_WORKSPACE/src/kalibr

RUN ./setup.sh

RUN sed -i '$isource "/home/catkin_ws/devel/setup.bash"' /ros_entrypoint.sh