#|||||||||||||||||||||
#---------------------
#TO DO ...
#сделать волюм репозиториев и убрать их гит клон
#ДОБАВИТЬ WORKDIR HOME И УБРАТЬ ПО МАКСИМОМУ CD 
#разделять загрузки и установки в разные команды а то при ошибках установки все еще раз качается
#---------------------
#|||||||||||||||||||||


#---------------------BUILD-----------------------
FROM nvidia/cuda:10.1-cudnn7-devel-ubuntu18.04 as build

ARG APT_INSTALL="apt-get install -y --no-install-recommends"
ARG PIP_INSTALL="python -m pip --no-cache-dir install --upgrade --user"
ARG GIT_CLONE="git clone --depth 10"

RUN apt-get update && $APT_INSTALL \
         build-essential \
         cmake \
         git \
         python2.7 \
         python-dev \
         python-pip \
         python-setuptools

RUN $PIP_INSTALL wheel \
         future \
         cython

#-----Pytorch with cuda 10.1-----
RUN $PIP_INSTALL torch==1.4.0 torchvision==0.5.0

RUN cd / && mkdir -p catkin_ws/src && cd catkin_ws/ && \
         $GIT_CLONE --single-branch --branch docker https://github.com/linukc/2D-3D-pose-tracking src

RUN cd / && cd catkin_ws/src/afm/scripts/lib && make -j$(nproc) && ls afm_op -l         


#---------------------PRODUCTION--------------------
FROM nvidia/cuda:10.1-base-ubuntu18.04

ARG APT_INSTALL="apt-get install -y --no-install-recommends"
ARG PIP_INSTALL="python -m pip --no-cache-dir install --upgrade"
ARG GIT_CLONE="git clone --depth 10"

RUN apt-get update && $APT_INSTALL \
         build-essential \
         cmake \
         git \
         unzip \
         wget \
         pkg-config \
         libglib2.0-0 \
         python2.7 \
         python-dev \
         python-pip \
         python-setuptools

COPY --from=build /root/.local /root/.local
ENV PATH=/root/.local/bin:$PATH

RUN $PIP_INSTALL yacs \
         yacs \
         matplotlib \
         tqdm \
         scikit-image


RUN $PIP_INSTALL opencv-python-headless==3.4.3.18

RUN wget https://github.com/opencv/opencv/archive/3.4.3.zip && \
         unzip 3.4.3.zip && \
         rm 3.4.3.zip && \
         mv opencv-3.4.3 OpenCV && \
         cd OpenCV && \
         mkdir build && \
         cd build && \
         cmake \
         -DWITH_QT=OFF \
         -DBUILD_EXAMPLES=OFF .. && \
         make -j$(nproc) && \
         make install && \
         cd .. && \
         cd .. && \
         rm -rf OpenCV


RUN apt-get update && $APT_INSTALL \
         libgoogle-glog-dev \
         libgflags-dev \
         libatlas-base-dev \
         libeigen3-dev \
         libsuitesparse-dev && \
         $GIT_CLONE https://ceres-solver.googlesource.com/ceres-solver && \
         cd ceres-solver && \
         mkdir build && cd build && \
         cmake .. && \
         make -j$(nproc) && \
         make install && \
         rm -rf ../../ceres-solver


ENV ROS_DISTRO melodic
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8

RUN echo 'Etc/UTC' > /etc/timezone && \
         ln -s /usr/share/zoneinfo/Etc/UTC /etc/localtime && \ 
         apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 && \
         echo "deb http://packages.ros.org/ros/ubuntu bionic main" > /etc/apt/sources.list.d/ros1-latest.list && \
         apt-get update && \
         $APT_INSTALL tzdata \
         ros-melodic-desktop \
         python-rosdep \
         python-catkin-tools \
         python-rosinstall \
         python-rosinstall-generator \
         python-wstool \
         build-essential \
         ros-${ROS_DISTRO}-cv-bridge \
         ros-${ROS_DISTRO}-image-transport \
         ros-${ROS_DISTRO}-message-filters \
         ros-${ROS_DISTRO}-tf && \
         $PIP_INSTALL catkin_pkg \
         rospkg \
         empy

#Нужно переносить в начало, а то проблемы с source
RUN mv /bin/sh /bin/sh-old && \
         ln -s /bin/bash /bin/sh

RUN $APT_INSTALL ros-melodic-pcl-ros \
         ros-melodic-diagnostic-msgs \
         python-tk && \
         source /opt/ros/melodic/setup.bash && \
         rosdep init && \
         sudo rosdep update && \
         cd / && mkdir -p catkin_ws/src && cd catkin_ws/ && \
         $GIT_CLONE --single-branch --branch docker https://github.com/linukc/2D-3D-pose-tracking src && \
         rm -rf src/VINS-Mono-config && \
         $GIT_CLONE https://github.com/HKUST-Aerial-Robotics/VINS-Mono.git /tmp/VINS-Mono && \
         mv /tmp/VINS-Mono src

RUN cd / && cd catkin_ws && source /opt/ros/melodic/setup.bash && \
         catkin_make -j6

COPY --from=build /catkin_ws/src/afm/scripts/lib/afm_op/CUDA.so /catkin_ws/src/afm/scripts/lib/afm_op/CUDA.so
COPY --from=build /catkin_ws/src/afm/scripts/lib/squeeze/squeeze.so /catkin_ws/src/afm/scripts/lib/squeeze/squeeze.so

RUN apt-get clean && apt-get autoremove && rm -rf /var/lib/apt/lists/* /tmp/*
