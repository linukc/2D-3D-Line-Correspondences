#|||||||||||||||||||||
#https://github.com/linukc/2D-3D-pose-tracking - репозиторий идентичный основному 2D-3D, только там исправлена https://github.com/cherubicXN/afm_cvpr2019/issues/17#issuecomment-590211288 
#docker build -t 2d-3d . --build-arg NUM_THREADS=4
#|||||||||||||||||||||


#---------------------BUILD-----------------------
FROM nvidia/cuda:10.1-cudnn7-devel-ubuntu18.04 as build

ARG APT_INSTALL="apt-get install -y --no-install-recommends"
ARG PIP_INSTALL="python -m pip --no-cache-dir install --upgrade --user"
ARG GIT_CLONE="git clone --depth 10"
ENV HOME /root

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
---------------------------------

WORKDIR $HOME
$GIT_CLONE --single-branch --branch docker https://github.com/linukc/2D-3D-pose-tracking

WORKDIR 2D-3D-pose-tracking/afm/scripts/lib
RUN make -j$(nproc)         


#---------------------PRODUCTION--------------------
FROM nvidia/cuda:10.1-base-ubuntu18.04

ARG APT_INSTALL="apt-get install -y --no-install-recommends"
ARG PIP_INSTALL="python -m pip --no-cache-dir install --upgrade"
ARG GIT_CLONE="git clone --depth 10"
ENV HOME /root

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
         python-setuptools \
         yacs \
         matplotlib \
         tqdm \
         scikit-image

# copy pytorch
COPY --from=build /root/.local /root/.local
ENV PATH=/root/.local/bin:$PATH

RUN $PIP_INSTALL opencv-python-headless==3.4.3.18

WORKDIR $HOME
RUN wget https://github.com/opencv/opencv/archive/3.4.3.zip && \
         unzip 3.4.3.zip && \
         rm 3.4.3.zip && \
         mv opencv-3.4.3 OpenCV

WORKDIR OpenCV/build
RUN cmake -DWITH_QT=OFF -DBUILD_EXAMPLES=OFF .. && \
         make -j$(nproc) && \
         make install && \

WORKDIR $HOME
RUN apt-get update && $APT_INSTALL \
         libgoogle-glog-dev \
         libgflags-dev \
         libatlas-base-dev \
         libeigen3-dev \
         libsuitesparse-dev && \
         $GIT_CLONE https://ceres-solver.googlesource.com/ceres-solver && \

WORKDIR ceres-solver/build         
RUN cmake .. && \
         make -j$(nproc) && \
         make install

WORKDIR $HOME
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

#свап sh и bash для работы source
RUN mv /bin/sh /bin/sh-old && \
         ln -s /bin/bash /bin/sh

RUN $APT_INSTALL ros-melodic-pcl-ros \
         ros-melodic-diagnostic-msgs \
         python-tk && \
         source /opt/ros/melodic/setup.bash && \
         rosdep init && \
         sudo rosdep update

WORKDIR catkin_ws
RUN mkdir src && \
         $GIT_CLONE --single-branch --branch docker https://github.com/linukc/2D-3D-pose-tracking src && \
         rm -rf src/VINS-Mono-config && \
         $GIT_CLONE https://github.com/HKUST-Aerial-Robotics/VINS-Mono.git src && \
         source /opt/ros/melodic/setup.bash && \
         catkin_make -j$(nproc)

COPY --from=build $HOME/2D-3D-pose-tracking/afm/scripts/lib/afm_op/CUDA.so src/afm/scripts/lib/afm_op/CUDA.so
COPY --from=build $HOME/2D-3D-pose-tracking/afm/scripts/lib/squeeze/squeeze.so src/afm/scripts/lib/squeeze/squeeze.so

WORKDIR $HOME

RUN apt-get clean && \
         apt-get autoremove && \
         rm -rf /var/lib/apt/lists/* /tmp/* $HOME/OpenCV $HOME/ceres-solver
