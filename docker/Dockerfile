FROM ubuntu:18.04

RUN rm -rf /var/lib/apt/lists/*
ENV NVIDIA_VISIBLE_DEVICES=all \
    NVIDIA_DRIVER_CAPABILITIES=all

# install packages
RUN dpkg --add-architecture i386 && \
    apt-get update && apt-get install -y --no-install-recommends \
        libxau6 libxau6:i386 \
        libxdmcp6 libxdmcp6:i386 \
        libxcb1 libxcb1:i386 \
        libxext6 libxext6:i386 \
        libx11-6 libx11-6:i386 && \
    rm -rf /var/lib/apt/lists/*

ENV LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:/usr/lib/i386-linux-gnu${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}

#GYM for openai pip2
RUN apt-get update -y && apt-get install -y software-properties-common
RUN add-apt-repository ppa:deadsnakes/ppa && apt-get update -y  && apt-get install -y python && apt-get install -y python-dev libmysqlclient-dev python-pip
RUN python -m pip install pip --upgrade
RUN apt-get update && \
    pip install future && \
    pip install gym

# #GYM for openai pip3
# RUN apt-get update -y && apt-get install -y software-properties-common
# RUN add-apt-repository ppa:deadsnakes/ppa && apt-get update -y  && apt-get install -y python3.7 && apt-get install -y python3.7-dev libmysqlclient-dev python3-pip
# RUN python3.7 -m pip install pip --upgrade
# RUN apt-get update && \
#     pip3 install future && \
#     pip3 install gym

RUN apt-get update && apt-get install -y --no-install-recommends \
        apt-utils && \
    apt-get install -y --no-install-recommends \
        git \
        ca-certificates \
        make \
        automake \
        autoconf \
        libtool \
        curl \
        pkg-config \
        python \
        libxext-dev \
        libx11-dev \
        x11proto-gl-dev && \
    rm -rf /var/lib/apt/lists/*

WORKDIR /opt/libglvnd
RUN git clone --branch=v1.0.0 https://github.com/NVIDIA/libglvnd.git . && \
    ./autogen.sh && \
    ./configure --prefix=/usr/local --libdir=/usr/local/lib/x86_64-linux-gnu && \
    make -j"$(nproc)" install-strip && \
    find /usr/local/lib/x86_64-linux-gnu -type f -name 'lib*.la' -delete

RUN dpkg --add-architecture i386 && \
    apt-get update && apt-get install -y --no-install-recommends \
        gcc-multilib \
        libxext-dev:i386 \
        libx11-dev:i386 && \
    rm -rf /var/lib/apt/lists/*

# 32-bit libraries
RUN make distclean && \
    ./autogen.sh && \
    ./configure --prefix=/usr/local --libdir=/usr/local/lib/i386-linux-gnu --host=i386-linux-gnu "CFLAGS=-m32" "CXXFLAGS=-m32" "LDFLAGS=-m32" && \
    make -j"$(nproc)" install-strip && \
    find /usr/local/lib/i386-linux-gnu -type f -name 'lib*.la' -delete

RUN echo '/usr/local/lib/x86_64-linux-gnu' >> /etc/ld.so.conf.d/glvnd.conf && \
    echo '/usr/local/lib/i386-linux-gnu' >> /etc/ld.so.conf.d/glvnd.conf && \
    ldconfig

ENV LD_LIBRARY_PATH=/usr/local/lib/x86_64-linux-gnu:/usr/local/lib/i386-linux-gnu${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}

# cuda 8.0
RUN apt-key adv --fetch-keys http://developer.download.nvidia.com/compute/cuda/repos/ubuntu1604/x86_64/7fa2af80.pub && \
    echo "deb http://developer.download.nvidia.com/compute/cuda/repos/ubuntu1604/x86_64 /" > /etc/apt/sources.list.d/cuda.list && \
    echo "deb http://developer.download.nvidia.com/compute/machine-learning/repos/ubuntu1604/x86_64 /" > /etc/apt/sources.list.d/nvidia-ml.list

ENV CUDA_VERSION_MAJOR=8.0 \
    CUDA_VERSION_MINOR=61 \
    CUDA_PKG_EXT=8-0
ENV CUDA_VERSION=$CUDA_VERSION_MAJOR.$CUDA_VERSION_MINOR
RUN apt-get update && apt-get install -y --no-install-recommends \
        cuda-nvrtc-dev-$CUDA_PKG_EXT=$CUDA_VERSION-1 \
        cuda-nvgraph-dev-$CUDA_PKG_EXT=$CUDA_VERSION-1 \
        cuda-cusolver-dev-$CUDA_PKG_EXT=$CUDA_VERSION-1 \
        cuda-cublas-dev-$CUDA_PKG_EXT=$CUDA_VERSION.2-1 \
        cuda-cufft-dev-$CUDA_PKG_EXT=$CUDA_VERSION-1 \
        cuda-curand-dev-$CUDA_PKG_EXT=$CUDA_VERSION-1 \
        cuda-cusparse-dev-$CUDA_PKG_EXT=$CUDA_VERSION-1 \
        cuda-npp-dev-$CUDA_PKG_EXT=$CUDA_VERSION-1 \
        cuda-cudart-dev-$CUDA_PKG_EXT=$CUDA_VERSION-1 \
        cuda-misc-headers-$CUDA_PKG_EXT=$CUDA_VERSION-1 && \
    ln -s cuda-$CUDA_VERSION_MAJOR /usr/local/cuda && \
    ln -s /usr/local/cuda-8.0/targets/x86_64-linux/include /usr/local/cuda/include && \
    rm -rf /var/lib/apt/lists/*

# nvidia-docker 1.0
LABEL com.nvidia.volumes.needed="nvidia_driver"
LABEL com.nvidia.cuda.version="${CUDA_VERSION}"
RUN echo "/usr/local/nvidia/lib" >> /etc/ld.so.conf.d/nvidia.conf && \
    echo "/usr/local/nvidia/lib64" >> /etc/ld.so.conf.d/nvidia.conf

ENV PATH=/usr/local/nvidia/bin:/usr/local/cuda/bin:${PATH} \
    LD_LIBRARY_PATH=/usr/local/nvidia/lib:/usr/local/nvidia/lib64${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}

# nvidia-container-runtime
ENV NVIDIA_REQUIRE_CUDA="cuda>=$CUDA_VERSION_MAJOR"

# install packages
RUN apt-get update && apt-get install -q -y \
    dirmngr \
    gnupg2 \
    lsb-release sudo autoconf automake libtool \
&& rm -rf /var/lib/apt/lists/*

# Setup package sources
# setup keys
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# setup sources.list
RUN echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros-latest.list

# install bootstrap tools
RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install --no-install-recommends -y \
    python-rosdep \
    python-rosinstall \
    python-vcstools \
    && rm -rf /var/lib/apt/lists/*

# setup environment
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8

# bootstrap rosdep
RUN rosdep init \
    && rosdep update

# install aptitude
RUN apt-get update -q && \
    apt-get install -y aptitude

# install ros packages
ENV ROS_DISTRO melodic
RUN apt-get update -q  && \
    apt-get upgrade -yq && \
    apt-get install -yq wget curl git build-essential vim sudo lsb-release locales bash-completion
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu bionic main" > /etc/apt/sources.list.d/ros-latest.list'
RUN curl -k https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | sudo apt-key add -
RUN apt-get update -q && \
    apt-get install -y ros-${ROS_DISTRO}-desktop-full python-rosdep &&\
    apt-get install -y python-rosinstall python-rosinstall-generator python-wstool build-essential python-catkin-tools python3-vcstool &&\
    apt-get install -y ros-${ROS_DISTRO}-catkin &&\
    apt-get install -y ros-${ROS_DISTRO}-roscpp &&\
    apt-get install -y ros-${ROS_DISTRO}-gazebo-ros &&\
    rm -rf /var/lib/apt/lists/*

#python for ros
RUN pip install rospkg catkin_pkg

RUN rosdep update && \
    echo ". /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc && \
    . /opt/ros/${ROS_DISTRO}/setup.sh

# # for HOKUYO lidar
# RUN sudo apt-get install -y ros-${ROS_DISTRO}-urg-node
# # for gmapping
# RUN sudo apt-get install -y ros-${ROS_DISTRO}-slam-gmapping \
#  && sudo apt-get install -y ros-${ROS_DISTRO}-pointcloud-to-laserscan ros-${ROS_DISTRO}-rosbridge-server \
#  && sudo apt-get install -y ros-${ROS_DISTRO}-costmap-2d \
#  && sudo apt-get install -y ros-${ROS_DISTRO}-map-server
# # for localization
# RUN sudo apt-get install -y ros-${ROS_DISTRO}-geodesy ros-${ROS_DISTRO}-grid-map-ros \
#  && sudo apt-get install -y ros-${ROS_DISTRO}-nmea-msgs \
#  && sudo apt-get install -y ros-${ROS_DISTRO}-pcl-ros
# # for local planner
# RUN sudo apt-get install -y ros-${ROS_DISTRO}-move-base ros-${ROS_DISTRO}-teb-local-planner
# # for joystick
# RUN sudo apt-get install -y ros-${ROS_DISTRO}-joy
# # for cmd_vel mux
# RUN sudo apt-get install -y ros-${ROS_DISTRO}-yocs-cmd-vel-mux
# # for dependency
# RUN sudo apt-get install -y ros-${ROS_DISTRO}-wiimote ros-${ROS_DISTRO}-serial ros-${ROS_DISTRO}-depth-image-proc ros-${ROS_DISTRO}-compressed-depth-image-transport ros-${ROS_DISTRO}-compressed-image-transport

# for gazebo
# RUN sudo curl -sSL http://get.gazebosim.org | sh
# RUN apt-get install -y libgazebo9 gazebo9-common gazebo9 ros-${ROS_DISTRO}-gazebo-dev ros-${ROS_DISTRO}-gazebo-ros-pkgs ros-${ROS_DISTRO}-gazebo-ros-control

#RUN mkdir /cudaSamples
#COPY particles /cudaSamples/
#COPY deviceQuery /cudaSamples/

SHELL ["/bin/bash", "-c"]
#RUN cd /usr/lib/aarch64-linux-gnu && rm -f libGL.so && sudo ln -sf tegra/libGL.so libGL.so
#RUN ln -s /usr/lib/aarch64-linux-gnu/libv4l2.so.0.0.999999 /usr/lib/aarch64-linux-gnu/libv4l2.so
