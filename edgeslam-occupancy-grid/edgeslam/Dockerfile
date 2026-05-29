FROM ubuntu:18.04

# Fix shell configurations to avoid future errors
RUN rm /bin/sh && ln -s /bin/bash /bin/sh

# Update ubuntu
RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y \
    build-essential \
    cmake \
    curl \
    gcc \
    git \
    libglew-dev \
    libgtk2.0-dev \
    pkg-config \
    ffmpeg \
    libavformat-dev \
    libavcodec-dev \
    libswscale-dev \
    python-dev \
    python-numpy \
    unzip \
    wget \
    libeigen3-dev \
    libboost-all-dev \
    lsb-release \
    ca-certificates \
    emacs

# Set dependencies as environment variables
ENV OPENCV_VERSION=3.4.2
ENV OPENCV_DIR=opencv-$OPENCV_VERSION
ENV BOOST_VERSION=1.65.1
ENV EIGEN_VERSION=3.3.4

# Install OpenCV
WORKDIR /home
RUN wget -O opencv.zip https://github.com/opencv/opencv/archive/${OPENCV_VERSION}.zip \
    && unzip opencv.zip \
    && cd $OPENCV_DIR \
    && mkdir release \
    && cd release \
    && cmake -D CMAKE_BUILD_TYPE=RELEASE -D CMAKE_INSTALL_PREFIX=/usr/local .. \
    && make \
    && make install

# Install Pangolin v0.5. Any later version leads to linking issues.
WORKDIR /home
RUN git clone --recursive https://github.com/stevenlovegrove/Pangolin.git -b v0.5 \
    && cd Pangolin \
    && mkdir build \
    && cd build \
    && cmake .. \
    && make

# Set up simlinks to Eigen3
RUN ln -s /usr/include/eigen3/Eigen /usr/local/include/Eigen

# Install ROS Melodic
WORKDIR ~/
RUN ln -snf /usr/share/zoneinfo/$CONTAINER_TIMEZONE /etc/localtime && echo $CONTAINER_TIMEZONE > /etc/timezone
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -
RUN apt update
RUN apt install -y ros-melodic-desktop-full
RUN echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
RUN source ~/.bashrc
RUN apt install python-rosdep
RUN rosdep init
RUN rosdep update

# Install iperf3.9
WORKDIR /home
RUN wget https://downloads.es.net/pub/iperf/iperf-3.9.tar.gz \
    && tar -xvf iperf-3.9.tar.gz \
    && cd iperf-3.9 \
    && ./configure \
    && sudo make \
    && sudo make install

# NOTE: THIS IS A HACK. Removing this line leads to missing library issues for iperf3.
RUN apt-get install -y iperf3

WORKDIR /home/edgeslam

COPY . .

# Build Edge-SLAM
RUN chmod +x build.sh && \
    ./build.sh

RUN echo "export ROS_PACKAGE_PATH=\${ROS_PACKAGE_PATH}:/home/edgeslam/Examples/ROS" >> ~/.bashrc

RUN source /opt/ros/melodic/setup.bash && \
    export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:/home/edgeslam/Examples/ROS && \
    bash build_ros.sh