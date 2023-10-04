FROM osrf/ros:noetic-desktop-full
ARG opencr_version="1.8.19"
# for avoidance of 'tzdata' configuring
ENV DEBIAN_FRONTEND=noninteractive
# change server to jp ones
RUN sed -i 's@archive.ubuntu.com@ftp.jaist.ac.jp/pub/Linux@g' /etc/apt/sources.list
# for GUI
ENV NVIDIA_DRIVER_CAPABILITIES ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics
RUN apt-get update -qq && apt-get install -y git vim wget python3-pip python3-dev -qq \
    && apt-get clean -qq && rm -rf /var/lib/apt/lists/*
RUN mkdir -p /root/catkin_ws/src
WORKDIR /root/catkin_ws
# RUN git clone https://github.com/UniversalRobots/Universal_Robots_ROS_Driver.git src/Universal_Robots_ROS_Driver\
#     && git clone -b calibration_devel https://github.com/fmauch/universal_robot.git src/fmauch_universal_robot
RUN apt-get update -qq && . /opt/ros/noetic/setup.sh \
    && rosdep update && rosdep install --from-paths src --ignore-src -y
RUN . /opt/ros/noetic/setup.sh && catkin_make
RUN echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
RUN . /root/catkin_ws/devel/setup.sh
ENV PATH /opt/ros/noetic/bin:$PATH
ENV PYTHONPATH /opt/ros/noetic/lib/python3/dist-packages
RUN pip install scipy jupyter
# For OpenCR
WORKDIR /root/tools
RUN wget https://downloads.arduino.cc/arduino-${opencr_version}-linux64.tar.xz && tar -xvf arduino-* && rm *.tar.xz\
    && ./arduino-${opencr_version}/install.sh\
    && dpkg --add-architecture i386 && apt-get update -qq \
    && apt-get install -y libxtst6 libncurses5-dev:i386 ros-noetic-rosserial-arduino ros-noetic-rosserial\
    && apt-get clean -qq && rm -rf /var/lib/apt/lists/*
ENV PATH $PATH:/root/tools/arduino-${opencr_version}
WORKDIR /root
# For Kinect-v2 (We may need to install driver on host OS)
RUN apt-get update -qq && \
    apt-get install -y -qq build-essential cmake pkg-config libusb-1.0-0-dev libturbojpeg0-dev libglfw3-dev \
                           libva-dev libjpeg-dev libopenni2-dev udev ffmpeg\
    && apt-get clean -qq && rm -rf /var/lib/apt/lists/*
RUN git clone https://github.com/OpenKinect/libfreenect2.git
WORKDIR /root/libfreenect2
RUN mkdir build && cd build && cmake .. -DCMAKE_INSTALL_PREFIX=$HOME/freenect2 && make && make install
RUN cp ./platform/linux/udev/90-kinect2.rules /etc/udev/rules.d/
RUN PKG_CONFIG_PATH=/root/freenect2/lib/pkgconfig pip install freenect2 \
    && ln -s /root/freenect2/lib/libfreenect2.so.0.2 /usr/lib/libfreenect2.so.0.2
RUN apt-get update -qq && apt-get install -y -qq python3-tk \
    && apt-get clean -qq && rm -rf /var/lib/apt/lists/*
RUN pip install dotmap

RUN apt-get update
RUN pip install ipdb 
RUN apt-get install -y tmux byobu 

# For Kinect-v2 Ros Package
RUN git clone https://github.com/paul-shuvo/iai_kinect2_opencv4.git  /root/catkin_ws/src/iai_kinect2
RUN rosdep install -r --from-paths /root/catkin_ws/src/iai_kinect2 

# WORKDIR /root/catkin_ws/
RUN  echo "cd  /root/catkin_ws;catkin_make -DCMAKE_BUILD_TYPE=Release -Dfreenect2_DIR=~/freenect2/lib/cmake/freenect2 -DCMAKE_CXX_STANDARD=14" >> ~/.bashrc

#hsr mode
# alias hsrb_mode='export ROS_MASTER_URI=http://192.168.11.5:11311 export PS1="\[\033[41;1;37m\]<hsrb>\[\033[0m\]\w$ "'

# RUN /bin/bash -c "source /opt/ros/noetic/setup.bash"
# RUN mkdir -p /root/catkin_ws/src
# RUN echo "source /root/catkin_ws/devel/setup.bash" >> ~/.bashrc
# RUN echo "cd /root" >> ~/.bashrc
RUN echo "export PYTHONPATH=$PYTHONPATH:/root/catkin_ws/src" >> ~/.bashrc
RUN echo "export ROS_MASTER_URI=http://192.168.11.5:11311" >> ~/.bashrc
RUN echo "export ROS_IP=192.168.11.6" >> ~/.bashrc
WORKDIR /root/catkin_ws/src
RUN echo "source /root/catkin_ws/devel/setup.bash" >> ~/.bashrc