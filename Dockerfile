FROM ros:noetic-ros-base

ARG DEBIAN_FRONTEND=noninteractive

# Install system packages & ROS dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-catkin-tools \
    ros-noetic-cv-bridge \
    ros-noetic-image-transport \
    ros-noetic-roslaunch \
    python3-opencv \
    libgl1 \
    python3-numpy \
    && rm -rf /var/lib/apt/lists/*

# Create catkin workspace
RUN mkdir -p /home/dev_ws/src
WORKDIR /home/dev_ws

# Environment setup
RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc && \
    echo "source /home/dev_ws/devel/setup.bash" >> /root/.bashrc

CMD ["/bin/bash"]
