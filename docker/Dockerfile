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

# Install Python dependencies for runtime nodes
# requirements.txt will be copied from build context root via docker-compose
COPY requirements.txt /tmp/requirements.txt
RUN pip3 install --no-cache-dir -r /tmp/requirements.txt || true

# Environment setup (source ROS always; source devel only if it exists at runtime)
RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc && \
    echo "[ -f /home/dev_ws/devel/setup.bash ] && source /home/dev_ws/devel/setup.bash" >> /root/.bashrc

CMD ["/bin/bash"]
