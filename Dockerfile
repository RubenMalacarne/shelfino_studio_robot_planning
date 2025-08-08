FROM osrf/ros:humble-desktop-full

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive

# Create a user with the same UID and GID as the host user
ARG USERNAME=rosuser
ARG USER_UID=1000
ARG USER_GID=1000

# Install packages as root first
RUN apt-get update && \
    apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    nano \
    sudo \
    libpcl-dev \
    ros-humble-slam-toolbox \
    ros-humble-teleop-twist-joy \
    ros-humble-teleop-twist-keyboard \
    ros-humble-twist-mux \
    ros-humble-joint-state-publisher-gui \
    ros-humble-xacro \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-turtlebot3* \
    ros-humble-gazebo-* \
    ros-humble-ros-gz* \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-ros2-control \
    ros-humble-robot-state-publisher \
    ros-humble-gazebo-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-topic-tools && \
    rm -rf /var/lib/apt/lists/*

RUN pip3 install pyserial \
    flask \
    flask-ask-sdk \
    ask-sdk \
    notebook \
    pyyaml \
    xmlschema \
    shapely
    


# RUN pip3 install open3d
# Create the workspace directory
RUN mkdir -p /ros2_ws

# Create user and group
RUN groupadd --gid ${USER_GID} ${USERNAME} && \
    useradd --uid ${USER_UID} --gid ${USER_GID} -m ${USERNAME} && \
    usermod -aG sudo ${USERNAME} && \
    echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers && \
    chown -R ${USERNAME}:${USERNAME} /ros2_ws

# Switch to non-root user
USER ${USERNAME}

# Set the workspace directory
WORKDIR /ros2_ws

# Source the ROS 2 setup script in the user's bashrc
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc && \
    echo "source /usr/share/gazebo/setup.sh" >> ~/.bashrc

# Default command to run when the container starts
CMD ["bash"]

# to mantain ros domain id in your pc copy this inside bashrc: 
# gedit ~/.bashrc
# export ROS_DOMAIN_ID=42


# colcon build --symlink-install --parallel-workers 1 && source install/setup.bash 