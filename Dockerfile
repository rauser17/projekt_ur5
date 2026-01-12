FROM ros:jazzy-ros-base

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
    ros-jazzy-desktop \
    ros-jazzy-ur-description \
    ros-jazzy-cv-bridge \
    ros-jazzy-image-publisher \
    ros-jazzy-image-tools \
    ros-jazzy-xacro \
    python3-colcon-common-extensions \
    python3-pip \
    python3-opencv \
    x11-apps \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /root/ros2_ws

COPY src ./src
COPY test_obraz.png /root/ros2_ws/test_obraz.png

RUN . /opt/ros/jazzy/setup.sh && \
    colcon build

RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc
RUN echo "source /root/ros2_ws/install/setup.bash" >> /root/.bashrc

CMD ["bash"]
