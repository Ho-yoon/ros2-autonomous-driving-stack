FROM ros:humble-ros-base-jammy

# System dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-colcon-common-extensions \
    python3-rosdep \
    ros-humble-pcl-ros \
    ros-humble-cv-bridge \
    ros-humble-tf2-ros \
    ros-humble-tf2-geometry-msgs \
    ros-humble-robot-state-publisher \
    libgeographic-dev \
    libeigen3-dev \
    libopencv-dev \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ws

# Copy source
COPY src/ /ws/src/

# Install ROS dependencies
RUN . /opt/ros/humble/setup.sh && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y

# Build
RUN . /opt/ros/humble/setup.sh && \
    colcon build \
      --symlink-install \
      --cmake-args -DCMAKE_BUILD_TYPE=Release

COPY scripts/ /ws/scripts/

# Source workspace in every shell
RUN echo ". /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo ". /ws/install/setup.bash" >> /root/.bashrc

ENTRYPOINT ["/bin/bash", "-c", \
  ". /opt/ros/humble/setup.bash && . /ws/install/setup.bash && exec \"$@\"", "--"]
CMD ["ros2", "launch", "av_bringup", "full_stack.launch.py"]
