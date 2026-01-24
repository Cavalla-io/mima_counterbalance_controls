FROM ros:jazzy-ros-base

# System dependencies
RUN apt-get update && apt-get install -y \
    tmux \
    python3-pip \
    ros-jazzy-nav2-bringup \
    ros-jazzy-slam-toolbox \
    ros-jazzy-robot-localization \
    && rm -rf /var/lib/apt/lists/*

# # Python packages - Method 1: requirements.txt
# COPY requirements.txt /tmp/requirements.txt
# RUN pip3 install --no-cache-dir -r /tmp/requirements.txt

RUN pip3 install --no-cache-dir --break-system-packages python-can numpy pyyaml

HEALTHCHECK --interval=30s --timeout=10s --retries=3 \
  CMD tmux list-sessions 2>/dev/null || exit 1

# Copy tmux config
COPY docker/.tmux.conf /root/.tmux.conf

# Create workspace
WORKDIR /ros_ws
COPY src/ /ros_ws/src/

# Build workspace
RUN . /opt/ros/jazzy/setup.sh && \
    colcon build --symlink-install

# Copy launch script
COPY launch.sh /launch.sh
RUN chmod +x /launch.sh

ENTRYPOINT ["/launch.sh"]
