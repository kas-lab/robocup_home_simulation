# Use the official ROS 2 Humble base image with Gazebo Fortress pre-installed
FROM osrf/ros:humble-desktop-full

# Set environment variables for non-interactive installation
ENV DEBIAN_FRONTEND=noninteractive

ENV WORKSPACE_DIR=/kaslab_robocup_ws

# Update and install additional dependencies if needed
RUN apt-get update && apt-get install -y \
    ros-humble-gazebo-ros-pkgs \
    python3-vcstool \
    python3-rosdep \
    ignition-fortress \
    vim \
    && rm -rf /var/lib/apt/lists/*

# Create workspace directories
RUN mkdir -p $WORKSPACE_DIR/src

# Copy dependencies.repos into the container
COPY general_dependencies.repos $WORKSPACE_DIR/
COPY mirte_dependencies.repos $WORKSPACE_DIR/

# Use vcs to import and clone all packages
RUN git clone -b fortress https://github.com/kas-lab/robocup_home_simulation.git $WORKSPACE_DIR/src/robocup_home_simulation
RUN vcs import $WORKSPACE_DIR/src < $WORKSPACE_DIR/general_dependencies.repos
RUN vcs import $WORKSPACE_DIR/src < $WORKSPACE_DIR/mirte_dependencies.repos
RUN rm -f $WORKSPACE_DIR/*.repos
RUN touch $WORKSPACE_DIR/src/mirte-ros-packages/mirte_telemetrix_cpp/COLCON_IGNORE

# Initialize rosdep
# RUN rosdep update && apt update

# Source ROS 2 setup script, install dependencies, and build the workspace
WORKDIR $WORKSPACE_DIR
# RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
#     rosdep install --from-paths src --ignore-src -r -y"

RUN ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash \
    && apt update \
    && rosdep update \
    && rosdep install --from-paths src --ignore-src -r -y \
    && sudo rm -rf /var/lib/apt/lists/"]

RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
    colcon build --symlink-install"

# RUN rm -rf /var/lib/apt/lists/*

# Set the default entrypoint
ENTRYPOINT ["/bin/bash"]