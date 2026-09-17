ARG ROS_IMAGE=ros:jazzy-ros-base
FROM ${ROS_IMAGE}

USER root
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake git pkg-config sudo \
    python3-colcon-common-extensions python3-rosdep python3-vcstool \
    python3-pip python3-venv \
    ros-jazzy-bondcpp ros-jazzy-naoqi-libqi \
    ros-jazzy-yasmin ros-jazzy-yasmin-ros \
    libsqlite3-dev libtinyxml2-dev libyaml-cpp-dev uuid-dev \
    && rm -rf /var/lib/apt/lists/*

RUN mkdir -p /workspace/src /workspace/build /workspace/install /workspace/log \
    && chown -R ubuntu:ubuntu /workspace \
    && printf 'ubuntu ALL=(ALL) NOPASSWD:ALL\n' > /etc/sudoers.d/ubuntu \
    && chmod 0440 /etc/sudoers.d/ubuntu \
    && printf '\nsource /opt/ros/jazzy/setup.bash\nif [ -f /workspace/install/setup.bash ]; then source /workspace/install/setup.bash; fi\n' >> /home/ubuntu/.bashrc

USER ubuntu
RUN rosdep update --rosdistro jazzy
WORKDIR /workspace
CMD ["sleep", "infinity"]
