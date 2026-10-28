# ROS 2 Humble + MuJoCo + PAL Kangaroo simulation, all from ROS apt binaries.
# Copied from the SO-2 repo (docker/ros2_kangaroo/Dockerfile); here the launch file is copied last so editing it keeps the NEST layer.
# Base pinned by digest (osrf/ros:humble-desktop, built 2026-09-10).
FROM osrf/ros@sha256:fb07245b32187d74350be25323d8ad2f8ca5c25c325759911a1eff2267a49c1e

# kangaroo-simulation pulls kangaroo_mujoco, kangaroo_robot, pal_sea_arm, mujoco_ros2_control, mujoco_vendor (MuJoCo).
# kangaroo-moveit-config: PAL's launch resolves it even with moveit:=False but does not declare it.
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
       ros-humble-kangaroo-simulation ros-humble-kangaroo-moveit-config \
    && rm -rf /var/lib/apt/lists/*

# Ring network: NEST pinned to HEAD@41892a5 (docker/image.env), built from source; matplotlib for the figures.
RUN apt-get update && apt-get install -y --no-install-recommends \
      build-essential cmake cython3 libgsl-dev libltdl-dev libboost-dev libreadline-dev \
      python3-dev python3-numpy python3-scipy python3-pandas python3-colorcet python3-matplotlib git \
    && rm -rf /var/lib/apt/lists/*
RUN git clone https://github.com/nest/nest-simulator.git /tmp/nest-src \
    && git -C /tmp/nest-src checkout --detach 41892a50f95ddf50966e2371151b207d098e814a \
    && cmake -S /tmp/nest-src -B /tmp/nest-build -DCMAKE_INSTALL_PREFIX=/usr/local/nest \
    && cmake --build /tmp/nest-build -j"$(nproc)" && cmake --install /tmp/nest-build \
    && rm -rf /tmp/nest-build /tmp/nest-src
ENV PYTHONPATH=/usr/local/nest/lib/python3.10/site-packages

# Every interactive shell (including `docker exec`) gets ROS sourced.
RUN echo 'source /opt/ros/humble/setup.bash' >> /etc/bash.bashrc
ENV NVIDIA_DRIVER_CAPABILITIES=all

COPY kangaroo_sim.launch.py /opt/kangaroo_sim.launch.py
