FROM ros:noetic
# FROM osrf/ros:noetic-desktop-focal

# ENV NVIDIA_VISIBLE_DEVICES=all
# ENV NVIDIA_DRIVER_CAPABILITIES=all
# ENV NVIDIA_REQUIRE_CUDA "cuda>=10.6"

ARG USERNAME=vedant
ARG USER_UID=1000
ARG USER_GID=$USER_UID
ARG WORKSPACE_PATH=/workspaces/plan-percep-ws

COPY . /install/

RUN export DEBIAN_FRONTEND=noninteractive &&\
    /install/scripts/create-user.bash $USERNAME $USER_UID $USER_GID &&\
    /install/scripts/setup-ros-dev-environment.bash $USERNAME $WORKSPACE_PATH &&\
    rm -rf /install &&\
    rm -rf /var/lib/apt/lists/*

USER $USERNAME


ENV SHELL /bin/bash