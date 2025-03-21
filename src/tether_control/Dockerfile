FROM ros:humble-ros-base

## generic dependencies
RUN apt-get update \
 && apt-get -y install \
    build-essential \
    cmake \
    git \
    nano

# ROS2 dependencies
RUN apt-get update \
 && apt-get -y install \
    ros-humble-asio-cmake-module \
    ros-humble-ros-gz

ARG USER_ID
ARG GROUP_ID
ARG USER

# Need to add user to be able to communicate with the host
RUN groupadd -g 1000 groundstation && \
    useradd -m -u 1000 -g 1000 groundstation

ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp

COPY . /home/workspace/

RUN chown -R groundstation:groundstation /home/workspace

WORKDIR /home/workspace

RUN chmod +x entrypoint.sh /home/workspace/entrypoint.sh

ENTRYPOINT ["/bin/bash", "/home/workspace/entrypoint.sh"]