FROM chenrc98/ros2-px4-pi:version1.1

SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y --no-install-recommends vim tmux

RUN apt-get install -y --no-install-recommends \
  ros-galactic-realsense2-camera

RUN apt-get install -y --no-install-recommends \
  libboost-all-dev

RUN apt-get install -y --no-install-recommends \
  libpcl-dev 

RUN apt-get install -y --no-install-recommends \
  ros-galactic-pcl-conversions

RUN apt-get install -y --no-install-recommends \
  ros-galactic-octomap ros-galactic-octomap-mapping

RUN rm -rf /var/lib/apt/lists

# # install mavlink-router
# COPY ./mavlink-router /root/mavlink-router
# WORKDIR /root/mavlink-router
# RUN meson setup build .
# RUN ninja -C build install


RUN echo "source /opt/ros/galactic/setup.bash" >> /root/.bashrc
RUN echo "source /root/px4_ros_com_ros2/install/setup.bash" >> /root/.bashrc
RUN echo "alias bridge=micrortps_agent -d /dev/ttyTHS2 -b 921600 -n drone6" >> /root/.bashrc
