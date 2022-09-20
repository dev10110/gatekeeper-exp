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

RUN echo "source /opt/ros/galactic/setup.bash" >> /root/.bashrc


