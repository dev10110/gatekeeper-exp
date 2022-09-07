FROM chenrc98/ros2-px4-pi:version1.1

SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y --no-install-recommends vim tmux


WORKDIR /root/dasc/px4_ros_com_ros2

COPY  px4_ros_com_ros2/src/px4_ros_com ./src/px4_ros_com
COPY  px4_ros_com_ros2/src/px4_msgs    ./src/px4_msgs

WORKDIR "/root/dasc/px4_ros_com_ros2"

RUN source /opt/ros/galactic/setup.bash && colcon build --symlink-install 

RUN echo "source /opt/ros/galactic/setup.bash" >> ~/.bashrc
RUN echo "source /root/dasc/px4_ros_com_ros2/install/setup.bash" >> ~/.bashrc
RUN echo 'alias bridge="micrortps_agent -d /dev/ttyTHS2 -b 921600 -n ${ROBOT_NAME}"' >> ~/.bashrc
