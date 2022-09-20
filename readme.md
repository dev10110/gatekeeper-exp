## Setup
- make sure `docker` and `docker compose` are installed
- `git clone https://github.com/dasc-lab/robot-jumpstart`
- `cd robot-jumpstart`
- `git submodule init`
- `git submodule update`
- `docker compose build` (this step takes a while)
- if using PI:
  - change `Device` in `mavlink_router.conf` to `Device = /dev/ttyAMA2`
  - change `bridge -d` in `Dockerfile` to `-d /dev/ttyAMA1`
  - inn docker, change alias for bridge in .bashrc to /dev/ttyAMA1
- if using JETSON:
  - change `Device` in `mavlink_router.conf` to `Device = /dev/ttyUSB0`
  - change `bridge -d` in `Dockerfile` to `-d /dev/ttyTHS2`
  - in docker, change alias for bridge in .bashrc to `-d /dev/ttyTHS2`
- in `mavlink_router.conf`:
  - change the base station's IP address
- in `docker-compose.yaml`:
  - change the base station's IP address in `ROS_MASTER_IP` and `ROS_DISCOVERY_SERVER`
  - change the `ROBOT_NAME`
- in `super_client_configuration_file.xml`
  - change the base station's IP address


## Running Experiments:
- get stuff running on the ground station
- `docker compose up &`
- `docker exec -it robot-jumpstart-px4-1 bash`
- `colcon build --symlink-install`
- `source install/setup.bash
- `mavlink-routerd &`
- `bridge`
- setup GQC parameters according to https://dasc-lab.github.io/robot-framework/px4_robots/px4_rover_setup.html

