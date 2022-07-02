FROM ros:foxy

RUN apt-get update \
    && apt-get -y install --no-install-recommends python3-pip less \
    && apt-get autoremove -y \
    && apt-get clean -y \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install requests pymodbus
RUN mkdir -p /workspaces
WORKDIR /workspaces
RUN git clone https://github.com/martinpetri/meso_control.git
WORKDIR /workspaces/meso_control
#RUN bin/bash source /opt/ros/foxy/setup.bash
#RUN colcon build
#RUN ./install/setup.sh
# export PATH=$PATH:/workspaces/meso_control/install/meso_control_pkg/lib/meso_control_pkg && ros2 launch launch/small.py"
