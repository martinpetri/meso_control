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

RUN /bin/bash -c 'cd /workspaces/meso_control \
                    && source /opt/ros/foxy/setup.bash \
                    && colcon build'

ENTRYPOINT /bin/bash -c 'cd /workspaces/meso_control \
                    && source /opt/ros/foxy/setup.bash \
                    && source ./install/setup.bash \
                    && ros2 launch launch/small.py'