FROM ros:foxy

RUN apt-get update \
    && apt-get -y install --no-install-recommends python3-pip vim less ros-foxy-rosbridge-server \
    && apt-get autoremove -y \
    && apt-get clean -y \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install requests pymodbus==2.5.3
RUN mkdir -p /workspaces

WORKDIR /workspaces

RUN git clone https://github.com/martinpetri/meso_control.git
RUN git checkout feature-osbk_library
WORKDIR /workspaces/meso_control

RUN echo "source /opt/ros/foxy/setup.bash && source ./install/setup.bash" >> ~/.bashrc

RUN /bin/bash -c 'cd /workspaces/meso_control \
                    && source /opt/ros/foxy/setup.bash \
                    && colcon build'

ENTRYPOINT /bin/bash -c 'cd /workspaces/meso_control \
                    && source /opt/ros/foxy/setup.bash \
                    && source ./install/setup.bash \
                    && ros2 launch meso_control_pkg local_sim_launch.py'