FROM wisevision/ros_with_wisevision_msgs:humble

WORKDIR /root/wisevision_gps_tools_ws

COPY . /root/wisevision_gps_tools_ws/src/wisevision_gps_tools


RUN apt-get update && \
    apt-get install -y libyaml-cpp-dev libjsoncpp-dev

RUN rosdep update --include-eol-distros && \
    rosdep install --from-paths src --ignore-src -r -y --rosdistro humble

RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
source /root/wisevision_msgs_ws/install/setup.bash && \
colcon build --symlink-install"

ENTRYPOINT ["/bin/bash", "-c", "source /root/wisevision_gps_tools_ws/install/setup.bash && ros2 run wisevision_gps_tools gps_device_manager_node"]