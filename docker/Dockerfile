FROM osrf/ros:noetic-desktop-full

# Copy scripts folder
COPY scripts/ /root/scripts/
WORKDIR /root/
RUN chmod a+x -R /root/scripts

# Env variables
ENV CATKIN_WS=/root/catkin_ws
ENV SCRIPTS_PATH=/root/scripts

# Run the general and ROS dep installation
RUN scripts/install_deps.sh

# Finally, build all the stuff we downloaded.
RUN scripts/build_ros.sh
