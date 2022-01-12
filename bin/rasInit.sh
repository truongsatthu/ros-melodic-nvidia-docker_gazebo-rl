#!/bin/bash
source /opt/ros/${ROS_DISTRO}/setup.bash
WORK_DIR=$HOME/ros-melodic-nvidia-docker/ros

cd $WORK_DIR/src
#catkin_init_workspace
cd $WORK_DIR
/sudo rosdep install --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} -y
