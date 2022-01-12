#!/bin/bash

usage_exit() {
        echo "Usage: $0 rl" 1>&2
        echo "rl: reinforcement learning"
        exit 1
}

if [ $# -ne 1 ]; then
	echo "set one option"
	echo ""
	usage_exit
fi

case $1 in
	rl) PROC=build_rl.sh
             ;;
	*) echo "Error option : $1"; usage_exit
	    ;;
esac

source /opt/ros/${ROS_DISTRO}/setup.bash
WORK_DIR=$HOME/ros-melodic-nvidia-docker/ros

cd $WORK_DIR
sudo rosdep install --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} -y
echo $PROC
./$PROC
