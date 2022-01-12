#!/bin/bash

usage_exit() {
        echo "Usage: $0 rl [options]" 1>&2
        echo "rl robot's name : reinforcement learning"
        exit 1
}

if [ $# -lt 1 ]; then
        echo "Too short parameter"
        echo ""
        usage_exit
fi

case $1 in
        trs) PROC="roslaunch translation_node simulation.launch"
	        ;;
	gaz) PROC="roslaunch gazebos_rl oncloudsbot_v0.launch"
	        ;;
	rel) PROC="rosrun gazebos_rl pin_box_qtable.py"
	        ;;
	tes) PROC="rosrun gazebos_rl qtable_test.py"
	        ;;
        *) PROC=$1
            ;;
esac

if [[ $NAVIGATION ]]; then
	LAUNCH_DIR="$HOME/ros-melodic-nvidia-docker/ros/launch/$2/"
	echo "cd $LAUNCH_DIR"
	cd $LAUNCH_DIR
	PROC="roslaunch rl.launch"
	shift
fi

shift
OPT=$@

source /opt/ros/${ROS_DISTRO}/setup.bash
source $HOME/ros-melodic-nvidia-docker/ros/devel/setup.bash

if [ $ROS_MASTER_URI ] && [ $ROS_IP ]; then
	export ROS_MASTER_URI=$ROS_MASTER_URI
	export ROS_IP=$ROS_IP
        echo $ROS_MASTER_URI
        echo $ROS_IP
else
        export ROS_MASTER_URI=http://localhost:11311
        export ROS_IP=127.0.0.1
        echo $ROS_MASTER_URI
        echo $ROS_IP
fi

export LAUNCH_DIR=$LAUNCH_DIR
echo "$PROC $OPT"
$PROC $OPT
