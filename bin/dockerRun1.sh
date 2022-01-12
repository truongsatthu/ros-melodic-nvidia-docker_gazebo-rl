#!/bin/bash

usage_exit() {
	echo "Usage: $0 [-n] -c container_name -b network_name script_file_name.sh" 1>&2
	echo "-n: nvidia"
	exit 1
}

while getopts gntsc:b:d:h OPT
do
	case $OPT in
                n) TARGET=nvidia
                   echo "set docker command: nvidia"
                   DOCKER=nvidia-docker
                   IMAGE_NAME=gazebos_rl-os
		   USER=root
		   WORK_DIR=/$USER
                   ;;
		c) CONTAINER_NAME=$OPTARG
		   ;;
		b) NETWORK_NAME=$OPTARG
		   ;;
		d) HOST_DATA_DIR=$OPTARG
		   ;;
		h) usage_exit
		   ;;
		\?) usage_exit
		    ;;
	esac
done

if [ ! $TARGET ]; then
	echo "set docker command: docker ( general )"
	DOCKER=docker
	IMAGE_NAME=gazebos_rl
	USER=root
	WORK_DIR=/$USER
fi

if [ ! $CONTAINER_NAME ]; then
	usage_exit
fi

echo "Set Container name : $CONTAINER_NAME"

if [[ $HOST_DATA_DIR ]]; then
	if [ -d $HOST_DATA_DIR ]; then
		echo "Set Data dir : $HOST_DATA_DIR"
		VOL_DATA_DIR="--volume=${HOST_DATA_DIR}:/root/share:rw"
		echo $VOL_DATA_DIR
	fi
fi

xhost +

VAR_RUN=$(sudo docker ps --format {{.Names}} | grep -x $CONTAINER_NAME)
VAR_SLEEP=$(sudo docker ps -a --format {{.Names}} | grep -x $CONTAINER_NAME)

if [ "$VAR_RUN" = "$CONTAINER_NAME" ] || [ "$VAR_SLEEP" = "$CONTAINER_NAME" ]; then
	echo "Error: Docker componet is already existed, Please docker rm $CONTAINER_NAME"
	exit
fi

HOST_SRC_DIR=`readlink -f ../`
SRC_DIR=$WORK_DIR/ros-melodic-nvidia-docker

#DATA_DIR=/home/ros/share
#DATA_DIR=$HOST_DATA_DIR

shift $((OPTIND-1))
SCRIPT_NAME=$@

if [[ $SCRIPT_NAME ]]; then
	echo $SCRIPT_NAME
	PROC=$WORK_DIR/ros-melodic-nvidia-docker/bin/$SCRIPT_NAME
else
	PROC=/bin/bash
fi
echo $PROC

sudo docker network create \
	--driver=bridge \
	--subnet=172.11.2.0/16 \
	--ip-range=172.11.2.0/24 \
	--gateway=172.11.2.254 \
	${NETWORK_NAME} \

sudo $DOCKER run \
	-it --rm --privileged \
	--user ${USER} \
	--name=${CONTAINER_NAME} \
        --env="DISPLAY" \
        --env="QT_X11_NO_MITSHM=1" \
	--network=${NETWORK_NAME} \
	--env="ROS_MASTER_URI=http://172.11.2.0:11311" \
	--env="ROS_IP=172.11.2.0" \
	--env="OPENBLAS_CORETYPE=$CPU_CORE" \
	--volume="/media":"/media:rw" \
        --volume="/dev":"/dev:rw" \
        --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
	${VOL_DATA_DIR} \
	--volume=${HOST_SRC_DIR}:${SRC_DIR}:rw \
        $IMAGE_NAME $PROC

#	-it --rm --privileged --net=host\
#	--env="ROS_MASTER_URI=$ROS_MASTER_URI" \
#	--env="ROS_IP=$ROS_IP" \
