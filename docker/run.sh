#!/bin/sh

usage_exit() {
        echo "Usage: $0 [-n] container name" 1>&2
	echo "-n: use nvidia-docker for RL"
        exit 1
}

while getopts mnh OPT
do
        case $OPT in
                n) NVIDIA=true
                   echo "set docker command: nvidia"
                   ;;
                h) usage_exit
                   ;;
                \?) usage_exit
                   ;;
        esac
done
shift $((OPTIND-1))

if [ $NVIDIA ]; then
        DOCKER=nvidia-docker
else
        DOCKER=docker
fi

VAR_RUN=$(sudo docker ps --format {{.Names}} | grep -x $1)
VAR_SLEEP=$(sudo docker ps -a --format {{.Names}} | grep -x $1)

if [ "$VAR_RUN" = "$1" ]; then
        sudo docker exec -it $1 bash
elif [ "$VAR_SLEEP" = "$1" ]; then
        sudo docker start -a -i $1
else
	#DATA
	HOST_DATA_DIR=/home/$USER/irmp_data
	DATA_DIR=/home/ros/irmp_data

	mkdir -p $HOST_DATA_DIR

	sudo $DOCKER run \
		-it --privileged --net=host \
		--name=${1} \
		--env="DISPLAY" \
		--env="QT_X11_NO_MITSHM=1" \
		--volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
		--volume=${HOST_DATA_DIR}:${DATA_DIR}:rw \
		$IMAGE:latest bash
fi
