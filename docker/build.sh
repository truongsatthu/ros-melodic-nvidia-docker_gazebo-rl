#!/bin/bash

usage_exit() {
	echo "Usage: $0 [-n] [-c]" 1>&2
	echo "-n: nvidia"
	echo "-c: use cache for saving installation time"
	exit 1
}

while getopts gstcmnh OPT
do
	case $OPT in
		n) OS=nvidia
		   TARGET=nvidia
                   echo "set OS: nvidia"
		   DOCKER=docker
		   IMAGE_NAME=gazebos_rl
		   DOCKER_OS="Dockerfile"
		   ;;
		c) CACHE=true
		   echo "use chache option"
		   ;;
	esac
done

if [ $CACHE ]; then
	CACHE_OPT="--no-cache=false"
else
	CACHE_OPT="--no-cache=true"
fi

echo "$DOCKER build -f $DOCKER_OS --tag gazebos_rl-os ."
sudo $DOCKER build -f $DOCKER_OS --tag gazebos_rl-os .

FILE=Dockerfile$EXT
echo "$DOCKER build $CACHE_OPT -f $FILE --tag $IMAGE_NAME ./../"
sudo $DOCKER build $CACHE_OPT -f $FILE --tag $IMAGE_NAME ./../
