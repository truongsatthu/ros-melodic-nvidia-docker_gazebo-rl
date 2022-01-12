#!/bin/bash

usage_exit() {
        echo "Usage: $0 [-i ip] main|sub " 1>&2
        echo "main: nav cpu main"
        echo "sub: nav cpu sub"
	echo "-m: master ros ip"
        echo "-i: client ros ip"
        exit 1
}

OPTIND=1

#unset 
unset ROS_MASTER_URI
unset ROS_IP

#default
MAIN_IP=192.168.0.1
SUB_IP=192.168.1.2
PORT=11311

while getopts m:i:h OPT
do
        case $OPT in
		m) MAIN_IP=$OPTARG
		   ;;
                i) IP=$OPTARG
                   ;;
		\?) usage_exit
		    ;;
	esac
done

shift $((OPTIND-1))

case $1 in
        main) ROS_MASTER_URI=http://$MAIN_IP:$PORT
	      ROS_IP=$MAIN_IP
              ;;
        sub) ROS_MASTER_URI=http://$MAIN_IP:$PORT
	     ROS_IP=$SUB_IP
             ;;
esac

if [ $IP ]; then
	ROS_IP=$IP
fi

#not set
if [ ! $ROS_MASTER_URI ] && [ ! $ROS_IP ]; then
	ROS_MASTER_URI=http://localhost:11311
	ROS_IP=127.0.0.1
fi

echo "set ROS_MASTER_URI=$ROS_MASTER_URI"
echo "set ROS_IP=$ROS_IP"
export ROS_MASTER_URI=$ROS_MASTER_URI
export ROS_IP=$ROS_IP
