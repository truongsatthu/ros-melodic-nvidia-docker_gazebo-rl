#!/bin/bash -e
#add .config/autostart/your sciript
xhost +
gnome-terminal -- bash -c "./dockerRun.sh -n -c rasmake -b network rasMake.sh rl; bash"


if [ -e ./shutdown_signal ]; then
	rm ./shutdown_signal
fi
touch ./shutdown_signal

while inotifywait -e CLOSE_WRITE ./shutdown_signal; do
	signal=$(cat ./shutdown_signal)
	if [ "$signal" = "true" ]; then
		echo "shutdown process after 10sec"
		sleep 10
		echo "nvidia" | sudo -S shutdown -h now
	fi
done
