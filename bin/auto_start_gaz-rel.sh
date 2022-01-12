#!/bin/bash -e
#add .config/autostart/your sciript
xhost +
gnome-terminal -- bash -c "sudo -S ./dockerRun.sh -n -c gaz_rel -b network raslaunch.sh gaz; bash"
gnome-terminal -- bash -c "sudo -S docker exec -it gaz_rel bash; bash"


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
