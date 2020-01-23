#/bin/sh
if [ $1 = "start" ]; then
    ~/bin/villa-start.sh
elif [ $1 = "stop" ]; then
    ~/bin/villa-stop.sh
elif [ $1 = "restart" ]; then
    ~/bin/villa-stop.sh
    sleep 2
    ~/bin/villa-start.sh
elif [ $1 = "status" ]; then
    systemctl --user status ut_motion
    systemctl --user status ut_vision
else
    echo "Invalid command"
fi
