#!/bin/bash

killall -9 villa-start.sh &> /dev/null

# First try stopping through systemd
systemctl --user stop ut_motion.service
systemctl --user stop ut_vision.service

sleep 1

killall -9 /home/nao/bin/vision &> /dev/null
# Use SIGTERM (15) for lola so we can catch and terminate
# propertly (e.g., set stiffness to zero).
killall -15 /home/nao/bin/lola &> /dev/null
