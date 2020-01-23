#!/bin/bash

# This script is called by systemd ut_vision service. It is meant to
# start bin/vision, fork the process, and then exit

source /etc/profile

# Reset the cameras. This is necessary to prevent hardware bugs in
# vision. This may be fixed in a future SoftBank update, but for now,
# we do it manually.
/usr/libexec/reset-cameras.sh toggle
sleep 3

memory_found=false
# Make sure lola starts ok and the memory blocks are created.
# This doesn't create all memory blocks but does create the ones
# needed for interacting with the NAO's sensors and motors
for i in {1..60}
do
  /home/nao/bin/memory_test
  if [ $? -eq 0 ]; then
    memory_found=true
    break
  fi
  sleep 1
done

# If memory blocks were created fine then start other processes
if $memory_found; then
  /home/nao/bin/vision &>> visionoutput.txt &
  sleep 1
else
  # systemd will automatically try to restart lola. Just wait in case
  # that takes some time
  sleep 5
fi
