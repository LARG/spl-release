#!/bin/bash

# This script is called by systemd ut_motion service. It is meant to
# start bin/lola, fork the process, and then exit

source /etc/profile

/home/nao/bin/lola &>> motionoutput.txt &
