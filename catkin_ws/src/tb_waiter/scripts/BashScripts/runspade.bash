#!/bin/bash

###################################
# ROS/SPADE Network Configuration #
###################################
IP_ADDR=$(ifconfig  wlan0 | grep 'inet addr:'| cut -d: -f2 | awk '{ print $1}')
###################################
# SPADE
configure.py $IP_ADDR
runspade.py








