#!/bin/bash

###################################
# ROS/SPADE Network Configuration #
###################################
IP_ADDR=$(ifconfig  wlan0 | grep 'inet addr:'| cut -d: -f2 | awk '{ print $1}')

###################################
# SPADE
configure.py $IP_ADDR
###################################
rosparam set robot_name 'robot_0'
rosparam set simulation 'False'
rosparam set host_ip $IP_ADDR
rosparam set agent_ip $IP_ADDR
rosrun tb_waiter mainSimCN.py








