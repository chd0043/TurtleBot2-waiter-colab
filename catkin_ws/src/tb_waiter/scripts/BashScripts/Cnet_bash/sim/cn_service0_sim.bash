#!/bin/bash

###################################
# ROS/SPADE Network Configuration #
###################################
IP_ADDR=$(ifconfig  wlan0 | grep 'inet addr:'| cut -d: -f2 | awk '{ print $1}')

###################################
# SPADE
configure.py $IP_ADDR
###################################
export ROS_NAMESPACE='robot_0'
rosparam set robot_name 'robot_0'
rosparam set simulation 'True'
rosparam set host_ip $IP_ADDR
rosrun tb_waiter mainSimCN.py








