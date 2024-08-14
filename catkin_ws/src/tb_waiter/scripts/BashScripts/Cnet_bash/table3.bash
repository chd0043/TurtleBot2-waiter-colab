#!/bin/bash

###################################
# ROS/SPADE Network Configuration #
###################################
IP_ADDR=$(ifconfig  wlan0 | grep 'inet addr:'| cut -d: -f2 | awk '{ print $1}')
#IP_ADDR='10.20.219.126'
NAME='table3'

###################################
# SPADE
configure.py $IP_ADDR
###################################
export ROS_NAMESPACE=$NAME
rosparam set table_name $NAME
rosparam set host_ip $IP_ADDR
rosrun tb_waiter AssignerAgent.py








