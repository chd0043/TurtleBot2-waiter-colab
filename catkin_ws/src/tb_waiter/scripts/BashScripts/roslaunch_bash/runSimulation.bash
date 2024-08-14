#!/bin/bash

###################################
# Run All together
###################################

cd 

clear
gnome-terminal -x bash /home/turtlebot/catkin_ws/src/tb_waiter/scripts/BashScripts/roslaunch_bash/launch_gazebo.bash
sleep 10
gnome-terminal -x bash /home/turtlebot/catkin_ws/src/tb_waiter/scripts/BashScripts/roslaunch_bash/launch_runspade.bash
sleep 10
gnome-terminal -x bash /home/turtlebot/catkin_ws/src/tb_waiter/scripts/BashScripts/roslaunch_bash/launch_service.bash
sleep 20
gnome-terminal -x bash /home/turtlebot/catkin_ws/src/tb_waiter/scripts/BashScripts/roslaunch_bash/launch_assigner.bash










