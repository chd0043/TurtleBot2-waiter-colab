#! /usr/bin/python

##############################################################################
# Thesis project
# ITESM campus Monterrey
# MIT12, Autonomous Agents in Ambient Intelligence
#
# A00812686
##############################################################################

import sys
import os, inspect
import math
sys.path.append('../..')
directory = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
sys.path.append(str(directory) + str("/CNetScripts/ServiceAgentScript")) #
#sys.path.append(str(directory) + str("/MovScripts")) #

##############################################################################

import roslib; roslib.load_manifest('tb_waiter')
import rospy

#import spade
import time
from string import *
import threading

from std_msgs.msg import String
from ServiceAgent import Service

##############################################################################

NAME = "a"; HOST = "hostname"
if rospy.has_param('robot_name'):   NAME = str(rospy.get_param("robot_name"))#
if rospy.has_param('host_ip'):      HOST = str(rospy.get_param("host_ip"))

##############################################################################
# Main
        
if __name__ == "__main__":
	# main routine
	rospy.init_node('service_'+ NAME )
   
	# Agent declaration
	print NAME+"@"+HOST
	serviceAgent = Service(NAME+"@"+ HOST, "secret")
	print "ServiceAgent On."
	serviceAgent.start()
	alive = True
	
	while alive and not rospy.is_shutdown():
	    try:
	        time.sleep(0.1)
	    except KeyboardInterrupt:
	        alive=False
	serviceAgent.stop()
	sys.exit(0)

##############################################################################
