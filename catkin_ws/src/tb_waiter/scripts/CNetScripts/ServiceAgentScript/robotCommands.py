#!/usr/bin/env python

##############################################################################
# Thesis project
# ITESM campus Monterrey
# MIT12, Autonomous Agents in Ambient Intelligence
#
# A00812686
##############################################################################

'''
 -- Action Subscribed Topics
- move_base/goal (move_base_msgs/MoveBaseActionGoal)
A goal for move_base to pursue in the world.
- move_base/cancel (actionlib_msgs/GoalID)
A request to cancel a specific goal.

Action Published Topics
- move_base/feedback (move_base_msgs/MoveBaseActionFeedback)
Feedback contains the current position of the base in the world.
- move_base/status (actionlib_msgs/GoalStatusArray)
Provides status information on the goals that are sent to the move_base action.
- move_base/result (move_base_msgs/MoveBaseActionResult)
Result is empty for the move_base action.

 -- Subscribed Topics
- move_base_simple/goal (geometry_msgs/PoseStamped)
Provides a non-action interface to move_base for users that don't care about tracking the execution status of their goals.

 -- Published Topics 
- cmd_vel (geometry_msgs/Twist)
A stream of velocity commands meant for execution by a mobile base.
'''
##############################################################################

import roslib
import rospy
import tf
import sys
import numpy as np
#
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf.transformations import euler_from_quaternion
from operator import sub

# Brings in the SimpleActionClient
import actionlib
from move_base_msgs.msg import MoveBaseAction 
from move_base_msgs.msg import MoveBaseGoal
from kobuki_msgs.msg import AutoDockingAction
from kobuki_msgs.msg import AutoDockingGoal
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Quaternion

from tb_waiter.msg import *
from actionlib_msgs.msg import *

from tb_waiter.srv import *
##############################################################################
SIM =""; NAME = "a"; MARK =""
if rospy.has_param('simulation'):        SIM = str(rospy.get_param("simulation"))
if rospy.has_param('robot_name'):        NAME = str(rospy.get_param("robot_name"))#
if rospy.has_param('enable_landmark'):   MARK = str(rospy.get_param("enable_landmark"))#
##############################################################################
class robotCommands(object):

    def __init__(self, frameId = '/map'):
        self._GoalFrameId = frameId
        self.distInitial = 0
        
        self.MoveBaseCient = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        print "waiting for Move_Base server..."
        self.MoveBaseCient.wait_for_server()
        print "Move_Base Service Ok..."
        
        if (SIM == 'True'): return
        # Construct action ac
        print "Starting turtlebot_move action client..."
        self.tbclient = actionlib.SimpleActionClient('turtlebot_move', TurtlebotMoveAction)
        self.tbclient.wait_for_server()
        print "Action client connected to action server."
        
        print "Starting AutoDocking action client..."
        self.AutoDockClient = actionlib.SimpleActionClient('dock_drive_action', AutoDockingAction)
        self.AutoDockClient.wait_for_server()
        print "Move_Base Service Ok..."
        
##########################################################################################
#################### Move_base actions ###################################################
##########################################################################################
    def cancelGoal(self):
        self.kobukiCommand("cancelGoal")
        
    def NavigateToGoals(self, goals):
        for goal in goals:
            print("Moving to position " + str(goal))
            self.NavigateToGoal(goal)
            
            
    def NavigateToGoal(self, goal):
        self.moveBaseGoal = self._CreateMoveBaseGoal(goal)
        self.bMovementFinished = False
        self.retryCount = 0
        while not self.bMovementFinished:
            self.MoveBaseCient.send_goal(self.moveBaseGoal)
            rospy.on_shutdown(self.MoveBaseCient.cancel_all_goals)
            self.MoveBaseCient.wait_for_result()
            if self.MoveBaseCient.get_state() == GoalStatus.SUCCEEDED:
                self.bMovementFinished = True
                rospy.loginfo("rotation reached")
                return True
            else:
                rospy.logerr("Could not execute rotation to goal for some reason")
                #self.cancelGoal()
                #rospy.sleep(2)
                self.bMovementFinished = False
                #self.retryCount+=1
                #if self.retryCount > 4: 
                #    self.bMovementFinished=True
                #print "Retry num "+str(self.retryCount)
        return False
            
    def _CreateMoveBaseGoal(self, goal):
        
        (x,y) = goal
        theta = self.calculate_rot(goal)     

        moveBaseGoal = MoveBaseGoal()
        moveBaseGoal.target_pose.header.frame_id = self._GoalFrameId
        moveBaseGoal.target_pose.header.stamp = rospy.Time.now()
        
        moveBaseGoal.target_pose.pose.position.x = x
        moveBaseGoal.target_pose.pose.position.y = y
        
        quaternionArray = tf.transformations.quaternion_about_axis(theta, (0,0,1))
        # quaternion_about_axis offers a convenient way for calculating the members of a quaternion.
        # In order to use it we need to convert it to a Quaternion message structure
        moveBaseGoal.target_pose.pose.orientation = self.array_to_quaternion(quaternionArray)
        
        print (x,y,theta)
        return moveBaseGoal
    
    def calculate_rot(self, goal):
        position = self.getPosActual()
        (x, y) = map(sub, goal, position)
        angle = np.arctan2(y, x)
        return angle

    def array_to_quaternion(self, nparr):
        
        quat = Quaternion()
        quat.x = nparr[0]
        quat.y = nparr[1]
        quat.z = nparr[2]
        quat.w = nparr[3]
        return quat

##########################################################################################
#################### Turtlebot actions ###################################################
##########################################################################################

    def turtlebotMove(self, turn_distance, forward_distance):
         action_goal = TurtlebotMoveGoal()
         action_goal.turn_distance = turn_distance
         action_goal.forward_distance = forward_distance # metres

         if self.tbclient.send_goal_and_wait(action_goal, rospy.Duration(50.0), rospy.Duration(50.0)) == GoalStatus.SUCCEEDED:
            rospy.loginfo('Call to Turtlebot action server succeeded')
         else:
            rospy.logerr('Call to Turtlebot action server failed')
    
    def turn_left(self):
        turn_distance = np.pi / 2
        forward_distance = 0
        self.turtlebotMove(turn_distance,forward_distance)
    
    def turn_rigth(self):
        turn_distance = - np.pi / 2
        forward_distance = 0
        self.turtlebotMove(turn_distance,forward_distance)
    
    def move_fwd(self, forward_distance):
        turn_distance = 0
        self.turtlebotMove(turn_distance,forward_distance)

##########################################################################################
#################### Turtlebot services ##################################################
########################################################################################## 

    def getPosActual(self):
        (x,y,theta, dist) = self.readRobPos("amcl")
        return (x,y)

    def readRobPos(self, infoSource):

        rospy.wait_for_service('RobPosInfo')
        try:
           position = rospy.ServiceProxy('RobPosInfo', RobPosInfo)
           resp1 = position(infoSource)
           return (resp1.x, resp1.y, resp1.theta, resp1.dist)
        except rospy.ServiceException, e:
           print "Service call failed: %s"%e
           
    def rEstimateCost(self, pGoalPosition):
        rospy.wait_for_service('RobPlanDist')
        (x,y) = pGoalPosition
        vel = float(rospy.get_param("move_base/TrajectoryPlannerROS/max_vel_x"))
        try:
            dist = rospy.ServiceProxy('RobPlanDist', RobPlanDist)
            resp = dist(x,y)
            print "plan len: " + str(resp.dist)
            timeStimate = resp.dist / vel
            return timeStimate
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
    
    def resetOdom(self):
        self.kobukiCommand("reset_Odom")
        
    def setInitialPose(self, num):
        if num == 1:
            self.kobukiCommand("initialpose1")
        if num == 2:
            self.kobukiCommand("initialpose2")
        else:
            print "Unknown."
 
    def kobukiCommand(self, command):
        command = str(command)
        rospy.wait_for_service('RobResetPos')
        try:
           robReset = rospy.ServiceProxy('RobResetPos', RobResetPos)
           result = robReset(command) # 1 = set Robot to initial Pos 1
           print "command Ok."
        except rospy.ServiceException, e:
           print "Service call failed: %s"%e

##########################################################################################
######################### Mark recognition service  ######################################
##########################################################################################

    def getLandMark(self, string_=""):
        rospy.wait_for_service('cvMarkRec')
        try:
           readLandMark = rospy.ServiceProxy('cvMarkRec', cvMarkRec)
           landmark = readLandMark(string_)
           return (landmark.figure, landmark.color)
        except rospy.ServiceException, e:
           print "Service call failed: %s"%e
           
    def readRefLandMark(self):
        mark = self.getLandMark
        print "figure = "+str(mark[0])+", color = "+str(mark[1])
        if ( mark[0] == "triangle"):
            refPosition = 1
        if ( mark[0] == "circle"):
            refPosition = 2
        else:
            refPosition = 1
        return refPosition
    
##########################################################################################
######################### Auto Docking  ##################################################
##########################################################################################

    def doneCb(self, status, result):
      if 0: print ''
      elif status == GoalStatus.PENDING   : state='PENDING'
      elif status == GoalStatus.ACTIVE    : state='ACTIVE'
      elif status == GoalStatus.PREEMPTED : state='PREEMPTED'
      elif status == GoalStatus.SUCCEEDED : state='SUCCEEDED'
      elif status == GoalStatus.ABORTED   : state='ABORTED'
      elif status == GoalStatus.REJECTED  : state='REJECTED'
      elif status == GoalStatus.PREEMPTING: state='PREEMPTING'
      elif status == GoalStatus.RECALLING : state='RECALLING'
      elif status == GoalStatus.RECALLED  : state='RECALLED'
      elif status == GoalStatus.LOST      : state='LOST'
      # Print state of action server
      print 'Result - [ActionServer: ' + state + ']: ' + result.text

    def activeCb(self):
      if 0: print 'Action server went active.'

    def feedbackCb(self,feedback):
      # Print state of dock_drive module (or node.)
      print 'Feedback: [DockDrive: ' + feedback.state + ']: ' + feedback.text

    def dock_drive_client(self):

      goal = AutoDockingGoal();
      self.AutoDockClient.send_goal(goal, self.doneCb, self.activeCb, self.feedbackCb)
      print 'Goal: Sent.'
      rospy.on_shutdown(self.AutoDockClient.cancel_goal)
      self.AutoDockClient.wait_for_result()
      #print '    - status:', client.get_goal_status_text()
      return self.AutoDockClient.get_result()

    def docking_sequence(self):
      try:
        self.dock_drive_client()
        #print ''
        #print "Result: ", result
      except rospy.ROSInterruptException: 
        print "program interrupted before completion"         

##########################################################################################
            
    def rAutoDocking(self):
        
        if (SIM == 'True'): return True
        print "Waiting to be autodocked..."
        self.docking_sequence()
        # Set initial position 
        self.resetOdom()
        refPosition = 1
        if (MARK == 'True'): refPosition = readRefLandMark
        self.setInitialPose(refPosition)
        rospy.sleep(0.5)
        #
        self.move_fwd(-0.2)
        
        print "name: " + NAME
        if   NAME == "robot_0":
            posIni=(-0.5, 0.5)
        elif NAME == "robot_1":
            posIni=(-0.5,-0.5)
        elif NAME == "robot_2":
            posIni=(-0.5, 1.0)
        elif NAME == "robot_3":
            posIni=(-0.5,-1.0)
        
        self.turn_left()
        self.turn_left()
        
        self.NavigateToGoal(posIni)
        return True