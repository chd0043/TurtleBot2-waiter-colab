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
import numpy as np
sys.path.append('../..')
directory = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
sys.path.append(str(directory))

import roslib; #roslib.load_manifest('tb_waiter')
import rospy
import tf
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

import spade
import time
from string import *
import threading
from random import randint 
from operator import sub
from robotCommands import robotCommands
from csvLoggerService import csvMaker, infoStruct

##############################################################################
# Global variables
NAME = ""; HOST = ""; TIME = 2.0; MAX_ASWERS_NUM = 2
if rospy.has_param('robot_name'):      NAME = str(rospy.get_param("robot_name"))#
if rospy.has_param('host_ip'):         HOST = str(rospy.get_param("host_ip"))
if rospy.has_param('service_timeout'): TIME = str(rospy.get_param("service_timeout"))
if rospy.has_param('max_num_aswer'):   MAX_ASWERS_NUM = str(rospy.get_param("max_num_aswer"))

##############################################################################

CHAR_HEADER = '\033[95m'
CHAR_BLUE = '\033[94m'
CHAR_GREEN = '\033[92m'
CHAR_YELLOW = '\033[93m'
CHAR_RED = '\033[91m'
ENDC = '\033[0m'

##############################################################################
##############################################################################
##############################################################################

class Service(spade.Agent.Agent):

    class cMainBehv(spade.Behaviour.Behaviour):#(spade.Behaviour.PeriodicBehaviour): 
        
        def _process(self):
            #
            if len(self.myAgent.lAnnounceHeard) > 0:
                time.sleep(TIME)
                self.myAgent.pActualPos = self.myAgent.robot.getPosActual()
                print self.myAgent.pActualPos 
                timeStamp = time.strftime("%H:%M:%S", time.gmtime())
                self.myAgent.LogInfoStruct.AnnounceHeardList = [timeStamp, self.myAgent.lAnnounceHeard.tolist()]
                self.rEvaluateProposal(MAX_ASWERS_NUM, self.myAgent.lAnnounceHeard, self.myAgent.pActualPos)  

##############################################################################

        def rEvaluateProposal(self, nMaxNumBid, lAnnounceHeard, pActualPos, lPosAssign = []):
            # Evaluate if Request satisfice criteria.
            lAnnounceHeard = self.rSortByEuclidianDist(lAnnounceHeard)
            for i in range(len(lAnnounceHeard)):
                stManagerName = lAnnounceHeard[i][0]
                if i < nMaxNumBid:
                    print CHAR_GREEN + "Accepted: " + str(stManagerName) + ", " + str(i) + ENDC
                    pGoalPosition = self.myAgent.rStringPosToFloats(lAnnounceHeard[i][1])
                    nOffer = self.myAgent.robot.rEstimateCost(pGoalPosition)
                    self.rSendProposalMsg(stManagerName,nOffer)
                else:
                    print CHAR_YELLOW + "Rejected: " + str(stManagerName) + ", " + str(i) + ENDC
                    self.rSendRefuseMsg(stManagerName)
            self.myAgent.lAnnounceHeard = np.array([])
                          
        def rSortByEuclidianDist(self, lCandidates):
            lCandidates = np.array(lCandidates).reshape(-1,3)
            lCandidates = lCandidates[np.argsort(lCandidates[:, 2])]
            return lCandidates
                          
##############################################################################

        def rSendRefuseMsg(self, stManagerName):
            stManagerName = str(stManagerName)
            msg = spade.ACLMessage.ACLMessage()  # Instantiate the message
            msg.setPerformative("inform")        # Set the "inform" FIPA performative
            msg.setOntology("rob_refuse")        # Set the ontology of the message content
            msg.setLanguage("OWL-S")             # Set the language of the message content
            msg.setContent("refuse")             # Set the message content
            msg.addReceiver(spade.AID.aid(stManagerName,["xmpp://"+stManagerName]))          
            self.myAgent.send(msg)
            #
            timeStamp = time.strftime("%H:%M:%S", time.gmtime())
            self.myAgent.LogInfoStruct.RefusedAdds.append([timeStamp, stManagerName])
            
        def rSendProposalMsg(self, stManagerName, nOffer):
            stManagerName = str(stManagerName)
            msg = spade.ACLMessage.ACLMessage()  # Instantiate the message
            msg.setPerformative("inform")        # Set the "inform" FIPA performative
            msg.setOntology("rob_offer")         # Set the ontology of the message content
            msg.setLanguage("OWL-S")             # Set the language of the message content
            msg.setContent(str(nOffer))          # Set the message content
            msg.addReceiver(spade.AID.aid(stManagerName,["xmpp://"+stManagerName]))            
            self.myAgent.send(msg)
            #
            timeStamp = time.strftime("%H:%M:%S", time.gmtime())
            self.myAgent.LogInfoStruct.AcceptedAdds.append([timeStamp, stManagerName, nOffer])

##############################################################################
##############################################################################

    class cListenAnnouncBehav(spade.Behaviour.EventBehaviour):
        #  
        def _process(self):
            # behaviour main loop   
            #(stManagerName, pGoalPosition) = self.rHearAnnoucement()
            #if ( self.myAgent.RobotFree == True ):
                self.rHearAnnoucement()
                print "self.myAgent.lAnnounceHeard: " 
                print self.myAgent.lAnnounceHeard

        def rHearAnnoucement(self):
            #print "This behaviour has been triggered by a message!"    
            msg = self._receive(block=True,timeout=10)
            stGoalPos = msg.getContent()
            nameID = msg.getSender()
            stManagerName = nameID.getName()
            #
            pGoalPosition = self.myAgent.rStringPosToFloats(stGoalPos)
            nDistance = self.myAgent.rEstimateDistance(pGoalPosition)
            lCandidate = np.array([stManagerName, stGoalPos, nDistance]) #aqui debe ir el costo que manda el robot
            self.rAddAnnounceHeard2List(lCandidate)
            #
            self.rSendReceivedFeedback(stManagerName)

        def rAddAnnounceHeard2List(self, Announce2Add):
          
            if len(self.myAgent.lAnnounceHeard) < 1:
               self.myAgent.lAnnounceHeard = Announce2Add
            else:    
               self.myAgent.lAnnounceHeard = np.vstack((self.myAgent.lAnnounceHeard, Announce2Add))

##############################################################################

        def rSendReceivedFeedback(self, stManagerName):
            stManagerName = str(stManagerName)
            msg = spade.ACLMessage.ACLMessage()  # Instantiate the message
            msg.setPerformative("inform")        # Set the "inform" FIPA performative
            msg.setOntology("request_received")  # Set the ontology of the message content
            msg.setLanguage("OWL-S")             # Set the language of the message content
            msg.setContent("Processing request") # Set the message content
            msg.addReceiver(spade.AID.aid(stManagerName,["xmpp://"+stManagerName]))          
            self.myAgent.send(msg)

##############################################################################
##############################################################################

    class cAssignMsgBehav(spade.Behaviour.EventBehaviour):
        # 
            def _process(self):
                # reads the received assign-msg from the AssignerAgent and add the item to a list.
                msg = self._receive(block=True,timeout=10)
                string_ = msg.getContent()
                nameID = msg.getSender()
                stManagerName = nameID.getName()
                posAssigned = self.myAgent.rStringPosToFloats(string_)
                #self.myAgent.lPosAssigned.append(pos)
                if ( self.myAgent.RobotFree == True ):
                    print "ASSIGNED " + stManagerName
                    timeStamp = time.strftime("%H:%M:%S", time.gmtime())
                    self.myAgent.LogInfoStruct.Assigment = [timeStamp, stManagerName,(posAssigned)]
                    #
                    self.myAgent.rAwardingResponseMsg(stManagerName, "Job_Accepted")
                    self.myAgent.RobotFree = False
                    status = self.rExecuteTask(posAssigned)
                    if status: self.myAgent.rAwardingResponseMsg(stManagerName, "Job_Finish")
                    if not status: self.myAgent.rAwardingResponseMsg(stManagerName, "Job_Fail")
                    #self.myAgent.RobotFree = True
                    self.rGoToHome(self.myAgent.pPosInicial)
                    self.myAgent.RobotFree = True
                else:
                    self.myAgent.rAwardingResponseMsg(stManagerName, "Job_Rejected")
                
            def rExecuteTask(self, posAssigned):
                #self.myAgent.lPosAssigned.sort(key=self.myAgent.rEuclidean)
                print CHAR_YELLOW + NAME + " assigments: " + ENDC
                print CHAR_YELLOW + str( posAssigned ) + ENDC
                
                tStart = time.time() 
                self.myAgent.robot.startMeasuringDistance()
                status = self.myAgent.robot.NavigateToGoal(posAssigned)
                tEnd = time.time()
                tCycleTime = tEnd - tStart
                distTraveled = self.myAgent.robot.getMeasuredDistance()
                print CHAR_YELLOW + "Distance traveled: " + str(distTraveled) + ENDC
                print CHAR_YELLOW + "Cycle time: " + str(tCycleTime) + ENDC
                
                self.myAgent.LogInfoStruct.Execution_Time = tCycleTime
                self.myAgent.LogInfoStruct.Execution_Distance = distTraveled
                self.myAgent.LogInfoStruct.Time = time.strftime("%d/%b/%Y %H:%M:%S", time.gmtime())
                self.myAgent.csvLogFile.addLineToCsv(self.myAgent.LogInfoStruct)
                self.myAgent.LogInfoStruct = infoStruct()
                self.myAgent.LogInfoStruct.Name = NAME
                
                posAssigned = []
                self.myAgent.bIgnoreRestriction = False
                return status
                
            def rGoToHome(self, pHome):
                #lPosAssigned = [pHome]
                self.myAgent.robot.NavigateToGoal(pHome)
                
##############################################################################

    def rAwardingResponseMsg(self, stManagerName, stResponse):
                stResponse = str(stResponse)
                stManagerName = str(stManagerName)
                msg = spade.ACLMessage.ACLMessage()  # Instantiate the message
                msg.setPerformative("inform")        # Set the "inform" FIPA performative
                msg.setOntology("robAwardResponce")  # Set the ontology of the message content
                msg.setLanguage("OWL-S")             # Set the language of the message content
                msg.setContent(stResponse)           # Set the message content
                msg.addReceiver(spade.AID.aid(stManagerName,["xmpp://"+stManagerName]))          
                self.send(msg)
                
##############################################################################
##############################################################################

    class cListenCancelMsgBehav(spade.Behaviour.EventBehaviour):
        
            def _process(self):
                self.myAgent.robot.cancelGoal()
                #self.myAgent.rAwardingResponseMsg(stManagerName, "Job_Cancel")
        
##############################################################################
##############################################################################

    def _setup(self):
        #agent constructor
        self.setDebugToScreen()
        print "Service Agent On"
        self.lPosAssigned =[]
        self.lAnnounceHeard = np.array([])
        self.pActualPos = (0,0)
        self.pPosInicial = (0,0)
        self.bDockBusy = True
        self.bIgnoreRestriction = False
        self.RobotFree = True
            
        # Goal Navigator object
        self.robot = robotCommands()
        
        # Log info
        self.LogInfoStruct = infoStruct()
        self.csvLogFile = csvMaker(NAME)
        self.LogInfoStruct.Name = NAME

        self.robot.rAutoDocking()

        # read odometer
        self.pActualPos = self.robot.getPosActual() 
        self.pPosInicial = self.pActualPos
        print "self.pActualPos: " + str(self.pActualPos)
        #
        template = spade.Behaviour.ACLTemplate()
        template.setOntology("posAnnoucement")
        t = spade.Behaviour.MessageTemplate(template)
        self.addBehaviour(self.cListenAnnouncBehav(),t)
        
        # message listener to announcement initialization
        template.setOntology("AwardPosition")
        t = spade.Behaviour.MessageTemplate(template)
        self.addBehaviour(self.cAssignMsgBehav(),t)
        
                # message cancel Jobs
        template.setOntology("cancelContract")
        t = spade.Behaviour.MessageTemplate(template)
        self.addBehaviour(self.cListenCancelMsgBehav(),t)
        
        #cMainBehv
        cyclicBehav = self.cMainBehv()
        self.addBehaviour(cyclicBehav, None)
        
    def takeDown(self):
        # agent destructor
        rospy.signal_shutdown(0)
        
    def rStringPosToFloats(self, stPos):
        # read string and translates it to numerical values.
        string_ = stPos.replace("(", ""); string_ = string_.replace(")", "")
        list_ = string_.split(",")
        x  = float(list_[0])
        y  = float(list_[1])
        posAnnounced = (x,y)
        return posAnnounced
    
        # Estimation function for the remaining distance to the goal.
    def rEstimateDistance(self, pGoalPosition):
        pActualPos = self.robot.getPosActual()
        xd = pGoalPosition[0] - pActualPos[0]
        yd = pGoalPosition[1] - pActualPos[1]
        # Euclidian Distance
        # d = math.sqrt(xd * xd + yd * yd)
        # Manhattan distance
        d = abs(xd) + abs(yd)
        # Chebyshev distance
        # d = max(abs(xd), abs(yd))
        return(d)
                
##############################################################################
##############################################################################
##############################################################################
