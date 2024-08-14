#! /usr/bin/python

##############################################################################
# Thesis project
# ITESM campus Monterrey
# MIT12, Autonomous Agents in Ambient Intelligence
#
# A00812686
##############################################################################

import sys, select, termios, tty
import os, inspect
import numpy as np
sys.path.append('../..')

import rospy
import spade
import time
from operator import sub
from string import *
from csvLoggerAssigner import csvMaker, infoStruct
import threading

##############################################################################
NAME = "a"; HOST = "hostname"; TIMEOUT = 5; SIM = "False"; REASSIGMENT_TIME = 50
if rospy.has_param('table_name'):        NAME    = str(rospy.get_param("table_name"))#
if rospy.has_param('host_ip'):           HOST    = str(rospy.get_param("host_ip"))
if rospy.has_param('assigner_timeout'):  TIMEOUT = str(rospy.get_param("assigner_timeout"))
#
HEADER = '\033[95m'
BLUE = '\033[94m'
GREEN = '\033[92m'
YELLOW = '\033[93m'
RED = '\033[91m'
ENDC = '\033[0m'

##############################################################################
##############################################################################
##############################################################################

class Assigner(spade.Agent.Agent):
    
    class cJobAssingerBeh(spade.Behaviour.Behaviour): #(spade.Behaviour.OneShotBehaviour):
              # Contractnet protocol            
            
        def _process(self):
            self.settings = termios.tcgetattr(sys.stdin)
            goal = (1.0,2.0)
            if   NAME == "table0":
                goal = (-3.280, 4.460) 
            elif NAME == "table1":
                goal = (-3.030, 0.820)
            elif NAME == "table2":
                goal = (-2.980,-1.000)
            elif NAME == "table3":
                goal = (-4.161,-5.884)
                
            if (self.waitUntilRequest() and len(self.myAgent.lJobAssignedTo)<1):# or \
               #(self.myAgent.bPeriodicReassigmentCNET and self.myAgent.bCnActiveMemory):
                self.myAgent.bCnActiveMemory = True
                self.myAgent.bPeriodicReassigmentCNET = False
                self.rContractNet(goal)
                time.sleep(1.0)
                if len(self.myAgent.lJobAssignedTo)==0:
                    print "Waiting for signal!"
        
        def rContractNet(self, pDesiredGoal):
            # ContractNet Manager
            step = 1
            while step <= 4 and step >= 1 and not rospy.is_shutdown():
                self.myAgent.bCNetInProcess = True
                if   step == 1: # Step 1
                        # Problem recognition
                        self.resetAllMemories()
                        self.myAgent.lCandidates = np.array([]) 
                        step = 2
                elif step == 2:  # Step 2
                        # announcement
                        self.rSendAnnounceTaskMsg(pDesiredGoal)
                        timeStamp = time.strftime("%d/%b/%Y %H:%M:%S", time.gmtime())
                        self.myAgent.LogInfoStruct.AnnounceTime = timeStamp
                        step = 3
                elif step == 3:  # Step 3
                        # Bidding
                        if self.waitUntilRobotsBidOrTimeOut(TIMEOUT):
                           step = 0
                           print RED+"Error. No one answer the CNET request."+ENDC
                        else:
                           step = 4
                elif step == 4:  # Step 4
                        # Awarding Contract
                        print " "; print "Proposals: "; print self.myAgent.lCandidates
                        self.myAgent.LogInfoStruct.lCandidates = self.myAgent.lCandidates.tolist()
                        step = self.rAwardContract(pDesiredGoal)
                else:         # No Step
                        # default
                        step = 0
                        print RED+"Error. Unknown step."+ENDC
                        timecount = 0
            # End While
            self.myAgent.bCNetInProcess = False
            timeStamp = time.strftime("%d/%b/%Y %H:%M:%S", time.gmtime())
            self.myAgent.LogInfoStruct.Time = timeStamp
            self.myAgent.csvLogFile.addLineToCsv(self.myAgent.LogInfoStruct)
            self.myAgent.LogInfoStruct = infoStruct()
            self.myAgent.LogInfoStruct.Name = NAME
##############################################################################

        def rAwardContract(self, pDesiredGoal):
            if self.myAgent.bAnyRobotBid and len(self.myAgent.lCandidates) > 0:
               bJobAssigned = False
               nIndex = 0
               while ( bJobAssigned == False and not rospy.is_shutdown()):
                    winner = self.rEvaluateProposal(self.myAgent.lCandidates, nIndex)
                    bWinnerChange = self.rPreviousWinnerChange(winner) ###
                    print "bWinnerChange: " +str(bWinnerChange)
                    #if not bWinnerChange: step = 5; bJobAssigned = True; break
                    print GREEN + "winner: " + str(winner) + ENDC
                    self.rSendAwardContractMsg(winner, pDesiredGoal)
                    
                    while not (self.myAgent.bAwardedContractRejected or self.myAgent.bAwardedContractAccepted): pass
                    # TODO: rospy.is_shutdown()
                    timeStamp = time.strftime("%H:%M:%S", time.gmtime())
                    
                    if self.myAgent.bAwardedContractRejected and len(self.myAgent.lCandidates)>nIndex+1:
                         print str(winner) + ": winner denied."
                         self.myAgent.LogInfoStruct.winnerDenied.append([timeStamp,winner])
                         nIndex = nIndex+1
                         bJobAssigned = False
                         
                    elif self.myAgent.bAwardedContractAccepted:
                         #if bWinnerChange and len(self.myAgent.lJobAssignedTo)>0: 
                         #       print "cancel to previous contractor"
                         #       self.rSendCancelContractMsg(self.myAgent.lJobAssignedTo[0])
                         self.myAgent.lJobAssignedTo = [winner]
                         print str(winner) + ": winner accept"
                         self.myAgent.LogInfoStruct.winnerAccepted.append([timeStamp,winner])
                         bJobAssigned = True
                         step = 5
                         
                    else:
                         print "retry..."
                         bJobAssigned = True
                         step = 1
                    # Reset all memories
                    self.resetAllMemories()

            elif self.myAgent.bSomeRobotRejectToBid:
                bJobAssigned = True
                step = 1
                
            return step
        
        def rPreviousWinnerChange(self, winner):
            if len(self.myAgent.lJobAssignedTo):
                if (self.myAgent.lJobAssignedTo[0] == winner):
                    return False
                else:
                    return True
            else:
                return False
                
        def rSendAnnounceTaskMsg(self, pDesiredGoal):
            print "announce task."
            #
            stGoalPos = str(pDesiredGoal)
            msg = spade.ACLMessage.ACLMessage()  # Instantiate the message
            msg.setPerformative("inform")        # Set the "inform" FIPA performative
            msg.setOntology("posAnnoucement")    # Set the ontology of the message content
            msg.setLanguage("OWL-S")             # Set the language of the message content
            msg.setContent(stGoalPos)            # Set the message content
            msg.addReceiver(spade.AID.aid("robot_0@"+HOST,["xmpp://robot_0@"+HOST])) 
            msg.addReceiver(spade.AID.aid("robot_1@"+HOST,["xmpp://robot_1@"+HOST]))
            msg.addReceiver(spade.AID.aid("robot_2@"+HOST,["xmpp://robot_2@"+HOST]))
            msg.addReceiver(spade.AID.aid("robot_3@"+HOST,["xmpp://robot_3@"+HOST]))   
            self.myAgent.send(msg)
            
        def rSendAwardContractMsg(self, stWinner, stGoalPos):
            msg = spade.ACLMessage.ACLMessage()  # Instantiate the message
            msg.setPerformative("inform")        # Set the "inform" FIPA performative
            msg.setOntology("AwardPosition")     # Set the ontology of the message content
            msg.setLanguage("OWL-S")             # Set the language of the message content
            msg.setContent(str(stGoalPos))       # Set the message content
            msg.addReceiver(spade.AID.aid(str(stWinner),["xmpp://"+str(stWinner)]))
            self.myAgent.send(msg)        

        def rSendCancelContractMsg(self, stReceiver):
            stReceiver = str(stReceiver)
            msg = spade.ACLMessage.ACLMessage()  # Instantiate the message
            msg.setPerformative("inform")        # Set the "inform" FIPA performative
            msg.setOntology("cancelContract")     # Set the ontology of the message content
            msg.setLanguage("OWL-S")             # Set the language of the message content
            msg.setContent(str('Cancel'))       # Set the message content
            msg.addReceiver(spade.AID.aid(stReceiver,["xmpp://"+stReceiver]))
            self.myAgent.send(msg)        
               
        def rEvaluateProposal(self, lCandidates, listPlace = 0):
            lCandidates = self.rSortBySecondColumn(lCandidates)
            return lCandidates[listPlace][0]

        def rSortBySecondColumn(self, lCandidates):
            lCandidates = np.array(lCandidates).reshape(-1,2)
            lCandidates = lCandidates[np.argsort(lCandidates[:, 1])]
            return lCandidates
        
        def resetAllMemories(self):
            self.myAgent.bAnyRobotBid = False
            self.myAgent.bSomeRobotRejectToBid = False
            self.myAgent.bAwardedContractAccepted = False
            self.myAgent.bAwardedContractRejected = False
            self.myAgent.bRequestHeardBySomeone = False
            self.myAgent.numOfRobotsThatHeardReq = 0
            self.myAgent.numOfRobotsThatBid = 0
            
        def waitUntilRobotsBidOrTimeOut(self, timeout):
            time_count = 0
            while not ( ((self.myAgent.numOfRobotsThatHeardReq  == self.myAgent.numOfRobotsThatBid) and 
                         (self.myAgent.bAnyRobotBid or self.myAgent.bSomeRobotRejectToBid)) or 
                        (time_count >= timeout) ):
                    time.sleep(0.1)
                    time_count += 0.1
            return not (self.myAgent.bAnyRobotBid or self.myAgent.bSomeRobotRejectToBid)

        def waitUntilRequest(self):
            # 
            if SIM == 'True':
                if (self.myAgent.bSimMonitorRequest == True):
                    self.myAgent.bSimMonitorRequest = False
                    return True
                else:
                    return False
            else:
                key = self.getKey()
                if (key == ''):
                    return False
                else:
                    if (key == '\x03'): sys.exit(0)
                    return True
            
        def getKey(self):
            tty.setraw(sys.stdin.fileno())
            rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
            if rlist:
                key = sys.stdin.read(1)
            else:
                key = ''
        
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
            return key
##############################################################################
##############################################################################

    class cRecvProposMsgBehav(spade.Behaviour.EventBehaviour):
        # 
        def _process(self):
            # read and interprete incoming message            
            msg = self._receive(block=True,timeout=10)
            stCostProposal = msg.getContent()
            nameID = msg.getSender()
            stServName = nameID.getName()
            # Add to de candidate list
            lCandidate = np.array([stServName, stCostProposal]).reshape(-1,2) #aqui debe ir el costo que manda el robot
            self.rAddCandidates2List(lCandidate)
            # memories
            self.myAgent.bAnyRobotBid = True
            self.myAgent.numOfRobotsThatBid += 1
  
        def rAddCandidates2List(self, Candidate2Add):
            if len(self.myAgent.lCandidates) < 1:
               self.myAgent.lCandidates = Candidate2Add
            else:
               self.myAgent.lCandidates = np.vstack((self.myAgent.lCandidates, Candidate2Add))

##############################################################################
##############################################################################

    class cRecvRefusalMsgBehav(spade.Behaviour.EventBehaviour):
        # 
        def _process(self):
            # read and interprete incoming message            
            msg = self._receive(block=True,timeout=10)
            stContent = msg.getContent()
            nameID = msg.getSender()
            stServName = nameID.getName()
            # Add to de candidate list
            self.myAgent.bSomeRobotRejectToBid = True
            self.myAgent.numOfRobotsThatBid += 1

##############################################################################
##############################################################################
    
    class cRecvAwardResponce(spade.Behaviour.EventBehaviour):
        #
        def _process(self):
            msg = self._receive(block=True,timeout=10)
            string_ = msg.getContent()
            nameID = msg.getSender()
            stServName = nameID.getName()
            #self.myAgent.bAnyRobotBid = True
            if (string_ == "Job_Accepted"):
                self.myAgent.bAwardedContractAccepted = True
            if (string_ == "Job_Rejected"):
                self.myAgent.bAwardedContractRejected = True
            if (string_ == "Job_Finish"):
                self.myAgent.lJobAssignedTo = []
                self.myAgent.bCnActiveMemory = False
            if (string_ == "Job_Fail"):
                self.myAgent.lJobAssignedTo = []

##############################################################################
##############################################################################
    
    class cRecvMyAnnounceHeardByRobot(spade.Behaviour.EventBehaviour):
        #
        def _process(self):
            msg = self._receive(block=True,timeout=10)
            nameID = msg.getSender()
            stServName = nameID.getName()
            print "My message was heard by " +str(stServName)
            #
            self.myAgent.bRequestHeardBySomeone = True
            self.myAgent.numOfRobotsThatHeardReq += 1

##############################################################################
##############################################################################
    class cPeriodicBehav(spade.Behaviour.PeriodicBehaviour):
        
        def onStart(self):
            print "Starting behaviour . . ."
            self.counter = 0

        def _onTick(self):
            #print "Counter:"+ str(self.counter)
            #self.counter = self.counter + 1
            self.myAgent.bPeriodicReassigmentCNET = True

##############################################################################
##############################################################################

    class cSimMonitorRequest(spade.Behaviour.EventBehaviour):
        #
        def _process(self):
            #msg = self._receive(block=True,timeout=10)
            #nameID = msg.getSender()
            #stServName = nameID.getName()
            print "Monitor trigger me. "
            #
            self.myAgent.bSimMonitorRequest = True
            
##############################################################################
##############################################################################
         
    def _setup(self):
        #time.sleep(2)
        print "AssignerAgent starting . . ."
        self.bAnyRobotBid = False
        self.bSomeRobotRejectToBid = False
        self.bAwardedContractAccepted = False
        self.bAwardedContractRejected = False
        self.bRequestHeardBySomeone = False
        self.bPeriodicReassigmentCNET = False
        self.bCNetInProcess = False
        self.bSimMonitorRequest = False
        self.bCnActiveMemory = False
        self.numOfRobotsThatHeardReq = 0
        self.numOfRobotsThatBid = 0
        
        self.lCandidates = np.array([])
        self.lJobAssignedTo = []
        self.LogInfoStruct = infoStruct()
        self.LogInfoStruct.Name = NAME
        self.csvLogFile = csvMaker(NAME)

        JobAssinger = self.cJobAssingerBeh()
        self.addBehaviour(JobAssinger, None)
        
        # message listener initialization
        template = spade.Behaviour.ACLTemplate()
        template.setOntology("rob_offer")
        t = spade.Behaviour.MessageTemplate(template)
        self.addBehaviour(self.cRecvProposMsgBehav(),t)
                
        # message listener initialization
        template.setOntology("rob_refuse")
        t = spade.Behaviour.MessageTemplate(template)
        self.addBehaviour(self.cRecvRefusalMsgBehav(),t)

        template.setOntology("robAwardResponce")
        t = spade.Behaviour.MessageTemplate(template)
        self.addBehaviour(self.cRecvAwardResponce(),t)

        template.setOntology("request_received")
        t = spade.Behaviour.MessageTemplate(template)
        self.addBehaviour(self.cRecvMyAnnounceHeardByRobot(),t)
        
        template.setOntology("Monitor_request")
        t = spade.Behaviour.MessageTemplate(template)
        self.addBehaviour(self.cSimMonitorRequest(),t)
        
        b = self.cPeriodicBehav(REASSIGMENT_TIME)
        self.addBehaviour(b, None)
        
        print "Press any key to request service . . ."
##############################################################################
##############################################################################
##############################################################################

if __name__ == "__main__":
    AssignerAgent = Assigner(NAME + "@" + HOST, "secret")
    #AssignerAgent.setDebugToScreen()
    print NAME+"@"+HOST
    AssignerAgent.start()
    alive = True

    while alive:
        try:
            time.sleep(0.1)
        except KeyboardInterrupt:
            alive=False
    AssignerAgent.stop()
    sys.exit(0)
        
