#! /usr/bin/python

##############################################################################
# Thesis project
# ITESM campus Monterrey
# MIT12, Autonomous Agents in Ambient Intelligence
#
# A00812686
##############################################################################

import sys, select, termios, tty
import os, inspect, socket
import numpy as np
sys.path.append('../..')

import rospy
import spade
import time
from operator import sub
from string import *
import threading

##############################################################################
#
HEADER = '\033[95m'
BLUE = '\033[94m'
GREEN = '\033[92m'
YELLOW = '\033[93m'
RED = '\033[91m'
ENDC = '\033[0m'

##############################################################################
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
try:
    s.connect(("gmail.com",80))
    HOST = s.getsockname()[0]
    s.close()
except (socket.error):
    HOST = '127.0.0.1'
##############################################################################
##############################################################################
##############################################################################

class Monitor(spade.Agent.Agent):
    
    class cSimCoordinator(spade.Behaviour.Behaviour): #(spade.Behaviour.OneShotBehaviour):
              # Contractnet protocol
        def _process(self):
            
            if self.myAgent.counter < 50:
                self.testSequence1()
                self.myAgent.counter+=1
                print self.myAgent.counter
            
        
        def testSequence1(self, timer=20):
            # Todos al mismo tiempo
            time.sleep(timer)
            self.rForceToRequest('table0')
            self.rForceToRequest('table1')
            self.rForceToRequest('table2')
            self.rForceToRequest('table3')
            self.rForceToRequest('table4')
            
        def testSequence2(self, timer=5):
            # En orden ascendente cada 5 segundos
            time.sleep(timer)
            self.rForceToRequest('table0')
            time.sleep(timer)
            self.rForceToRequest('table1')
            time.sleep(timer)
            self.rForceToRequest('table2')

        def testSequence3(self, timer=5):
            # En orden ascendente cada 5 segundos
            time.sleep(timer)
            self.rForceToRequest('table2')
            time.sleep(timer)
            self.rForceToRequest('table1')
            time.sleep(timer)
            self.rForceToRequest('table0')
            
        def testSequence4(self, timer=5):
            # En orden ascendente cada 5 segundos
            time.sleep(timer)
            self.rForceToRequest('table1')
            time.sleep(timer)
            self.rForceToRequest('table2')
            time.sleep(timer)
            self.rForceToRequest('table0')
            
        def randomSequence1(self):
            pass
               #  break
        def rForceToRequest(self, stReceiver):
            stReceiver = str(stReceiver)
            msg = spade.ACLMessage.ACLMessage()         # Instantiate the message
            msg.setPerformative("inform")               # Set the "inform" FIPA performative
            msg.setOntology("Monitor_request")          # Set the ontology of the message content
            msg.setLanguage("OWL-S")                    # Set the language of the message content
            msg.setContent(str("Order a Coke now!"))    # Set the message content
            msg.addReceiver(spade.AID.aid(stReceiver+"@"+HOST,["xmpp://"+stReceiver+"@"+HOST]))
            self.myAgent.send(msg)        
      
    def _setup(self):

        self.counter = 0
        JobAssinger = self.cSimCoordinator()
        self.addBehaviour(JobAssinger, None)
        
##############################################################################
##############################################################################
##############################################################################

if __name__ == "__main__":
    MonitorAgent = Monitor("monitor" + "@" + HOST, "secret")
    MonitorAgent.setDebugToScreen()
    print "monitor"+"@"+HOST
    MonitorAgent.start()
    alive = True

    while alive:
        try:
            time.sleep(0.1)
        except KeyboardInterrupt:
            alive=False
    MonitorAgent.stop()
    sys.exit(0)
        
