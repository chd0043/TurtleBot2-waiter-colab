#! /usr/bin/python

##############################################################################
# Thesis project
# ITESM campus Monterrey
# MIT12, Autonomous Agents in Ambient Intelligence
#
# A00812686
##############################################################################

import os, inspect,sys
import time
import csv
import numpy as np

##############################################################################
##############################################################################

class infoStruct():
    def __init__(self):
        self.Time = ""
        self.Name = ""
        self.AnnounceTime = ""
        self.lCandidates = []
        self.winnerAccepted = []
        self.winnerDenied = []

##############################################################################
##############################################################################

class csvMaker:
    def __init__(self, AgentName=""):
        self.Num = 0
        AgentName = str(AgentName)
        FolderPath = str(os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe()))))
        TimeStamp = time.strftime("%d%b%Y_%H:%M:%S", time.gmtime())
        
        self.FilePath = FolderPath+'/log/log_'+TimeStamp+'_'+AgentName+'.csv'

        with open(self.FilePath, 'wb') as csvfile:
            self.spamwriter = csv.writer(csvfile, delimiter=',',
                                         quotechar=',', quoting=csv.QUOTE_MINIMAL)
            self.spamwriter.writerow(['Num.']+['Time']+['Name']+['AnnounceTime'] + 
                                     ['lCandidates']+['winnerAccepted']+['winnerDenied'])
            
    def prepareStrings(self, input):
        string_ = str(input)
        string_ = string_.replace(",", " | ")
        return string_

    def addLineToCsv(self, data):
        with open(self.FilePath, 'a') as csvfile:
            self.spamwriter = csv.writer(csvfile, delimiter=',',
                                         quotechar=' ', quoting=csv.QUOTE_MINIMAL)
            
            lCandidates = self.prepareStrings( data.lCandidates )
            winnerAccepted = self.prepareStrings( data.winnerAccepted )
            winnerDenied = self.prepareStrings( data.winnerDenied )
            
            self.spamwriter.writerow([self.Num]+[data.Time]+[data.Name]+[data.AnnounceTime]+ 
                                     [lCandidates]+[winnerAccepted]+[winnerDenied])
        
        self.Num = self.Num + 1


##############################################################################
##############################################################################
