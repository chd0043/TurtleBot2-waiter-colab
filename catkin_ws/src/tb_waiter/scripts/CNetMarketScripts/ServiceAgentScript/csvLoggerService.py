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
        self.Time = 0
        self.Name = ""
        self.AnnounceHeardList = []
        self.AcceptedAdds = []
        self.RefusedAdds = []
        self.Assigment = []
        self.Execution_Time = 0
        self.Execution_Distance = 0

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
            self.spamwriter.writerow(['Num.']+['Time']+['Name']+['AnnounceHeardList'] + ['AcceptedAdds']+
                                     ['RefusedAdds']+['Assigment']+['Execution_Time']+['Execution_Distance'])
            
    def prepareStrings(self, input):
        string_ = str(input)
        string_ = string_.replace(",", " | ")
        return string_

    def addLineToCsv(self, data):
        with open(self.FilePath, 'a') as csvfile:
            self.spamwriter = csv.writer(csvfile, delimiter=',',
                                         quotechar=' ', quoting=csv.QUOTE_MINIMAL)
            
            AnnounceHeardList = self.prepareStrings( data.AnnounceHeardList )
            AcceptedAdds = self.prepareStrings( data.AcceptedAdds )
            RefusedAdds = self.prepareStrings( data.RefusedAdds )
            Assigment = self.prepareStrings( data.Assigment )
            
            self.spamwriter.writerow([self.Num]+[data.Time]+[data.Name]+[AnnounceHeardList] + [AcceptedAdds]+
                                     [RefusedAdds]+[Assigment]+[data.Execution_Time]+[data.Execution_Distance])
        
        self.Num = self.Num + 1

'''
if __name__ == "__main__":
    data1 =infoStruct()
    
    text = csvMaker()
    
    text.addLineToCsv(data1)
    data1.Name="robot_1"
    text.addLineToCsv(data1)
    data1.AnnounceHeardList = ('table0', 343, (12, 3) )
    text.addLineToCsv(data1)
'''

##############################################################################
##############################################################################
