# MATLAB library
import matlab.engine 
# Python math and numpy libraries
import math
import numpy as np
# Naturalpoints adapted NatNet library
from NatNetClient import NatNetClient
# Eva SDK Utilities libraries
from evasdk import Eva
# Utilities libraries
import evaUtilities as evaUtils
import visionSystemUtilities as visUtils

"""
Setup
"""
eng = matlab.engine.start_matlab() # Start MATLAB engine
eng.matEVASetup(nargout=0)         # Load variables into MATLAB workspace

limitsPath = "C:\\Users\\ljtov\\Documents\\roboticSurgicalLight\\jointInfo\\jointLimitsEVA.csv"
datasheetLimits = evaUtils.loadLimits(limitsPath) # Load limits found in datasheet as a np array.

hostIP = "172.16.16.2"
token = "d8b5bffcb19a1bbbe3da4772c143e364b54b42b2"
eva = Eva(hostIP, token) # EVA class 

focalDistance = 1
phi = evaUtils.deg2rad(0)
theta = evaUtils.deg2rad(0)

serverIP = "143.167.157.45"
localIP = "143.167.158.31"
multicastIP = "239.255.42.99"
numberOfRigidBodies = 2

streamingClient = NatNetClient(serverIP, localIP, multicastIP, numberOfRigidBodies) # NatNet class
streamingClient.run() # Run seperate thread receiving new data

"""
Main 
"""
waitInput = input("Press the enter key to move robot to home position.")

# Move to home position
with eva.lock():
    eva.control_wait_for_ready()
    eva.control_home()

try:
    while True:
        waitInput = input("Press enter to move robot to focal point.")
        with eva.lock():

            """
            Initial test is to run once through
            """
            streamingData = streamingClient.buffer
            robotRB = streamingClient.buffer[0]
            focalPointRB = streamingClient.buffer[1]

            focalPointCoordinates = np.array(list(focalPointRB[1]))
            robotBaseCoordinates = np.array(list(robotRB[1]))
            robotBaseQ = np.array(list(robotRB[2]))

            focalPointCoordinates = visUtils.translatePoint(robotBaseCoordinates, focalPointCoordinates)
            focalPointCoordinates = visUtils.rotatePoint(robotBaseQ, focalPointCoordinates)

            focalPointCoordinates = np.squeeze(np.asarray(focalPointCoordinates))

            endEffPos = evaUtils.endEffectorPosition(focalPointCoordinates, focalDistance, theta, phi)

            currentAngles = eva.data_servo_positions()

            currentAngles = matlab.double(currentAngles)
            endEffPos = matlab.double(endEffPos.tolist())

            finalJointAngles = eng.evaIKSoln(endEffPos,theta,phi,currentAngles,nargout=1)

            finalJointAngles = np.array(finalJointAngles._data).tolist()
            finalJointAngles[5] = 0 

            eva.control_wait_for_ready()
            eva.control_go_to(finalJointAngles)
except KeyboardInterrupt:
    with eva.lock():
        eva.control_wait_for_ready()
        eva.control_home()