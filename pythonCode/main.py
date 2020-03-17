# MATLAB library
import matlab.engine 
# Python libraries
import math
import numpy as np
import time
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

handRadius = 0.1 # 10cm radius
endEffRadius = 0.1 # 10cm radius

serverIP = "143.167.157.45"
localIP = "143.167.158.31"
multicastIP = "239.255.42.99"
numberOfRigidBodies = 4

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

streamingData = streamingClient.buffer

"""
For this test, only intersted in coordinates, not rotation data
"""
streamingData = visUtils.extractCoordinates(streamingData)
oldStreamingData = streamingData.copy()

oldStreamingData = visUtils.roundToMillimeter(oldStreamingData)

# Move to focal point
robotBaseCoordinates = oldStreamingData[0]
focalPointCoordinates = oldStreamingData[1]

focalPointCoordinates = visUtils.translatePoint(robotBaseCoordinates, focalPointCoordinates)
focalPointCoordinates = visUtils.rotatePoint(focalPointCoordinates)

focalPointCoordinates = np.squeeze(np.asarray(focalPointCoordinates))

endEffPos = evaUtils.endEffectorPosition(focalPointCoordinates, focalDistance, theta, phi)

currentAngles = eva.data_servo_positions()

currentAngles = matlab.double(currentAngles)
endEffPos = matlab.double(endEffPos.tolist())

finalJointAngles = eng.evaIKSoln(endEffPos,theta,phi,currentAngles,nargout=1)

finalJointAngles = np.array(finalJointAngles._data).tolist()
finalJointAngles[5] = 0 

with eva.lock():
    eva.control_wait_for_ready()
    eva.control_go_to(finalJointAngles)

"""
Assuming focal point and robot base doesn't move for this test
"""

try:
    while True:
        streamingData = streamingClient.buffer
        streamingData = visUtils.extractCoordinates(streamingData)

        rightCoords = streamingData[2]
        rightCoords = visUtils.roundToMillimeter(rightCoords)
        
        # Translate/rotate coordinates
        rightCoords = visUtils.translatePoint(robotBaseCoordinates, rightCoords)
        rightCoords = visUtils.rotatePoint(rightCoords)

        if evaUtils.collisionCheck(rightCoords, endEffPos, handRadius, endEffRadius):
            print("Collision")
        else: print("No collision")

        time.sleep(1)

except KeyboardInterrupt:
    with eva.lock():
        eva.control_wait_for_ready()
        eva.control_home()