# Matlab library
import matlab.engine
# Naturalpoints adapted NatNet library
from NatNetClientAdapted import NatNetClient
# Eva SDK
from evasdk import Eva
# Custom utilities library
import utilities as utils
# Python libraries 
import math
import numpy as np
import time

"""
Setup
"""
eng = matlab.engine.start_matlab() # Start MATLAB engine
eng.matEVASetup(nargout=0)         # Load variables into MATLAB workspace

# Connect to Eva robotic arm
hostIP = "172.16.16.2"
token = "d8b5bffcb19a1bbbe3da4772c143e364b54b42b2"
eva = Eva(hostIP, token) # EVA class 

# NatNetClient constants
serverIP = "143.167.157.45"
localIP = "143.167.158.31"
multicastIP = "239.255.42.99"
numberOfRigidBodies = 4 # Change depending on how many rigid bodies

dataStream = NatNetClient(serverIP, localIP, multicastIP, numberOfRigidBodies) # NatNet class
dataStream.run() # Run seperate thread receiving new data

# Robot position variables
# Initially is directly above focal point at a distance of one meter
fDist = 1            # One meter
theta = 0.0
phi = 0.0
robotHomePose = [0, 0.5235, -1.7453, 0, -1.9198, 0]

# Spherical object radii 
handRadius = 0.1 # 10cm
eeRadius = 0.125 # 12.5cm

# Length of predicted direction vector
length = 0.3 # 30cm

"""
Main
"""
waitInput = input("Press enter to return arm to home position") # Homes the robot if not already homed
with eva.lock():
    eva.control_wait_for_ready()
    eva.control_go_to(robotHomePose, mode='teach')

input("Press enter to start main program")
time.sleep(10)

# Move to above focal point 
visionData = dataStream.dataBuffer
coordsData = utils.extractCoordinates(visionData) # Data always in the from [robotCoords, focalCoords, object1Coords, ..., objectNCoords]

fDist, theta, phi, eeCoords = utils.calcEndEff(coordsData[1], fDist) # Get eeCoord and angles

currentAngles = eva.data_servo_positions() # Get servo angles

jointAngles = utils.calcJointAngles(theta, phi, eeCoords, currentAngles, eng)

with eva.lock():
    eva.control_go_to(jointAngles)

time.sleep(5)

eeCoords = coordsData[1] + np.array([0,0,1])
currentAngles = eva.data_servo_positions() # Get servo angles
theta = phi = 0.0

jointAngles = utils.calcJointAngles(theta, phi, eeCoords, currentAngles, eng)

with eva.lock():
    eva.control_go_to(jointAngles)

time.sleep(5)

with eva.lock():
    eva.control_go_to(robotHomePose)

    


# visionData = dataStream.dataBuffer # Get data from motive each run of while loop
# visionData = utils.extractCoordinates(visionData)
# objectCoords = visionData[2]
# objectCoords = utils.transRot(visionData[0], objectCoords) # Transform to robot frame

# try:
#     while True:
        
#         # visionData = dataStream.dataBuffer # Get data from motive each run of while loop
#         # visionData = utils.extractCoordinates(visionData)
#         # newObjectCoords = visionData[2] 
        
#         # newObjectCoords = utils.transRot(visionData[0], objectCoords) # Transform to robot frame
                
#         # dirVector = length*(np.linalg.norm(newObjectCoords - objectCoords)) # Direction vector of length 'length' in robot coords frame
        
#         # if utils.isPointInsideSphere(dirVector, eeCoords, eeRadius):
#         #     print("Predicted point is inside sphere")
#         #     # Move if point is inside sphere
#         #     newEndEffCoords = eeCoords.copy() + dirVector
#         #     newEndEffCoords = matlab.double(newEndEffCoords.tolist())

#         #     currentAngles = eva.data_servo_positions() # Get servo angles 
#         #     currentAngles = matlab.double(currentAngles)

#         #     jointAngles = eng.evaIKSoln(eeCoords, theta, phi, currentAngles, nargout=1) # Use IK solver to get joint positions
#         #     jointAngles = np.array(jointAngles._data).tolist()
#         #     jointAngles[5] = 0 # Convert back to list and joint 6 never needs to move

#         #     with eva.lock():
#         #         eva.control_go_to(jointAngles)
            
#         # objectCoords = newObjectCoords.copy()
        
# except KeyboardInterrupt:
#     with eva.lock():
#         eva.control_wait_for_ready()
#         eva.control_home()