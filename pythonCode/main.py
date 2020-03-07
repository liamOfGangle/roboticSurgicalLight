# MATLAB library
import matlab.engine 

# Python math and numpy libraries
import math
import numpy as np

# Eva SDK and utilities libraries
from evasdk import Eva
import evaUtilities as utils

"""
Setup
"""
eng = matlab.engine.start_matlab() # Start MATLAB engine
eng.matEVASetup(nargout=0)         # Load variables into MATLAB workspace

limitsPath = "C:\\Users\\ljtov\\Documents\\roboticSurgicalLight\\jointInfo\\jointLimitsEVA.csv"
datasheetLimits = utils.loadLimits(limitsPath) # Load limits found in datasheet as a np array.

hostIP = "172.16.16.2"
token = "d8b5bffcb19a1bbbe3da4772c143e364b54b42b2"

eva = Eva(hostIP, token)

focalDistance = 0.3
# theta
# phi

focalPointCoordinates = np.array([-0.3, 0.3, -0.05])

"""
Main 
"""
waitInput = input("Press the enter key to continue.")

# Move to home position
with eva.lock():
    eva.control_wait_for_ready()
    eva.control_home()

try:
    while True:
        theta = input("Enter an angle between 0 and 90 degrees: ")
        theta = utils.deg2rad(float(theta))

        phi = input("Enter an angle between -180 and 180 degrees: ")
        phi = utils.deg2rad(float(phi))

        endEffPos = utils.endEffectorPosition(focalPointCoordinates, focalDistance, theta, phi)

        currentAngles = eva.data_servo_positions()

        currentAngles = matlab.double(currentAngles)
        endEffPos = matlab.double(endEffPos.tolist())

        finalJointAngles = eng.evaIKSoln(endEffPos,theta,phi,currentAngles,nargout=1)

        finalJointAngles = np.array(finalJointAngles._data).tolist()

        with eva.lock():
            eva.control_wait_for_ready()
            eva.control_go_to(finalJointAngles)

except KeyboardInterrupt:
    with eva.lock():
        eva.control_wait_for_ready()
        eva.control_home()
    pass