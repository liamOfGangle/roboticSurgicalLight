import math
import numpy as np
from NatNetClient import NatNetClient

"""
Utilities library for use with vision systems (this case is Opitrack's Motive).
Listener functions and functions to translate/rotate etc.
"""

def npArrayCheck(listOrArray):
    """
    Checks if input is an np.ndarray, converts to np array if not.
    """
    if listOrArray is not np.ndarray:
        listOrArray = np.array(listOrArray)
    return listOrArray

def translatePoint(robotBaseCoordinates, rigidBodyCoordinate):
    """
    x' = x - h, y' = y - k, z' = z - l 	
    Translate points from vision system coordinates to robot base coordinates.
    Robot base is (0,0,0) for all calculations related to the robot. 
    """
    # robotBaseCoordinates = npArrayCheck(robotBaseCoordinates)
    # rigidBodyCoordinate = npArrayCheck(rigidBodyCoordinate)
    
    translatedRigidBodyCoordinates = rigidBodyCoordinate - robotBaseCoordinates
    return translatedRigidBodyCoordinates
     
    
def rotatePoint(robotBaseQuaternion, rigidBodyCoordinate):
    """
    Rotate points from vision system coordinates to robot base coordinates
    """
    # q = npArrayCheck(robotBaseQuaternion)
    # rigidBodyCoordinate = npArrayCheck(rigidBodyCoordinate)

    # q = robotBaseQuaternion

    # idx = np.array([3, 1, 2, 0])
    # q = q[idx] # Rearrange so w component is first in array

    # # Taken from https://www.thepoorengineer.com/en/quaternion/
    # # First row of transformation matrix
    # m11 = q[0]**2 + q[1]**2 - q[2]**2 - q[3]**2
    # m12 = 2*(q[1]*q[2] - q[0]*q[3])
    # m13 = 2*(q[1]*q[3] + q[0]*q[2])
    # # Second row
    # m21 = 2*(q[1]*q[2] + q[0]*q[3])
    # m22 = q[0]**2 - q[1]**2 + q[2]**2 - q[3]**2
    # m23 = 2*(q[2]*q[3] - q[0]*q[1])
    # # Third row
    # m31 = 2*(q[1]*q[3] - q[0]*q[2])
    # m32 = 2*(q[2]*q[3] + q[0]*q[1])
    # m33 = q[0]**2 - q[1]**2 - q[2]**2 + q[3]**2
    # # Construct matrix
    # qMat = np.matrix([[m11, m12, m13],
    # [m21, m22, m23],
    # [m31, m32, m33]])

    # Rotation about x-axis matrix of 90 to get z-axis pointing in same direction as robot base
    rotX = np.matrix([[1.0, 0.0, 0.0],
                      [0.0, 0.0, -1.0],
                      [0.0, 1.0, 0.0]])

    # rotationMat = rotX@qMat # Complete rotation matrix
    rotationMat = rotX 

    finalRotCoordinate = rotationMat@rigidBodyCoordinate
    return finalRotCoordinate