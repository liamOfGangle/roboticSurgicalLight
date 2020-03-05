import math
import numpy as np

"""
Utilities library for use with Automata's Eva arm.
"""

def deg2rad(value):
    """ 
    Computationally quicker to use math lib on scalars and numpy lib on arrays
    """
    if np.isscalar(value):
        value = math.radians(value)
    else:
        value = np.deg2rad(value)
    return value

def loadLimits(path):
    datasheetLimits = np.genfromtxt(path, delimiter=",", usecols=(1,2))
    datasheetLimits = deg2rad(datasheetLimits)
    return datasheetLimits

def limitCheck(jointAngles, datasheetLimits):
    """
    Keep the joints +/- 3 degrees away from datasheet limits as joint angles 
    can change up to degree if power is removed.
    A precaution so eva arm doesn't move past it's hardware limit lock.
    """
    offset = deg2rad(3.0) # 3 degrees in radians

    for i in range(len(jointAngles)):
        if jointAngles[i] <= datasheetLimits[i][0]:
            jointAngles[i] = datasheetLimits[i][0] + offset
        elif jointAngles[i] >= datasheetLimits[i][1]:
            jointAngles[i] = datasheetLimits[i][1] - offset
    return jointAngles

def endEffectorPosition(focalPointCoordinates, radius, theta, phi):
    """
    Calculates end effector position in cartesian space from spherical coordinates
    """
    endEffectorCoordinates = np.zeros(3)
    endEffectorCoordinates[0] = focalPointCoordinates[0] + radius*math.sin(theta)*math.cos(phi)
    endEffectorCoordinates[1] = focalPointCoordinates[1] + radius*math.sin(theta)*math.sin(phi)
    endEffectorCoordinates[2] = focalPointCoordinates[2] + radius*math.cos(theta)
    return endEffectorCoordinates



     