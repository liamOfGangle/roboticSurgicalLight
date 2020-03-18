import math
import numpy as np

"""
Utilities library for use with Automata's Eva arm.
"""

def npArrayCheck(listOrArray):
    """
    Checks if input is an np.ndarray, converts to np array if not.
    """
    if listOrArray is not np.ndarray:
        listOrArray = np.array(listOrArray)
    return listOrArray

def deg2rad(value):
    """ 
    Computationally quicker to use math lib on scalars and numpy lib on arrays
    """
    if np.isscalar(value):
        value = math.radians(value)
    else:
        value = npArrayCheck(value)
        value = np.deg2rad(value)
    return value

def loadLimits(path): # Probably not needed as can use evaSDK calc_pose_valid()
    """
    Loads limit file
    """
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
    v11 = focalPointCoordinates[0] + radius*math.sin(theta)*math.cos(phi)
    v12 = focalPointCoordinates[1] + radius*math.sin(theta)*math.sin(phi)
    v13 = focalPointCoordinates[2]+ radius*math.cos(theta)
    endEffectorCoordinates = np.array([v11, v12, v13])
    return endEffectorCoordinates

def obscureCheck(focalPointCoordinates, focalDistance, theta, phi, obstacleRadius, obstacleCentreCoordinates):
    """
    Calculates if endEffector to focalPoint vector intersects a spherical object.
    Uses formula found here - https://en.wikipedia.org/wiki/Line%E2%80%93sphere_intersection
    """
    # focalPointCoordinates = npArrayCheck(focalPointCoordinates)
    # obstacleCentreCoordinates = npArrayCheck(obstacleCentreCoordinates)

    originPoint = endEffectorPosition(focalPointCoordinates, focalDistance, theta, phi)
    unitVec = np.linalg.norm(focalPointCoordinates - originPoint)

    discriminant = np.dot(unitVec,(originPoint - obstacleCentreCoordinates))**2 - ((np.linalg.norm(originPoint - obstacleCentreCoordinates)**2) - obstacleRadius**2) 

    if discriminant >= 0: return True
    else: return False

def collisionCheck(centre1, centre2, radius1, radius2):
    """
    Check if a spherical object intersects another spherical object
    """
    dist = np.linalg.norm(centre1 - centre2)
    if dist <= (radius1 + radius2): return True
    else: return False


