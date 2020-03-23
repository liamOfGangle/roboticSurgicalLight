"""
Library for useful functions for use with this project 
Eva, vision system and general functions
"""
import math
import numpy as np
from numpy import linalg as la
import operator


# General #
###########

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

def obscureCheck(focalPointCoordinates, focalDistance, theta, phi, obstacleRadius, obstacleCentreCoordinates):
    """
    Calculates if endEffector to focalPoint vector intersects a spherical object.
    Uses formula found here - https://en.wikipedia.org/wiki/Line%E2%80%93sphere_intersection
    """
    # focalPointCoordinates = npArrayCheck(focalPointCoordinates)
    # obstacleCentreCoordinates = npArrayCheck(obstacleCentreCoordinates)

    originPoint = endEffectorPosition(focalPointCoordinates, focalDistance, theta, phi)
    unitVec = la.norm(focalPointCoordinates - originPoint)

    discriminant = np.dot(unitVec,(originPoint - obstacleCentreCoordinates))**2 - ((la.norm(originPoint - obstacleCentreCoordinates)**2) - obstacleRadius**2) 

    if discriminant >= 0: return True
    else: return False

def collisionCheck(centre1, centre2, radius1, radius2):
    """
    Check if a spherical object intersects another spherical object
    """
    dist = la.norm(centre1 - centre2)
    if dist <= (radius1 + radius2): return True
    else: return False
    
def directionVector(oldData, currentData, length, baseCoords):
    """
    Creates a vector of length 'length' from previous point in space and current point in space.
    This vector is in same coordinate frame as robot.
    """
    # Only current data needs to be converted as current data becomes old data on next cycle
    currentData = transRot(baseCoords, currentData)
    
    vec = currentData - oldData
    vec = (length/la.norm(vec))*vec
    
    predictedPoint = currentData + vec
    
    return currentData, predictedPoint    
    
def isPointInsideSphere(point, sphereCentre, sphereRadius):
    """
    Check if a point is within a sphere
    """
    dist = la.norm(point - sphereCentre)
    if dist <= sphereRadius: return True
    else: return False

# Eva #
#######

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

def isInsideDextrousWorkspace(endEffCoords):
    """
    Returns True if inside robot dextrous workspace, False if not
    """
    centre = np.array([0, 0, 0.187 + 0.096]) # Position of 2nd joint from base
    radius = 0.6 - 0.104 # Max distance between 2nd and 5th joint
    
    if la.norm(endEffCoords - centre) > radius: return False
    else: return True

def calcThetaPhi(eeCoords, fCentre, fRadius):
    """
    Calculates theta and phi
    """
    noTran = eeCoords - fCentre # Remove translation so sphere is centred on (0,0,0) to calculate phi and theta
            
    theta = math.acos(noTran(2)/fRadius) # theta = arccos(z/r), r = radius of sphere
    phi = math.atan2(noTran(1), noTran(0)) # phi = arctan(y/x)
    
    return theta, phi
    

def calcEndEff(focalCoords, focalRadius, endEffCoords=None):
    """
    Three cases when assuming end effector is directly above focal point at a distance 'r': 
        - End effector location is within dextrous sphere of robot.
        - End effector location is outside of dextrous sphere but focal sphere intersects.
        - End effector location is outside of dextrous sphere and focal sphere doesn't intersect.
    Function calculates end effector location and orientation so always pointed at focal point at distance 'r' away from it.
    Function also calculates orientation of end effector if location already known.
    """
    theta = phi = 0 # Intially start as zero
    
    centre = np.array([0, 0, 0.187 + 0.096]) # Position of 2nd joint from base
    radius = 0.6 - 0.104 # Max distance between 2nd and 5th joint
    
    c1 = centre, c2 = focalCoords, r1 = radius, r2 = focalRadius
    # c1 = centre of workspace sphere, c2 = centre of focal sphere, r1 = radius of workspace, r2 = radius of focal space
    
    d = la.norm(c2 - c1)
    
    if endEffCoords == None:
        # Assume directly above 
        endEffCoords = c2 + np.array([0, 0, r1])
        
        # Second case. Test if spheres intersect and test if end eff coords are outside of arm workspace
        if d < (r1 + r2) and la.norm(endEffCoords - c1) > r1:
            rho = np.arange(360.0)
            
            alpha = 0.5 + (r1**2 - r2**2)/2*d**2
            
            ci = c1 + alpha*(c2 - c1) # Centre of intersect circle
            
            ri = math.sqrt(r1**2 - (alpha*d)**2) # Radius of intersect circle
            
            normCC = ci/d # Normalised vector that runs perpendicular to intersect circle
            
            zu = -(normCC[0] - normCC[1])/normCC[2] # z value of tangent vector
            U = np.array([1, 1, zu]) # Tangent vector U
            normU = U/la.norm(U)
            
            V = np.cross(normCC, normU) # Bitangent vector
            
            zMax = -np.inf # Choose max z so angle between focal point and [0 0 1] will be minium 
            for i in range(len(rho)):
                interP = ci + ri*(normU*math.cos(rho[i]) + V*math.sin(rho[i])) # Interect point p(rho) on cicumference of intesect circle
                if interP[2] > zMax: 
                    zMax = interP[2]
                    endEffCoords = interP
            
            theta, phi = calcThetaPhi(endEffCoords, c2, r2)
        
        # Third case. Spheres just touch or do not touch
        elif d >= (r1 + r2):
            
            if d > (r1 + r2):
                r2 = d - r1
                
            endEffCoords = c1 + r1*(c2 - c1)
            
            theta, phi = calcThetaPhi(endEffCoords, c2, r2) 
        
        return phi, theta, endEffCoords, r2
            
# Vision #
##########

def translatePoint(hkl, xyz):
    """
    x' = x - h, y' = y - k, z' = z - l
    xyz are coordinates to be translated into same frame as hkl 	
    Translate points from vision system coordinates to robot base coordinates.
    Robot base is (0,0,0) for all calculations related to the robot. 
    """
    xyzPrime = xyz - hkl
    return xyzPrime
     
def rotatePoint(xyz):
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

    xyzPrime = rotationMat@xyz
    return xyzPrime

def transRot(baseCoords, objectCoords):
    """
    Translate point and then rotates point using translatePoint & rotatePoint function
    returns data as a numpy array, not a matrix
    """
    objectCoords = translatePoint(baseCoords, objectCoords)
    objectCoords = rotatePoint(objectCoords)

    objectCoords = np.squeeze(np.asarray(objectCoords)) # Return it from a matrix to an array
    return objectCoords

def extractCoordinates(data):
    """
    Extracts coordinate data from Motive's datastream and puts them into a list.
    That list is in the correct order e.g. base idx 0, focal idx 1 etc.
    Function also converts to robot frame coordinates.
    Final array should be [robot, focal, object1, ... , objectN]
    """
    data = sorted(data, key=operator.itemgetter(0))
    coordinateData = []
    for i in range(len(data)):
        coordinateData.append(np.asarray(data[i][1])) # Also converts to numpy array
        if i == 0:
            continue
        coordinateData[i] = transRot(coordinateData[0], coordinateData[i])
    return coordinateData

def roundToMillimeter(data):
    """
    Don't need sub-milli accuracy so rounds data to nearest millimeter
    """
    data = np.around(data, decimals=3)
    return data