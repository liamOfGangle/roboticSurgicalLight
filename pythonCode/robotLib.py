import csv
import math

def angleLimitsCheck(jointAngles, inputUnit, jointHardLimits):
    # Keep the angle of the joints 3 degrees (or 3pi/180 radians) away from the joint maximums and minimums.

    offset = 3.0

    # Test for correct number of joint angles 
    if len(jointAngles) != len(jointHardLimits):
        print('ERROR: Not enough joint angles provided')
    
    if inputUnit == 'deg':
        for i in range(len(jointAngles)):
            if jointAngles[i] <= jointHardLimits[i][1]:
                jointAngles[i] = jointHardLimits[i][1] + offset
            elif jointAngles[i] >= jointHardLimits[i][2]:
                jointAngles[i] = jointHardLimits[i][2] - offset

    elif inputUnit == 'rad':
        offset = math.radians(offset)

        for i in range(len(jointAngles)):
            for j in range(len(jointHardLimits[0])):
                # Convert jointHardLimits to radians
                if j < 1:
                    pass
                else:
                    jointHardLimits[i][j] = math.radians(jointHardLimits[i][j])
            
            if jointAngles[i] <= jointHardLimits[i][1]:
                jointAngles[i] = jointHardLimits[i][1] + offset
            elif jointAngles[i] >= jointHardLimits[i][2]:
                jointAngles[i] = jointHardLimits[i][2] - offset
    
    else:
        print('inputUnit must be deg or rad')

    return jointAngles

def loadLimitsFile(path):
    # Load in csv of joint limits. Joint limits found in documentation
    jointFile = [line.split(',') for line in open(path)]

    for i in range(len(jointFile)):
        for j in range(len(jointFile[i])):
            jointFile[i][j] = float(jointFile[i][j])
    
    return jointFile

def convert(self, x, ROUND=0):

    conv = []

    for _ in range(x.size[0]):
        lst = x._data[_::x.size[0]].tolist()

        if ROUND is not 0:
            lst = [self.round_sig(elem, ROUND) if elem != 0 and
                   elem == elem else elem for elem in lst]

        conv.append(lst)

    return conv

def round_sig(self, x, sig=2):
    return round(x, sig-int(math.floor(math.log10(abs(x))))-1)