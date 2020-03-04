import matlab.engine
import robotLib as rl
import math
import numpy as np

# Start the MATLAB engine
eng = matlab.engine.start_matlab()

## Setup ##
# Load in joint limits 
limitsPath = 'C:\\Users\\ljtov\\Documents\\roboticSurgicalLight\\jointInfo\\jointLimitsEVA.csv'
jointLimits = rl.loadLimitsFile(limitsPath)

# Create .mat file
eng.matEVASetup(nargout=0)

# Create home angles
homeAngles = [0, 70, -160, 0, -90, 0]
homeAngles = rl.angleLimitsCheck(homeAngles, 'deg', jointLimits)

delta = 0.2

### Assume that robot always starts from home pos ###

try:
    while True:
        print('Input x coordinate')
        x = input()
        x = float(x)
        
        print('Input y coordinate')
        y = input()
        y = float(y)

        print('Input z coordinate')
        z = input()
        z = float(z)

        focalPos = [x, y, z]

        ### Assume for initial test that end effector always directly above focal point at distance delta ###
        focalOri = [0.0, 1.0, 0.0, math.pi]

        focalPos = matlab.double(focalPos)
        focalOri = matlab.double(focalOri)
        newJointAngles = eng.ikSolution(focalPos,focalOri,delta,nargout=1)

        print(type(newJointAngles))
        print(newJointAngles)

        newJointAngles = np.array(newJointAngles._data).tolist()
        print(type(newJointAngles)) 
        print(newJointAngles) 

except KeyboardInterrupt:
    pass
