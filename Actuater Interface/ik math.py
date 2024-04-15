#####################################
## Inverse Kinematics of 2DOF Link ##
#####################################

import numpy as np

a1= 0.5
a2=0.4
'''
TASK : To find Theta 1 & Theta 2
'''

# First Possible Movement of Links #
'''
STEP (1) : Find Angle  ~ Alpha

'''
alpha  = np.cos(((a1**2) + (a2**2) - (x**2) - (y**2)) / (2*a1*a2))  ## a1 and a2 are the length of links 1 and 2 (0.8m & 1.08m)
'''
STEP (2) : Find Angle ~ Theta2
'''
theta2 = pi - alpha
'''
STEP (3) : Find Angle ~ Beta
'''
length = a1 + (a2 * cos(theta2))
height = a2 * sin(theta2)
beta   = atan2(height, length)
'''
STEP (4) : Find Angle ~ Gamma
'''
gamma  = atan2(y, x)
'''
STEP (5) : Find Angle ~ Theta1
'''
theta1 = gamma - beta


# Second Possible Movement of Links #
# theta2 = alpha - pi
# theta1 = gamma + beta