import numpy as np

class KinematicSystem:
    def __init__(self, axis_wr, arm_length, theta_0, a, d, alpha):
        self.axis_wr = axis_wr
        self.arm_length = arm_length
        self.rDH_param = DHParameters(theta_0, a, d, alpha)
        self.jacobian_matrix = np.zeros((2, 2))
        self.theta = np.zeros(2)
        self.__p_target = np.zeros(2)
        self.p = np.zeros(2)
        self.__Tn_theta = np.identity(4)
        self.__theta_target = np.zeros(2)
        self.__rounding_index = 5

    def inverse_kinematics(self, p, cfg):
        self.__p_target = np.array(p)

        theta_aux = np.zeros(2)

        # Cosine Theorem [Beta]
        cosT_beta_numerator = ((self.rDH_param.a[0]**2) + (self.__p_target[0]**2 + self.__p_target[1]**2) - (self.rDH_param.a[1]**2))
        cosT_beta_denumerator = (2*self.rDH_param.a[0]*np.sqrt(self.__p_target[0]**2 + self.__p_target[1]**2))

        # Calculation angle of Theta 1
        if cosT_beta_numerator/cosT_beta_denumerator > 1:
            theta_aux[0] = np.arctan2(self.__p_target[1], self.__p_target[0])
        elif cosT_beta_numerator/cosT_beta_denumerator < -1:
            theta_aux[0] = np.arctan2(self.__p_target[1], self.__p_target[0]) - np.pi
        else:
            if cfg == 0:
                theta_aux[0] = np.arctan2(self.__p_target[1], self.__p_target[0]) - np.arccos(cosT_beta_numerator/cosT_beta_denumerator)
            elif cfg == 1:
                theta_aux[0] = np.arctan2(self.__p_target[1], self.__p_target[0]) + np.arccos(cosT_beta_numerator/cosT_beta_denumerator)

        # Cosine Theorem [Alpha]
        cosT_alpha_numerator = (self.rDH_param.a[0]**2) + (self.rDH_param.a[1]**2) - (self.__p_target[0]**2 + self.__p_target[1]**2)
        cosT_alpha_denumerator = (2*(self.rDH_param.a[0]*self.rDH_param.a[1]))

        # Calculation angle of Theta 2
        if cosT_alpha_numerator/cosT_alpha_denumerator > 1:
            theta_aux[1] = np.pi
        elif cosT_alpha_numerator/cosT_alpha_denumerator < -1:
            theta_aux[1] = 0.0
        else:
            if cfg == 0:
                theta_aux[1] = np.pi - np.arccos(cosT_alpha_numerator/cosT_alpha_denumerator)
            elif cfg == 1:
                theta_aux[1] = np.arccos(cosT_alpha_numerator/cosT_alpha_denumerator) - np.pi

        self.theta = theta_aux

        # Calculate the forward kinematics from the results of the inverse kinematics.
        self.forward_kinematics(0,self.theta, 'rad')

    def forward_kinematics(self, calc_type, theta, angle_repr):
        self.__theta_target = np.zeros(2)
        self.__theta_target[0] = theta[0]
        self.__theta_target[1] = theta[1]

        if angle_repr == 'deg':
            self.rDH_param.theta = [x * (np.pi/180) for x in self.__theta_target]
        elif angle_repr == 'rad':
            self.rDH_param.theta = self.__theta_target

        if calc_type == 0:
            for i in range(len(self.rDH_param.theta)):
                self.__Tn_theta =  self.__Tn_theta @ self.__dh_calc_fk(i)

            self.__separete_translation_part()
        elif calc_type == 1:
            self.__fast_calc_fk()

        self.__Tn_theta = np.array(np.identity(4))
        
    def __separete_translation_part(self):
        """
        Description: 
            Separation translation part from the resulting transformation matrix.
        """

        for i in range(len(self.p)):
            self.p[i] = round(self.__Tn_theta[i, 3], self.__rounding_index)

        
    def __dh_calc_fk(self, index):
        """
        Description:
            Calculate the transformation matrix for each joint using the Denavit-Hartenberg parameters.
        """

        # Initialize the transformation matrix
        T = np.identity(4)

        # Set the rotation matrix

        
        R = np.array([
            [np.cos(self.rDH_param.theta[index]), -np.sin(self.rDH_param.theta[index]) * np.cos(self.rDH_param.alpha[index]), np.sin(self.rDH_param.theta[index]) * np.sin(self.rDH_param.alpha[index])],
            [np.sin(self.rDH_param.theta[index]), np.cos(self.rDH_param.theta[index]) * np.cos(self.rDH_param.alpha[index]), -np.cos(self.rDH_param.theta[index]) * np.sin(self.rDH_param.alpha[index])],
            [0, np.sin(self.rDH_param.alpha[index]), np.cos(self.rDH_param.alpha[index])]
            ])
        #print("R", R)

        # Set the transformation matrix
        T[:3, :3] = R
        #print("T", T)
        T[:3, 3] = np.array([self.rDH_param.a[index] * np.cos(self.rDH_param.theta[index]), self.rDH_param.a[index] * np.sin(self.rDH_param.theta[index]), self.rDH_param.d[index]])
        

        return T
    
    def check_trajectory(self, kinematics_str):
        """
        Description:
            Function for checking all reachable trajectory points.

        Args:
            (1) kinematics_str [Structure Type Array]: Structure of the trajectory points.
        
        Returns:
            (1) parameter{1}, parameter{2} [Bool Array]: 1 - Reachable points error (True -> NOK, False -> OK), 2 - Index
        """

        err_p = [[], []]

        for i in range(len(kinematics_str)):
            aux_item = self.__get_item_str(kinematics_str[i])
            aux_kinematics_str = {aux_item[0]: kinematics_str[i][aux_item[0]], aux_item[1]: kinematics_str[i][aux_item[1]], aux_item[2]: kinematics_str[i][aux_item[2]]}

            err_p[0].append(self.check_target_err(aux_kinematics_str))
            err_p[1].append(i)

        return err_p
    
    def check_target_err(self, kinematics_str):
        """
        Description:
            Function to check whether the point is reachable or not. The function allows to check the input structure for Cartesian and Joint parameters.

    
        """

        if kinematics_str['calc_type'] == 'IK': 
            self.inverse_kinematics([kinematics_str['p'][0], kinematics_str['p'][1]], kinematics_str['cfg'])
        
            if (self.theta[0] < self.ax_wr[0] or self.theta[0] > self.ax_wr[1]) or (self.theta[1] < self.ax_wr[2] or self.theta[1] > self.ax_wr[3]):
                print('[INFO] Calculation Error [The target position (Joint representation) is outside of the working range].')
                print('[INFO] Calculation Error [Working range: Theta_1 (%f, %f), Theta_2(%f, %f)]:' % (self.ax_wr[0] * (180/np.pi), self.ax_wr[1]*(180/np.pi), self.ax_wr[2]*(180/np.pi), self.ax_wr[3]*(180/np.pi)))
                print('[INFO] Calculation Error [Target Position: Joint]:', self.theta[0]*(180/np.pi), self.theta[1]*(180/np.pi))

                # Mark error points with -> [*].
                self.plt.plot(kinematics_str['p'][0], kinematics_str['p'][1], marker = '*', ms = 10, mfc = [1,0,0], markeredgecolor = [0,0,0], mew = 2.5)

                return True
            else:
                if (round(self.__p_target[0], 3) != round(self.p[0], 3)) and (round(self.__p_target[1], 3) != round(self.p[1], 3)):
                    print('[INFO] Calculation Error [The target position (Cartesian representation) is outside of the workspace].')
                    print('[INFO] Calculation Error [Actual Position: End-Effector]:', round(self.p[0], 3), round(self.p[1], 3))
                    print('[INFO] Calculation Error [Target Position: End-Effector]:', round(self.__p_target[0], 3), round(self.__p_target[1], 3))

                    # Mark error points with -> [*].
                    self.plt.plot(kinematics_str['p'][0], kinematics_str['p'][1], marker = '*', ms = 10, mfc = [1,0,0], markeredgecolor = [0,0,0], mew = 2.5)

                    return True
                else:
                    return False

            self.__p_target = [kinematics_str['p'][0], kinematics_str['p'][1]]

        elif kinematics_str['calc_type'] == 'FK':
            theta_aux = np.zeros(2)

            if kinematics_str['degree_repr'] == True:
                theta_aux = [x * (np.pi/180) for x in kinematics_str['theta']]
            else:
                theta_aux = kinematics_str['theta']

            if (theta_aux[0] < self.ax_wr[0] or theta_aux[0] > self.ax_wr[1]) or (theta_aux[1] < self.ax_wr[2] or theta_aux[1] > self.ax_wr[3]):
                print('[INFO] Calculation Error [The target position (Joint representation) is outside of the working range].')
                print('[INFO] Calculation Error [Working range: Theta_1 (%f, %f), Theta_2(%f, %f)]:' % (self.ax_wr[0] * (180/np.pi), self.ax_wr[1]*(180/np.pi), self.ax_wr[2]*(180/np.pi), self.ax_wr[3]*(180/np.pi)))
                print('[INFO] Calculation Error [Target Position: Joint]:', theta_aux[0]*(180/np.pi), theta_aux[1]*(180/np.pi))

                return True
            else:
                return False

            self.__p_target = np.zeros(2)

        self.forward_kinematics(1, [0.0, 0.0], 'rad')

# Define the DH parameters for the 2-DOF arm

class DHParameters:
    def __init__(self, theta_0, a, d, alpha):
        self.theta = theta_0
        self.a = a
        self.d = d
        self.alpha = alpha

axis_wr = [[-270.0, 270.0], [-270.0, 270.0]]
arm_length = [0.4, 0.4]
theta_0 = [0.0, 0.0]
a = arm_length
d = [0.0, 0.0]
alpha = [0.0, 0.0]

# Example usage
kinematic_system = KinematicSystem(axis_wr, arm_length, theta_0, a, d, alpha)
p = [0.45, 0.10]  # Target position (x, y)
cfg = 0  # Robot configuration
kinematic_system.inverse_kinematics(p, cfg)
print("Joint angles (Theta_1, Theta_2):", kinematic_system.theta)