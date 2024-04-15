

# System (Default Lib.)
import sys
# Own library for robot control (kinematics), visualization, etc. (See manipulator.py)
import manipulator
# Matplotlib (Visualization Lib. -> Animation) [pip3 install matplotlib]
from matplotlib import animation
# Numpy (Array computing Lib.) [pip3 install numpy]
import numpy as np

def generate_rectangle(centroid, dimension, angle):

    p = [[(-1)*dimension[0]/2, (-1)*dimension[0]/2, (+1)*dimension[0]/2, (+1)*dimension[0]/2, (-1)*dimension[0]/2],
         [(-1)*dimension[1]/2, (+1)*dimension[1]/2, (+1)*dimension[1]/2, (-1)*dimension[1]/2, (-1)*dimension[1]/2]]

    x = []
    y = []

    for i in range(len(p[0])):
        # Calculation position of the Rectangle
        x.append((p[0][i]*np.cos(angle * (np.pi/180)) - p[1][i]*np.sin(angle * (np.pi/180))) + centroid[0])
        y.append((p[0][i]*np.sin(angle * (np.pi/180)) + p[1][i]*np.cos(angle * (np.pi/180))) + centroid[1])

    return [x, y]

def generate_circle(centroid, radius):
    """
    Description:
        A simple function to generate a path for a circle.

    Args:
        (1) centroid [Float Array]: Centroid of the Circle (x, y).
        (1) radius [Float]: Radius of the Circle (r).
        
    Returns:
        (1 - 2) parameter{1}, parameter{2} [Float Array]: Results of path values.

    Examples:
        generate_circle([1.0, 1.0], 0.1)
    """

    # Circle ->  0 to 2*pi
    theta = np.linspace(0, 2*np.pi, 25)

    # Calculation position of the Circle
    x = radius * np.cos(theta) + centroid[0]
    y = radius * np.sin(theta) + centroid[1]

    return [x, y]

def get_user_input():
    initial_coords_x = input("Enter initial coordinates (x): ")
    initial_coords_y = input("Enter initial coordinates (y): ")
    target_coords_x = float(input("Enter target coordinates (x): "))
    target_coords_y = float(input("Enter target coordinates (y): "))
    initial_coords_x = float(initial_coords_x)
    initial_coords_y = float(initial_coords_y)
    #target_coords = float(target_coords)
    
    return initial_coords_x , initial_coords_y,target_coords_x,initial_coords_y
    #initial_coords = list(map(float, initial_coords.split(',')))
    #target_coords = list(map(float, target_coords.split(',')))
    


def main():
    # Initial Parameters -> ABB IRB910SC 
    # Product Manual: https://search.abb.com/library/Download.aspx?DocumentID=3HAC056431-001&LanguageCode=en&DocumentPartId=&Action=Launch

    # Working range (Axis 1, Axis 2)
    initial_coords_x , initial_coords_y,target_coords_x,target_coords_y = get_user_input()
    print("initial_coords_x",initial_coords_x)
    print("target_coords_x",target_coords_x)
    axis_wr = [[-270.0, 270.0],[-270.0, 270.0]]
    # Length of Arms (Link 1, Link2)
    arm_length = [0.4, 0.4]

    # DH (Denavit-Hartenberg) parameters
    theta_0 = [0.0,0.0]
    a       = [arm_length[0], arm_length[1]]
    d       = [0.0, 0.0]
    alpha   = [0.0, 0.0]

    # Initialization of the Class (Control Manipulator)
    # Input:
    #   (1) Robot name         [String]
    #   (2) DH Parameters      [DH_parameters Structure]
    #   (3) Axis working range [Float Array]
    scara = manipulator.Control('Mule Robot Testing', manipulator.DH_parameters(theta_0, a, d, alpha), axis_wr)

    """
    Example (1): 
        Description:
            Chack Target (Point) -> Check that the goal is reachable for the robot

        Cartesian Target:
            x = {'calc_type': 'IK', 'p': [0.20, 0.60], 'cfg': 0}
        Joint Target:
            x = {'calc_type': 'FK', 'theta': [0.0, 155.0], 'degree_repr': True}

        Call Function:
            res = scara.check_target(check_cartesianTarget)

    Example (2): 
        Description:
            Test Results of the kinematics.

        Forward Kinematics:
            x.forward_kinematics(1, [0.0, 0.0], 'rad')
        Inverse Kinematics:
            (a) Default Calculation method
                x.inverse_kinematics([0.35, 0.15], 1)
            (b) Jacobian Calculation method
                x.inverse_kinematics_jacobian([0.35, 0.15], [0.0, 0.0], 0.0001, 10000)
        Both kinematics to each other:
            x.forward_kinematics(0, [0.0, 45.0], 'deg')
            x.inverse_kinematics(x.p, 1)
    """
    #res = scara.check_target_err(check_cartesianTarget)

    # Test Trajectory (Select one of the options - Create a trajectory structure -> See below)
    #test_trajectory = 'Default_1'
    

    # Structure -> Null
    trajectory_str = []
    #trajectory_str.append({'interpolation': 'linear', 'start_p': [0.30,0.2], 'target_p': [0.40, 0.30], 'step': 25, 'cfg': 1})
    trajectory_str.append({'interpolation': 'joint', 'start_p': [initial_coords_x,initial_coords_y], 'target_p': [target_coords_x, target_coords_y], 'step': 50, 'cfg': 1})

    

 
    # Structure -> Null
    check_cartesianTrajectory_str = []

    for i in range(len(trajectory_str)):
        # Generating a trajectory from a structure
        x, y, cfg = scara.generate_trajectory(trajectory_str[i])

        for j in range(trajectory_str[i]['step']):
            # Create a Cartesian trajectory structure for each of the points and configurations
            check_cartesianTrajectory_str.append({'calc_type': 'IK', 'p': [x[j], y[j]], 'cfg': cfg[j]})
            # Adding points and configurations to the resulting trajectory
            scara.trajectory[0].append(x[j])
            scara.trajectory[1].append(y[j])
            scara.trajectory[2].append(cfg[j])
    
    # Check that the trajectory points for the robot are reachable
    tP_err = scara.check_trajectory(check_cartesianTrajectory_str)
    print("tP_err: ",tP_err)

    # Smoothing the trajecotory using Bézier Curve (3 points -> Quadratic, 4 -> Points Cubic)
    tP_smooth = False

    if tP_smooth == True:
        smooth_trajectory = [[], [], []]
        try :
            # Check that trajectory smoothing is possible and smooth the trajectory using an appropriate method
            [smooth_trajectory[0], smooth_trajectory[1], smooth_trajectory[2]] = scara.smooth_trajectory(trajectory_str)
            # Trajectory smoothing is successful
            scara.trajectory = smooth_trajectory
        except TypeError:
            print('[INFO] Trajectory smoothing is not possible (Insufficient or too many entry points).')

    scara.display_environment([True, 1])

    if True in tP_err[0]:
        # Trajectory Error (some points are not not reachable)
        scara.init_animation()
    else:
        # Call the animator for the SCARA Robotics Arm (if the results of the solution are error-free).
        animator = animation.FuncAnimation(scara.figure, scara.start_animation, init_func=scara.init_animation, frames=len(scara.trajectory[0]), interval=2, blit=True, repeat=False)
        # Save Animation 
        animator.save(f'new.gif', fps=30, bitrate=1000)

    #scara.plt.show()

if __name__ == '__main__':
    sys.exit(main())
