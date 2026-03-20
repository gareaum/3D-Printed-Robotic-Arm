import numpy as np
from scipy.spatial.transform import Rotation as R
from scipy.optimize import fsolve

def FK_dh(dh_params, joint_angles, link): #Calculates the forward kinematics using DH convention
    """
        @param  dh_params:     The dh parameters as a 2D list each row represents a link and has the format [a, alpha, d, theta]
        @param  joint_angles:  The joint angles of the links
        @param  link:          The link to transform from
        @param  returns:       A transformation matrix representing the pose of the desired link
    """
    T = np.eye(4)
    transforms = []

    if link > len(dh_params):
        raise ValueError("Link index exceeds Dh parameter list")
    
    for i in range(link):
        a , alpha, d, theta = dh_params[i]
        theta += joint_angles[i]
        T = T @ get_transform_from_dh(a,alpha,d,theta)
        transforms.append(T.copy())

    return T,transforms

def get_transform_from_dh(a, alpha, d, theta): #Gets the transformation matrix T from dh parameters
    """!
    Args:
        a:  a milimeters
    alpha:  alpha radians
        d:  d milimeters
    theta:  theta radians

    return:  
            The 4x4 transformation matrix.
    """
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)

    return np.array([
        [ct, -st * ca,  st * sa, a * ct],
        [st,  ct * ca, -ct * sa, a * st],
        [0,   sa,       ca,      d     ],
        [0,   0,        0,       1     ]
    ], dtype=np.float64)

def get_euler_angles_from_T(T): #Gets the euler angles from a transformation matrix.

    """
    Arg:
        T:  Transformation matrix


    return:    
            The euler angles from T.
    """
    return R.from_matrix(T[:3,:3]).as_euler('xyz', degrees = True)

def get_pose_from_T(T): #Gets the pose from T.
    """
    Arg:
        T:     Transformation matrix

    return:
               The pose vector from T.
    """
    euler = get_euler_angles_from_T(T)

    x, y, z = T[:3,3]
    phi, theta, psi = euler

    return np.array([x, y, z, phi, theta, psi])

###TODO###
def IK_geometric(dh_params, pose): #Get all possible joint configs that produce the pose.
    """!
    Args:

        dh_params:  The dh parameters
             pose:  The desired pose vector as np.array 

    return:     
            All four possible joint configurations in a numpy array 4x4 where each row is one possible joint
            configuration
    """

    pass
