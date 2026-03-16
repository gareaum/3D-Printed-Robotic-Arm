import numpy as np
from scipy.linalg import expm
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

    for i in range(link):
        a , alpha, d, theta = dh_params[i]
        theta += joint_angles[i]
        T = T @ get_transform_from_dh(a,alpha,d,theta)

    return T

def get_transform_from_dh(a, alpha, d, theta): #Gets the transformation matrix T from dh parameters
    """!
    @param      a      a milimeters
    @param      alpha  alpha radians
    @param      d      d milimeters
    @param      theta  theta radians

    @return     The 4x4 transformation matrix.
    """
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)

    return np.matrix([
        [ct, -st * ca,  st * sa, a * ct],
        [st,  ct * ca, -ct * sa, a * st],
        [0,   sa,       ca,      d     ],
        [0,   0,        0,       1     ]
    ], dtype=np.float64)

def get_euler_angles_from_T(T): #Gets the euler angles from a transformation matrix.

    """
    @param      T     transformation matrix

    @return     The euler angles from T.
    """
    return R.from_matrix(T[:2,:2]).as_euler('zyx', degrees = True)

def get_pose_from_T(T): #Gets the pose from T.
    """
    @param      T     transformation matrix

    @return     The pose vector from T.
    """
    euler = get_euler_angles_from_T(T)

    x, y, z = T[:3,3]
    roll, pitch, yaw = euler

    return [x, y, z, roll, pitch, yaw]

def IK_geometric(dh_params, pose):
    """!
    @brief      Get all possible joint configs that produce the pose.

                TODO: Convert a desired end-effector pose vector as np.array to joint angles

    @param      dh_params  The dh parameters
    @param      pose       The desired pose vector as np.array 

    @return     All four possible joint configurations in a numpy array 4x4 where each row is one possible joint
                configuration
    """
    pass
