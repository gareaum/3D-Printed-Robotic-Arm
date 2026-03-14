import numpy as np
# expm is a matrix exponential function
from scipy.linalg import expm
from scipy.spatial.transform import Rotation as Rot
from scipy.optimize import fsolve

def clamp(angle):
    """!
    @brief      Clamp angles between (-pi, pi]

    @param      angle  The angle

    @return     Clamped angle
    """
    while angle > np.pi:
        angle -= 2 * np.pi
    while angle <= -np.pi:
        angle += 2 * np.pi
    return angle


def FK_dh(dh_params, joint_angles, link):
    """!
    @brief      Get the 4x4 transformation matrix from link to world

                TODO: implement this function

                Calculate forward kinematics for rexarm using DH convention

                return a transformation matrix representing the pose of the desired link

                note: phi is the euler angle about the y-axis in the base frame

    #@param      dh_params     The dh parameters as a 2D list each row represents a link and has the format [a, alpha, d,
                              theta]
                              
    @param      joint_angles  The joint angles of the links
    @param      link          The link to transform from

    @return     a transformation matrix representing the pose of the desired link
    """
    T = np.eye(4)

    for i in range(link):
        a, alpha, d, theta0 = dh_params[i]
        theta = theta0 + joint_angles[i]

        A = get_transform_from_dh(a, alpha, d, theta)
       
        T = T @ A

    return T


def get_transform_from_dh(a, alpha, d, theta):
    """!
    @brief      Gets the transformation matrix T from dh parameters.

    TODO: Find the T matrix from a row of a DH table

    @param      a      a meters
    @param      alpha  alpha radians
    @param      d      d meters
    @param      theta  theta radians

    @return     The 4x4 transformation matrix.
    """
    return  np.array([
            [np.cos(theta), -np.sin(theta)*np.cos(alpha),  np.sin(theta)*np.sin(alpha), a*np.cos(theta)],
            [np.sin(theta),  np.cos(theta)*np.cos(alpha), -np.cos(theta)*np.sin(alpha), a*np.sin(theta)],
            [0,              np.sin(alpha),                np.cos(alpha),               d],
            [0            ,  0,                            0,                           1]
        ])
    

def get_euler_angles_from_T(T):
    """!
    @brief      Gets the euler angles from a transformation matrix.

                TODO: Implement this function return the 3 Euler angles from a 4x4 transformation matrix T
                If you like, add an argument to specify the Euler angles used (xyx, zyz, etc.)

    @param      T     transformation matrix

    @return     The euler angles from T.
    """
     
    #Get rotation from T
    R = Rot.from_matrix(T[0:2, 0:2])
    
    #Euler angle definition
    phi,theta,psi = R.as_euler('zyz', degrees=False)
    
    phi = clamp(phi)
    theta = clamp(theta)
    psi = clamp(psi)

    #Hangle singularities
    return phi, theta, psi

def get_pose_from_T(T):
    """!
    @brief      Gets the pose from T.

                TODO: implement this function return the 6DOF pose vector from a 4x4 transformation matrix T

    @param      T     transformation matrix

    @return     The pose vector from T.
    """

    phi, theta, psi = get_euler_angles_from_T(T)

    x = T[0,3]
    y = T[1,3]
    z = T[2,3]

    return [x, y, z, phi, theta, psi]
