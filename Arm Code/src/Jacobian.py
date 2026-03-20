import numpy as np
from src.Kinematics import FK_dh, get_euler_angles_from_T

def geometric_jacobian(transforms,joint_types,joint_amts): #Computes the geometric Jacobian
 
    J = np.zeros((6,joint_amts))

    o_n = transforms[-1][:3,3]

    for i in range(joint_amts):
        #joint i position and z-axis
        o_i = transforms[i][:3,3]
        z_i = transforms[i][:3,2]

        if joint_types[i] == 'revolute':
            #linear velocity
            J[3:, i] = z_i
            #angular velocity
            J[:3, i] = np.cross(z_i,(o_n-o_i))
        elif joint_types[i] == 'prismatic':
            #linear velocity
            J[3:, i] = z_i
            #angular velocity(0)
            J[:3, i] = 0
        else:
            pass

    return J

def analytical_jacobian(euler_angles,J_geom): #Computes the Analytical Jacobian
    """
    Args:
        euler_angles: Contains euler angles
              J_geom: Geometric Jacobian 
    
    Returns: 
             J_analy: Analytical Jacobian
    """
    phi,theta,psi = euler_angles
    
    #Transformation from angular velocity to Euler angle rates
    cr = np.cos(phi)
    sr = np.sin(phi)
    cp = np.cos(theta)
    sp = np.sin(theta)
    
    T = np.array([
        [cp, 0, 1],
        [sp*sr, cr, 0],
        [sp*cr, -sr, 0]
    ])
    
    Tf = np.eye(6)
    Tf[3:,3:] = np.linalg.inv(T)
    
    J_analy = Tf @ J_geom

    return J_analy

