import pandas as pd
import numpy as np
import Kinematics as kin
import Jacobian as J

class RobotArm:
    def __init__(self, dh_table, joint_types):
        
        df = pd.read_csv(dh_table)
        jt = pd.read_csv(joint_types)

        self.dh = df[['a','alpha','d','theta']].to_numpy(dtype=np.float64)
        self.jt = jt[['joint_amt','joint_types']].to_numpy(dtype=np.float64)
        self.joint_amt = len(self.jt)

    def Kinematics(self, joint_angles):
        # Compute FK and all transforms
        self.T, self.transforms = kin.FK_dh(self.dh, joint_angles, links=len(self.dh))
        
        # Euler angles
        self.R = kin.get_euler_angles_from_T(self.T)
        
        # Pose
        self.x, self.y, self.z, self.phi, self.theta, self.psi = kin.get_pose_from_T(self.T)

    def Jacobian(self):

        # Compute geometric Jacobian
        self.J_geom = J.geometric_jacobian(self.transforms, self.jt[:,1], self.joint_amt)
        
        # Compute analytical Jacobian
        self.J_analy = J.analytical_jacobian(self.J_geom, self.R)