import numpy as np
from Body import Body
from base_func import get_rot_mat, V

class Joint:
    def __init__(self, body1: Body, body2: Body, loc1: np.ndarray, loc2: np.ndarray):
        self.bodies = [body1, body2]
        self.locs = [loc1, loc2]

class RevJoint(Joint):
    def __init__(self, body1: Body, body2: Body, loc1: np.ndarray, loc2: np.ndarray):
        super().__init__(body1, body2, loc1, loc2)
        
    def get_constraint(self):
        Cs = []
        C = np.empty((2,1), dtype="float")
        for body, loc in zip(self.bodies, self.locs):
            state = body.get_state()
            pos = state[:2]
            theta = state[2,0]  # theta_i
            omega = state[5,0]  # omega_i
            A = get_rot_mat(theta)
            C = np.concatenate((C, (pos+np.dot(A,pos))), axis=1)
            Cq = np.concatenate((np.eye(2,dtype="float"), 
                                np.einsum("ij,jk,kl->il",V,A,loc)), axis=1)
            Cqdqq = np.concatenate((np.zeros((2,2),dtype="float"), 
                                    -np.dot(A,loc)*omega), axis=1)
            Ct = Ctt = np.zeros((2,1), dtype="float")
            Cqt = np.zeros((2,3), dtype="float")
            Cs.append([Cq, Cqdqq, Ct, Ctt, Cqt])
        C = C[:,0] - C[:,1]
        return C, Cs
        
        