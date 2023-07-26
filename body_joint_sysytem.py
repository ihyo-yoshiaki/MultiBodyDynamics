import numpy as np
import scipy.integrate
from Body import Bar, Ellipse

class Body_joint_system:
    def __init__(self, bodies: list=[]):
        self._bodies = bodies 
        self._n_bodies = len(list)
        self._spring = np.zeros((self.nb,self.nb),dtype="float")
        self._damper = np.zeros((self.nb,self.nb),dtype="float")
        # the diagonal element of these matrix is tentatively zero.
        # not-diagonal element (i,j) experess the coeffiecient of spring/damper between body i and body j
        # element (i,j) where i<j express that spring/damper is attached 
        # counter-clockwise from body j to body i
    
    def add_bodies(self, bodies: list=[]):
        new_n_bodies = len(bodies)
        self._bodies.appned(bodies)
        self._n_bodies = len(self._bodies)
        
        # expand coefficient matrix of spring and damper
        new_coe1 = np.zeros((self._spring.shape[0], new_n_bodies), dtype="float")
        new_coe2 = np.zeros((self._spring.shape[0]+new_n_bodies, new_n_bodies), dtype="float")
        self.spring = np.concatenate((self._spring, new_coe1), axis=1)
        self.spring = np.concatenate((self._spring, new_coe2), axis=0)
        self.damper = np.concatenate((self._damper, new_coe1), axis=1)
        self.damper = np.concatenate((self._damper, new_coe2), axis=0)
        
    def set_spring(self, ib: int, jb: int, coe: float, clockwise: bool=False):
        if clockwise:
            self._spring[jb,ib] = coe
        else:
            self._spring[ib,jb] = coe
            
    def set_damper(self, ib: int, jb: int, coe: float, clockwise: bool=False):
        if clockwise:
            self._damper[jb,ib] = coe
        else:
            self._damper[ib,jb] = coe
        
    def get_rot_mat(self, theta: np.float64):
        rot_mat = np.empty((2, 2), dtype="float")
        rot_mat[0,0] = np.cos(theta)
        rot_mat[1,1] = np.cos(theta)
        rot_mat[0,1] = -np.sin(theta)
        rot_mat[1,0] = np.sin(theta)
        return rot_mat
        
    def func_rev_b2b(self, ib: int, jb: int, q: np.ndarray, v: np.ndarray,
                     n_loci: np.float64, n_locj: np.float64):
        # q_i = q[3*ib:3*(ib+1)+1]
        # q_j = q[3*jb:3*(jb+1)+1]
        # n_loci = r^{\prime}_{GiAi}
        # n_locj = r^{\prime}_{GjAj}
        
        # 対象とするボディの一般化座標
        # r_i = [x_i, y_i]
        # r_j = [x_j, y_j]
        r_i  = q[3*ib:3*(ib+1)]
        r_j = q[3*jb:3*(jb+1)]
        theta_i = q[3*(ib+1)]
        theta_j = q[3*(jb+1)]
        dtheta_i = v[3*(ib+1)]
        dtheta_j = v[3*(jb+1)]
        
        # 回転行列
        A_i = self.get_rot_mat(theta=theta_i)
        A_j = self.get_rot_mat(theta=theta_j)
        
        # 拘束式
        C_rev = r_j + np.dot(n_locj, A_j.T) - (r_i + np.dot(n_loci, A_i.T))
        
        # ヤコビマトリックス
        C_rev_qi = np.concatenate((np.zeros((2, 3*(ib-1)), dtype="float"),
                                  np.eye(2, dtype="float"),
                                  np.zeros((2, 3*(self.nb-ib)), dtype="float")
                                  ), axis=1)
        C_rev_qj = np.concatenate((np.zeros((2, 3*(jb-1)), dtype="float"),
                                  np.eye(2, dtype="float"),
                                  np.zeros((2, 3*(self.nb-jb)), dtype="float")
                                  ), axis=1)
        C_rev_q = C_rev_qi + C_rev_qj
        
        C_rev_q_dq_qi = np.concatenate((np.zeros((2, 3*(ib-1)+2), dtype="float"),
                                        (np.dot(n_loci, A_i) * dtheta_i), 
                                        np.zeros((2, 3*(self.nb-ib)), dtype="float")
                                        ), axis=1)
        C_rev_q_dq_qj = np.concatenate((np.zeros((2, 3*(jb-1)+2), dtype="float"),
                                        (np.dot(n_locj, A_j) * dtheta_j), 
                                        np.zeros((2, 3*(self.nb-jb)), dtype="float")
                                        ), axis=1)
        C_rev_q_dq_q = C_rev_q_dq_qi + C_rev_q_dq_qj
        
        C_rev_t = C_rev_tt = np.zeros((2, 1), dtype="float")
        C_rev_qt = np.zeros((2, 3*self.nb), dtype="float")
        
        return C_rev, C_rev_q, C_rev_q_dq_q, C_rev_t, C_rev_tt, C_rev_qt
    
    def get_Q(self, ib: int, jb: int, delta_theta: np.float64, delta_dtheta: np.float64):
        """calculate force by spring and damper between body ib and body jb

        Args:
            ib (int): id of one of bodies
            jb (int): id of the other body
            delta_theta (np.float64): the angle between body ib and body jb
            delta_dtheta (np.float64): the angle velocity between body ib and body jb

        Returns:
            np.ndarray: force by spring and damper, 2D array (3, 3*nb)
        """
        Q = np.zeros((self._n_bodies*3) ,dtype="float")
        n_rsi = -(self._spring[ib,jb] * delta_theta + self._damper[ib,jb] * delta_dtheta)
        n_rsi += -(self._spring[jb,ib] * delta_theta + self._damper[jb,ib] * delta_dtheta)
        Q[3*(ib-1)+2] = n_rsi
        Q[3*(jb-1)+2] = -n_rsi
        return Q
    
    # constraint force
    def get_Qc(self, Cq, Q, gamma):
        tmp = np.linalg.inv(np.einsum("i,ij,j->ij", Cq, self.M_inv, Cq))
        tmp2 = np.einsum("i,ij,j->ij", Cq, self.M_inv, Q) - gamma
        Qc = np.einsum("i,ij,jk->k",Cq, tmp, tmp2)
        return Qc
    
    def dif_eq(self,q,v):
        # 加速度方程式
        C_rev, C_rev_q, C_rev_q_dq_q,\
        C_rev_t, C_rev_tt, C_rev_qt \
            = self.func_rev_b2b(ib=0, jb=0, q=q, v=v, n_loci=self.n_loci, n_locj=self.n_locj)
        gamma = -(np.dot(v, C_rev_q_dq_q + 2 * C_rev_qt) + C_rev_tt) 
        gamma_beta = gamma - 2 * self.alpha * np.dot(v, C_rev_q) - np.square(self.beta) * C_rev
        
        # 外力ベクトル
        Q = self.get_Q(ib=self.ib, jb=self.jb, delta_theta=0)
         
        # 拘束力ベクトル
        Qc = self.get_Qc(Cq=C_rev_q, Q=Q, gamma=gamma_beta) 
        
        # 1次微分方程式
        qdot = v
        vdot = np.dot(self.M_inv, (Q + Qc))
        return np.concatenate((qdot, vdot),axis=0)
    
    def forward(self):
        # forwardの度にパラメータ設定
        self.M = np.diag([[b._m,b._m,b._J_G] for b in zip(self.bodies)])
        
        self.M_inv = np.linalg.inv(self.M)
        
        pass
    
    
    # self.get_Qの調整中
    # self.Mの作り方
        
        