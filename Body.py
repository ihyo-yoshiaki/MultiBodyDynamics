import numpy as np

# 親クラス
class Body:
    # 初期設定
    def __init__(self, m: float, pos_g: tuple=(0.,0.), theta0: float=0.):
        """_summary_

        Args:
            m (float): the mass of body [kg]
            pos_g (tuple): the position of center of gravity on absolute coordination [mm] 
            theta0 (float): the angle from the standart posture, counter clockwise is positive [radius]
            
        Description:
            initialization 
        """
        # the common parameter initialized by specified value
        self._m = m
        self._pos_g = pos_g
        self._theta0 = theta0
        
        # the common parameter not determined in init
        self._joints = np.empty((1,2), dtype=np.float64)
        
        # the parameter differed by body types and caluculaterd based on 
        self._J_G = self.get_J_G()
        
        # the optional paramter, setting in each subclass
        
    def set_joint_point(self, dx: float, dy: float):
        """_summary_

        Args:
            dx (float): the distance from center of gravity on x axis [mm]
            dy (float): the distance from center of gravity on y axis [mm]
        """
        self.joints = np.concatenate((self.joints, np.array([[dx, dy]], dtype=np.float64)), axis=0)
    
    # calculate inertia
    def get_J_G(self):
        pass
    
    # return Cq, Cqdqq, and so on
    def get_eq_constraint(self, dtheta, joint_id, joint_type):
        pass
    
    # draw object on figure
    def draw_fig(self,ax):
        pass
    
# 子クラス-棒
class Bar(Body):
    pass
 
# 子クラス-楕円 (円)       
class Ellipse(Body):
    pass

# 子クラス-長方形（正方形）      
class Rectangle(Body):
    pass

# 子クラス-質点      
class Mass(Body):
    pass

"""
ボディi-j間の拘束式
  > 回転ジョイントor固定ジョイント，i-j間の角度が分かれば求まる
ボディi-j間に働くばねダンパの外力
  > ばね係数，減衰係数，i-j間の角度が分かれば求まる
ボディiに働く外力
  > 重力

> 最終的な加速度方程式に用いるCq，Cqdqqは各ボディiのCq_i, Cqdqq_iから構築できる
  > 各ボディでCq_i, Cqdqq_iを計算，body_joint_systemで全体の拘束式を構築すればよい
"""