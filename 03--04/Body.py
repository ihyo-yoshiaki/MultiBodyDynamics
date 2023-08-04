import numpy as np
import matplotlib.pyplot as plt
from base_func import get_rot_mat

class Body:
    def __init__(self, m: float, J_G:float, 
                 x:float, y:float, theta: float, 
                 vx: float=0., vy: float=0., omega: float=0.):
        self._m = m
        self._J_G = J_G
        self._state = np.array([[x], [y], [theta], [vx], [vy], [omega]], dtype="float")
    
    def get_m(self):
        return self._m
    
    def get_J_G(self):
        return self._J_G
    
    def get_state(self):
        return self._state
    
class Rectangle(Body):
    def __init__(self, m: float, x: float, y: float, theta: float, a: float, b: float):
        J_G = a * b / 12 
        super().__init__(m, J_G, x, y, theta)
        self._a = max(a, b)  # long side 
        self._b = min(a, b)  # short side
        
    def draw_ax(self, ax: plt.Axes, color: str="skyblue"):
        A = get_rot_mat(self._state[2,0])
        vec1 = np.array([[self._a/2], [self._b/2]], dtype="float")
        vec2 = np.array([[self._a/2], [-self._b/2]], dtype="float")
        pos1 = self._state[:2] + np.dot(A, vec1)  # upper right
        pos2 = self._state[:2] + np.dot(A, vec2)  # lower right
        pos3 = self._state[:2] + np.dot(A, -vec1)  # lower left
        pos4 = self._state[:2] + np.dot(A, -vec2)  # upper left
        poss = np.concatenate([pos1, pos2, pos3, pos4], axis=1).T
        # draw sides
        for i in range(4):
            j = (i + 1) % 4
            ax.plot([poss[i,0], poss[j,0]], [poss[i,1], poss[j,1]], color=color, lw=2)
        
        # paint inside of rectangle
        poss_sort = poss[np.argsort(poss[:,0])]  # sort by x (asc)
        # calculate potion of edge
        poss_sort = np.concatenate((poss_sort, np.zeros((4,1))), axis=1)
        poss_sort[0,2] = poss_sort[0,1]
        poss_sort[3,2] = poss_sort[3,1]
        poss_sort[1,2] = (poss_sort[2,1] - poss_sort[0,1]) / (poss_sort[2,0] - poss_sort[0,0]) * (poss_sort[1,0] - poss_sort[0,0]) + poss_sort[0,1]
        poss_sort[2,2] = (poss_sort[3,1] - poss_sort[1,1]) / (poss_sort[3,0] - poss_sort[1,0]) * (poss_sort[2,0] - poss_sort[1,0]) + poss_sort[1,1]
        poss_sort[1,1:] = np.sort(poss_sort[1,1:])
        poss_sort[2,1:] = np.sort(poss_sort[2,1:])
        # paint 
        ax.fill_between(poss_sort[:,0], poss_sort[:,1], poss_sort[:,2],
                        color=color, alpha=0.5)
    
class Bar(Body):
    def __init__(self, m: float, x: float, y: float, theta: float, l: float):
        J_G = np.square(l) / 3
        super().__init__(m, J_G, x, y, theta)
        self._l = l  # half length of bar
        
    def draw_ax(self, ax: plt.Axes, color: str="skyblue"):
        A = get_rot_mat(self._state[2,0])
        l_vec = np.array([[self._l], [0]], dtype="float")
        pos1 = self._state[:2] + np.dot(A, l_vec)
        pos2 = self._state[:2] + np.dot(A, -l_vec)
        ax.plot([pos1[0], pos2[0]], [pos1[1], pos2[1]], color=color, lw=4)