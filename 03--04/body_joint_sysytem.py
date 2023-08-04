import numpy as np
import matplotlib.pyplot as plt
from Body import Body
from Joint import Joint

class MultiBodyDynamics2D:
    def __init__(self, body1: Body, body2: Body):
        self.Bodies = [body1, body2]
        self.Springs = [[None for _ in range(2)] for _ in range(2)]
        self.Dampers = [[None for _ in range(2)] for _ in range(2)]
        self.Joints = [[None for _ in range(2)] for _ in range(2)]
    
    def add_body(self, body: Body):
        self.Bodies.append(body)
        n_body = len(self.Bodies)
        for i in range(n_body-1):
            self.Springs[i].append(None)
            self.Dampers[i].append(None)
            self.Joints[i].append(None)
        self.Springs.append([None]*n_body)
        self.Dampers.append([None]*n_body)
        self.Joints.append([None]*n_body)
        
    def set_joint(self, i: int, j: int, joint: Joint, loc_i: np.ndarray, loc_j: np.ndarray):
        if i > j:
            i, j = j, i
            loc_i, loc_j = loc_j, loc_i
        self.Joints[i][j] = joint(self.Bodies[i], self.Bodies[j], loc_i, loc_j)
        
    def draw_ax(self):
        fig, ax = plt.subplots(1,1)
        for body in self.Bodies:
            body.draw_ax(ax)
        ax.set_aspect("equal")
        plt.show()
    
def get_Q(forces: np.ndarray, ns: np.ndarray):
    """concatenate force matrix and momemt vector

    Args:
        forces (np.ndarray): shape is (2, n_body)
        ns (np.ndarray): shape is (n_body)

    Returns:
        Q: concatenated matrix, shape is (n_body, 3)
    """
    Q = np.concatenate((forces, ns[np.newaxis]), axis=0)
    return Q
    



    