import numpy as np
from Body import Body

class Spring:
    def __init__(self, coef: float, l0: float):
        self.coef = coef
        self.l0 = l0
        
    def get_force(self, d_AB: np.asarray):
        """get force by spring

        Args:
            d_AB (np.asarray): displacement of both ends of spring

        Returns:
            force: force vector
        """
        assert d_AB.shape == (2,), "invalid shape"
        force = self.coef * d_AB
        return force
    
class Damper:
    def __init__(self, coef: float):
        self.coef = coef
        
    def get_force(self, dd_AB: np.ndarray):
        """get force by damper

        Args:
            d_AB (np.asarray): displacement rate of both ends of damper

        Returns:
            force: force vector
        """
        assert dd_AB.shape == (2,), "invalid shape"
        force = self.coef * dd_AB
        return force
    
class Gravity():
    def __init__(self, body: Body):
        self.m = body.get_m()
        self.g = 9.8165
        self.force = self.m * self.g
        
    def get_force(self):
        return self.force
