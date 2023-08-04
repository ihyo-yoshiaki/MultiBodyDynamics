import numpy as np

V = np.array([[0., -1,], [1., 0.]], dtype="float")

def get_rot_mat(theta: float):
    """get rotation matrix

    Args:
        theta (float): angle [rad]

    Returns:
        rotation matrix: 
    """
    c = np.cos(theta)
    s = np.sin(theta)
    return np.array([[c, -s],[s, c]], dtype="float")

def def_rot_mat(A: np.ndarray):
    """get differetiate rotation matrix

    Args:
        A (np.asarray): rotation matrix

    Returns:
        dA_dt: differetiation of rotation matrix
    """
    assert A.shape == (2,2), "invalid shape"
    return np.dot(np.array([[0.,-1.],[1.,0]]), A)

def get_moment(r: np.ndarray, force: np.ndarray):
    assert r.shape == (2,), "r is invalid shape"
    assert force.shape == (2,), "force is invalid shape"
    return np.dot(r, force.T)

def get_B_accel_eq(C, Cq, Ct, Cqt, Ctt, Cqdqq, dq, alpha=100., beta=100.):
    term1 = - (np.dot(Cqdqq,dq) + 2 * np.dot(Cqt, dq) + Ctt)
    term2 = - 2 * alpha * (np.dot(Cq, dq) + Ct)
    term3 = - np.square(beta) * C
    beta_gamma = term1 + term2 + term3
    return beta_gamma
    
def get_accel_eq(Cqdqq, Cqt, Ctt, dq):
    gamma = np.dot(Cqdqq, dq) + 2 * np.dot(Cqt, dq) + Ctt
    return gamma
    
def get_vel_eq(Cqdqq, dq, Cqt, Ctt):
    pass