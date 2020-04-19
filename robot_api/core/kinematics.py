from math import sin, cos, sqrt, acos, asin, pi
from .constants import A0, A1, A2, A3, A5, A6, A7, A8, A9, A10, PHI1_RANGE, PHI2_RANGE, PHI3_RANGE
import numpy as np


cache = []


def direct(angles, fast=False):
    """ 
    SOLVING THE DIRECT KINEMATICS PROBLEM \n
    The angles should be indicated in the coordinates
    of the kinematic problem, in radians
    """
    phi1, phi2, phi3 = angles[0], angles[1], angles[2]
    phi4 = four_link_angle(phi3) - (pi/2) if not fast else phi3
    X = A2 + A3 * cos(phi2) + A6 * sin(phi2) + A8 * sin(phi2 + phi4) \
        + (A9 + A10) * cos(phi2 + phi4)
    V = A1 - A3 * sin(phi2) + A6 * cos(phi2) + A8 * cos(phi2 + phi4) \
        - (A9 + A10) * sin(phi2 + phi4)
    Y = V * sin(phi1)
    Z = V * cos(phi1)
    return np.array([X, Y, Z])


def four_link_angle(phi3):
    """ returns angle of the last joint """
    d = sqrt(A5**2 + A7**2 - 2 * A5 * A7 * cos(phi3 + pi/2))
    gamma = asin((A5 / d) * cos(phi3))
    delta = acos((d**2 + A9**2 - A7**2) / (2 * d * A9))
    return (pi - gamma - delta)


def four_link_angle_fast(phi3):
    """ phi4 is approx equal to phi3 """
    return phi3


def jacobian(angles):
    h = pi * 1e-4 # diff step
    F = direct(angles)
    eye = np.eye(3) * h # matrix with all h in diagonal elements
    dFdPhi1 = (direct(angles + eye[0]) - F) / h
    dFdPhi2 = (direct(angles + eye[1]) - F) / h
    dFdPhi3 = (direct(angles + eye[2]) - F) / h
    return np.array([dFdPhi1, dFdPhi2, dFdPhi3]).reshape((3, 3)).T
    
    
def inversed(coordinates, fast=False):
    """ 
    SOLVING THE INVERSE KINEMATICS PROBLEM \n
    Param coordinates is [X, Y, Z] numpy vector, in meters.
    This function uses Newton's numerical method.
    """
    phis = find_nearest_solution(coordinates)
    error = np.ones((3,), dtype=np.float64)

    i = 0
    while np.linalg.norm(error, 2) > 1e-5 and i < 100:
        J = jacobian(phis)
        X = direct(phis, fast)
        error = X - coordinates
        p = np.matmul(np.linalg.pinv(J), error)
        phis = phis - p
        i += 1
    
    return phis


def find_nearest_solution(coordinates):
    global cache
    return min(cache, key=lambda di: np.linalg.norm((coordinates - di[0]), ord=2))[1]


def cache_inversed_kinematics():
    global cache
    cache = []
    phi1_start, phi1_end = PHI1_RANGE
    phi2_start, phi2_end = PHI2_RANGE
    phi3_start, phi3_end = PHI3_RANGE
    
    for phi1 in np.linspace(phi1_start, phi1_end, num=2):
        for phi2 in np.linspace(phi2_start, phi2_end, num=10):
            for phi3 in np.linspace(phi3_start, phi3_end, num=10):
                angles = [phi1, phi2, phi3]
                cache.append((direct(angles), np.array(angles)))
    

