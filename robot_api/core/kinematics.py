from math import sin, cos, sqrt, acos, asin, pi
from .constants import A0, A1, A2, A3, A5, A6, A7, A8, A9, A10
import numpy as np


def direct(phi1, phi2, phi3):
    """ 
    SOLVING THE DIRECT KINEMATICS PROBLEM \n
    The angles should be indicated in the coordinates
    of the kinematic problem, in radians
    """
    phi4 = four_link_angle(phi3) - (pi/2)
    X = A2 + A3 * cos(phi2) + A6 * sin(phi2) + A8 * sin(phi2 + phi4) \
        + (A9 + A10) * cos(phi2 + phi4)
    V = A1 - A3 * sin(phi2) + A6 * cos(phi2) + A8 * cos(phi2 + phi4) \
        - (A9 + A10) * sin(phi2 + phi4)
    Y = V * sin(phi1)
    Z = V * cos(phi1)
    return (X, Y, Z)


def four_link_angle(phi3):
    """ returns angle of the last joint """
    d = sqrt(A5**2 + A7**2 - 2 * A5 * A7 * cos(phi3 + pi/2))
    gamma = asin((A5 / d) * cos(phi3))
    delta = acos((d**2 + A9**2 - A7**2) / (2 * d * A9))
    return (pi - gamma - delta)


def four_link_angle_fast(phi3):
    """ phi4 is approx equal to phi3 """
    return phi3


def jacobian(phi1, phi2, phi3):
    h = pi * 1e-4 # diff step
    F = np.array(direct(phi1, phi2, phi3))
    dFdPhi1 = (np.array(direct(phi1 + h, phi2, phi3)) - F) / h
    dFdPhi2 = (np.array(direct(phi1, phi2 + h, phi3)) - F) / h
    dFdPhi3 = (np.array(direct(phi1, phi2, phi3 + h)) - F) / h
    return np.array([dFdPhi1, dFdPhi2, dFdPhi3]).reshape((3, 3)).T
     
    