import unittest
from robot_api.core.kinematics import *
from math import pi
import numpy as np

def err(x):
    return x*x

class KinematicsTest(unittest.TestCase):
    """ TEST ALL KINEMATICS FUNCTIONS """

    @classmethod
    def setUpClass(cls):
        print("========== Set up Kinematics test ==========\n")

    @classmethod
    def tearDownClass(cls):
        print("========== Tear down Kinematics test ==========\n")

    def setUp(self):
        pass

    def tearDown(self):
        pass

    def test_four_link_angle(self):
        """ FOUR LINK ANGLE TEST """
        assert(err(four_link_angle(-pi/4) - 0.68138) < 1e-4)
        assert(err(four_link_angle(0) - 1.57199) < 1e-4)
        assert(err(four_link_angle(pi/4) - 2.46874) < 1e-4)

    def test_four_link_angle_fast(self):
        """ FOUR LINK ANGLE (FAST) TEST """
        for x in range(0, int(pi * 100)):
            assert(four_link_angle_fast(x/100) == x/100)

    def test_direct_kinematics(self):
        """ DIRECT KINEMATICS TEST """
        # leg is fully expanded
        result = direct([0, -pi/2, -pi/4])
        X, Y, Z = result[0], result[1], result[2]
        assert(err(X - -0.160366) < 1e-3)
        assert(err(Y - 0) < 1e-3)
        assert(err(Z - 0.149703) < 1e-3)

        # leg is fully closed
        result = direct([0, 0, pi/4])
        X, Y, Z = result[0], result[1], result[2]
        assert(err(X - 0.142443) < 1e-3)
        assert(err(Y - 0) < 1e-3)
        assert(err(Z - 0.0550596) < 1e-3)

        # leg is bent 90 degree
        result = direct([pi/2, 0, 0])
        X, Y, Z = result[0], result[1], result[2]
        assert(err(X - 0.174035) < 1e-3)
        assert(err(Y - 0.145709) < 1e-3)
        assert(err(Z - 0) < 1e-3)
    

    def test_jacobian(self):
        """ JACOBIAN TEST """
        J = jacobian([pi/2, 0, 0])
        expected_J = np.array([0, 0.0994888, 0.0137636, \
                               0, -0.154015, -0.121244, \
                                -0.145709, 0, 0]).reshape((3, 3))
        assert(np.linalg.norm(expected_J - J, 2) < 1e-4)

        J = jacobian([0, -pi/2, -pi/4])
        expected_J = np.array([0, 0.103483, 0.0730283, \
                               0.149703, 0, 0, \
                               0, 0.180386, 0.114652]).reshape((3, 3))
        assert(np.linalg.norm(expected_J - J, 2) < 1e-4)
    
    def test_inversed_kinematics(self):
        """ INVERSED KINEMATICS TEST """
        cache_inversed_kinematics()
        arr = [
            [0,     -pi/2,  -pi/4],
            [pi/16, -pi/2,  -pi/4],
            [pi/8,  -pi/2,  -pi/4],

            [0,     -pi/4,  -pi/4],
            [pi/16, -pi/4,  -pi/4],
            [pi/8,  -pi/4,  -pi/4],

            [0,     0,      -pi/4],
            [pi/16, 0,      -pi/4],
            [pi/8,  0,      -pi/4],


            [0,     -pi/2,  0],
            [pi/16, -pi/2,  0],
            [pi/8,  -pi/2,  0],

            [0,     -pi/4,  0],
            [pi/16, -pi/4,  0],
            [pi/8,  -pi/4,  0],

            [0,     0,      0],
            [pi/16, 0,      0],
            [pi/8,  0,      0],


            [0,     -pi/2,  pi/4],
            [pi/16, -pi/2,  pi/4],
            [pi/8,  -pi/2,  pi/4],

            [0,     -pi/4,  pi/4],
            [pi/16, -pi/4,  pi/4],
            [pi/8,  -pi/4,  pi/4],

            [0,     0,      pi/4],
            [pi/16, 0,      pi/4],
            [pi/8,  0,      pi/4],
        ]
        for i, q in enumerate(arr):
            print("Test point #{}".format(i+1))
            result = inversed(direct(q))
            norm = np.linalg.norm(np.array(q) - result)
            print("norm={}".format(norm))
