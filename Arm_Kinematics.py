import numpy as np
import math as m
from Constants import LeArmConstants as LeArm
from Constants import LinkType

np.set_printoptions(precision=5, suppress=True, )

# Arm class - generates base to end-effector transform, handles kinematic operations
class Arm:
    def __init__(self):
        self.base2end_effector_transformation = None
        self.link_list = []

        for link_parameters in LeArm.LINK_PARAMETERS:
            alpha, a, d, theta, link_type = link_parameters
            self.link_list.append(Link(a, alpha, d, theta, link_type))

        self.update_base2end_effector_transformation()

    def update_base2end_effector_transformation(self):
        self.base2end_effector_transformation = np.identity(4)
        for link in reversed(self.link_list):
            self.base2end_effector_transformation = np.matmul(link.get_homogeneous_transform(), self.base2end_effector_transformation)

    def __str__(self):
        return self.base2end_effector_transformation


# Link class - generates homogeneous transforms and updates servo values
class Link:
    def __init__(self, a, alpha, d, theta, link_type):
        self.link_parameters = LinkParameters(a, alpha, d, theta)
        self.link_type = link_type

        self.homogeneous_transform = None
        self.update_homogeneous_transform()

    def get_link_type(self):
        return self.link_type

    def update_homogeneous_transform(self):
        a, alpha, d, theta = self.link_parameters.get_link_parameters()

        self.homogeneous_transform = np.array([
            [m.cos(theta), -m.sin(theta), 0, a],
            [m.sin(theta) * m.cos(alpha), m.cos(theta) * m.cos(alpha), -m.sin(alpha), -m.sin(alpha) * d],
            [m.sin(theta) * m.sin(alpha), m.cos(theta) * m.sin(alpha), m.cos(alpha), m.cos(alpha) * d],
            [0, 0, 0, 1]
        ])

    def get_homogeneous_transform(self):
        return self.homogeneous_transform

    def __str__(self):
        return self.homogeneous_transform


# Link Parameters class - keeps track of link parameters and updates the joint variable with the newest data
class LinkParameters:
    def __init__(self, a, alpha, d, theta):
        self.a = a
        self.alpha = alpha
        self.d = d
        self.theta = theta

        self.link_parameters = np.array([self.a, self.alpha, self.d, self.theta])

    def get_link_parameters(self):
        return self.link_parameters

    def set_link_parameters(self, a, alpha, d, theta):
        self.a = a
        self.alpha = alpha
        self.d = d
        self.theta = theta

    def set_d(self, d):
        self.d = d

    def set_theta(self, theta):
        self.theta = theta

    def __str__(self):
        return "Link Parameters" + "\n" + self.link_parameters.__str__()
