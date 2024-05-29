from Constants import LeArmConstants
import math as m
import numpy as np


# Link class - generates homogeneous transforms
class Link:
    def __init__(self, link_parameters, link_type: LeArmConstants.LinkType):
        # a, alpha, d, theta, link_type
        self.link_parameters = LinkParameters(link_parameters[0], link_parameters[1], link_parameters[2],
                                              link_parameters[3])
        self.link_type = link_type

        self.homogeneous_transform = None
        self.update_homogeneous_transform()

    def update_joint_variable(self, value):
        if self.link_type.value == LeArmConstants.LinkType.REVOLUTE_LINK.value:
            self.link_parameters.theta = value
        else:
            self.link_parameters.d = value

    def update_homogeneous_transform(self):
        a = self.link_parameters.a
        alpha = self.link_parameters.alpha
        d = self.link_parameters.d
        theta = self.link_parameters.theta

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
# This class should only be used by Link class
class LinkParameters:
    def __init__(self, a, alpha, d, theta):
        self.a = a
        self.alpha = alpha
        self.d = d
        self.theta = theta

        self.link_parameters = np.array([self.a, self.alpha, self.d, self.theta])

    def __str__(self):
        return "Link Parameters" + "\n" + self.link_parameters.__str__()


class LinkList:
    def __init__(self):
        self.base_link = Link(LeArmConstants.LINK_PARAMETERS[0][:4], LeArmConstants.LINK_PARAMETERS[0][4])
        self.elbow_1_link = Link(LeArmConstants.LINK_PARAMETERS[1][:4], LeArmConstants.LINK_PARAMETERS[1][4])
        self.elbow_2_link = Link(LeArmConstants.LINK_PARAMETERS[2][:4], LeArmConstants.LINK_PARAMETERS[2][4])
        self.elbow_3_link = Link(LeArmConstants.LINK_PARAMETERS[3][:4], LeArmConstants.LINK_PARAMETERS[3][4])
        self.wrist_link = Link(LeArmConstants.LINK_PARAMETERS[4][:4], LeArmConstants.LINK_PARAMETERS[4][4])

        self.list = [self.base_link, self.elbow_1_link, self.elbow_2_link, self.elbow_3_link, self.wrist_link]
        self.list_reversed = [self.wrist_link, self.elbow_3_link, self.elbow_2_link, self.elbow_1_link, self.base_link]

    def get_list_reversed(self):
        return self.list_reversed

    def update_joint_revolute_variables(self, values: list):
        self.base_link.link_parameters.theta = values[0]
        self.elbow_1_link.link_parameters.theta = values[1]
        self.elbow_2_link.link_parameters.theta = values[2]
        self.elbow_3_link.link_parameters.theta = values[3]
        self.wrist_link.link_parameters.theta = values[4]
