import numpy as np


class Arm:
    pass


class Link:
    pass


class Link_Parameters:
    def __init__(self, a, alpha, d=None, theta=None):
        self.a = a
        self.alpha = alpha
        self.d = d
        self.theta = theta

        if d is None:
            self.is_prismatic = True
        else:
            self.is_revolute = True

    def get_link_parameters(self):
        if self.is_revolute:
            return np.array([self.a, self.alpha, self.d])
        return np.array([self.a, self.alpha, self.theta])


class Homogeneous_Transform:
    def __init__(self, link_parameters):
        self.link_parameters = link_parameters

    def get_homogeneous_transform(self, joint_variable, ):
        return
