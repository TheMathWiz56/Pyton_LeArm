import numpy as np
import math as m
from Constants import LeArmConstants, PINS
import Kinematics
from adafruit_servokit import ServoKit

np.set_printoptions(precision=5, suppress=True, )


# Arm class - generates base to end-effector transform, handles kinematic operations, handles servo periodic??? update
class Arm:
    def __init__(self):
        self.base_to_wrist_frame_transformation = None
        self.link_list = []
        self.kit = ServoKit(channels=16)
        self.servo_setpoints = []

        for link_parameters in LeArmConstants.LINK_PARAMETERS:
            alpha, a, d, theta, link_type = link_parameters
            self.link_list.append(Link(a, alpha, d, theta, link_type))

        self.update_base_to_wrist_frame_transformation()

    def update_base_to_wrist_frame_transformation(self):
        self.base_to_wrist_frame_transformation = Kinematics.get_forward_kinematics(self.link_list)

    def get_kit(self):
        return self.kit

    def update_servos_setpoints(self, outputs: list):
        i = 0
        for output in outputs:
            self.set_setpoint_to_servo(output, PINS(i).value)
            i += 1

    def set_setpoint_to_servo(self, value, pin: int):
        if value is not None:
            self.kit.servo[pin].angle = value

    def __str__(self):
        return self.base_to_wrist_frame_transformation


# Link class - generates homogeneous transforms
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
