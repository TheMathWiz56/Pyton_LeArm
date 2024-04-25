import numpy as np
import math as m
from Constants import LeArmConstants, PINS, square
from adafruit_servokit import ServoKit

np.set_printoptions(precision=5, suppress=True, )


# Arm class - generates base to end-effector transform, handles kinematic operations, handles servo periodic??? update
class Arm:
    def __init__(self):
        self.base_to_wrist_frame_transformation = None
        self.link_list = LinkList()
        self.kit = ServoKit(channels=16)

        self.update_base_to_wrist_frame_transformation()

    def update_base_to_wrist_frame_transformation(self):
        self.base_to_wrist_frame_transformation = get_forward_kinematics(self.link_list)

    def get_kit(self):
        return self.kit

    def update_servos_setpoints(self, outputs: list):
        i = 0
        for output in outputs:
            if output is not None:
                self.set_setpoint_to_servo(output, PINS(i).value)
            i += 1

    def set_setpoint_to_servo(self, value, pin: int):
        self.kit.servo[pin].angle = value

    def __str__(self):
        return self.base_to_wrist_frame_transformation


# Link class - generates homogeneous transforms
class Link:
    def __init__(self, link_parameters):
        # a, alpha, d, theta, link_type
        self.link_parameters = LinkParameters(link_parameters[0], link_parameters[1], link_parameters[2],
                                              link_parameters[3])
        self.link_type = link_parameters[4]

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


class LinkList:
    def __init__(self):
        self.base_link = Link(LeArmConstants.LINK_PARAMETERS[0])
        self.elbow_1_link = Link(LeArmConstants.LINK_PARAMETERS[1])
        self.elbow_2_link = Link(LeArmConstants.LINK_PARAMETERS[2])
        self.elbow_3_link = Link(LeArmConstants.LINK_PARAMETERS[3])
        self.wrist_link = Link(LeArmConstants.LINK_PARAMETERS[4])

        self.list = [self.base_link, self.elbow_1_link, self.elbow_2_link, self.elbow_3_link, self.wrist_link]
        self.list_reversed = [self.wrist_link, self.elbow_3_link, self.elbow_2_link, self.elbow_1_link, self.base_link]

    def get_list(self):
        return self.list

    def get_list_reversed(self):
        return self.list_reversed


class ServoList:
    def __init__(self):
        self.servo_angle_list = [0, 0, 0, 0, 0, 0]
        self.base_angle = 0
        self.elbow_1_angle = 0
        self.elbow_2_angle = 0
        self.elbow_3_angle = 0
        self.wrist_angle = 0
        self.gripper_angle = 0

    def update_servo_angle_list(self, angles_list):
        for i in range(len(angles_list)):
            self.servo_angle_list[i] = angles_list[i]
        self.update_servo_angles()

    def update_servo_angles(self):
        self.base_angle = self.servo_angle_list[0]
        self.elbow_1_angle = self.servo_angle_list[1]
        self.elbow_2_angle = self.servo_angle_list[2]
        self.elbow_3_angle = self.servo_angle_list[3]
        self.wrist_angle = self.servo_angle_list[4]
        self.gripper_angle = self.servo_angle_list[5]


def get_forward_kinematics(link_list: LinkList):
    base_to_wrist_frame_transformation = np.identity(4)

    for link in link_list.get_list_reversed():
        base_to_wrist_frame_transformation = np.matmul(link.get_homogeneous_transform(),
                                                       base_to_wrist_frame_transformation)
    return base_to_wrist_frame_transformation


class ArmKinematics:
    def __init__(self, x, y, z, roll, pitch):
        self.x = x
        self.y = y
        self.z = z
        self.roll = roll
        self.pitch = pitch

    def update_setpoint(self, x, y, z, roll, pitch):
        self.x = x
        self.y = y
        self.z = z
        self.roll = roll
        self.pitch = pitch

    def get_x_z_length(self):
        return m.sqrt(square(self.x) + square(self.z))

    def check_x_z_coordinate(self):
        if m.sqrt(square(self.x) + square(self.z)) > LeArmConstants.LINK2_LENGTH + LeArmConstants.LINK3_LENGTH:
            print("Desired (x,z) cannot be achieved \nScaled to max extension")
            scaler = (LeArmConstants.LINK2_LENGTH + LeArmConstants.LINK3_LENGTH) / self.get_x_z_length()
            self.x = scaler * self.x
            self.z = scaler * self.z

    def get_inverse_kinematics(self):
        pass


class planar_3_axis_inverse_kinematics(ArmKinematics):
    # Should return an array of joint angles starting from Shoulder going to Wrist
    # In this case Shoulder and Wrist will be 0 always
    def get_inverse_kinematics(self):
        super().check_x_z_coordinate()
        theta2p = m.acos((square(super().get_x_z_length()) - square(LeArmConstants.LINK2_LENGTH) - square(
            LeArmConstants.LINK3_LENGTH)) /
                         (2 * LeArmConstants.LINK2_LENGTH * LeArmConstants.LINK3_LENGTH))
        theta2n = -theta2p

        beta = m.atan2(self.z, self.x)
        phi = m.acos((square(super().get_x_z_length()) + square(LeArmConstants.LINK2_LENGTH) - square(
            LeArmConstants.LINK3_LENGTH)) /
                     (2 * LeArmConstants.LINK2_LENGTH * super().get_x_z_length()))

        theta1p = beta + phi
        theta1n = beta - phi

        return [theta1p, theta2p, theta1n, theta2n]
