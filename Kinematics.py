from Constants import square
from Constants import LeArmConstants
import math as m
import numpy as np
import LeArm


def get_forward_kinematics(link_list: LeArm.LinkList):
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
        theta2p = m.acos((square(super().get_x_z_length()) - square(LeArmConstants.LINK2_LENGTH) - square(LeArmConstants.LINK3_LENGTH)) /
                         (2 * LeArmConstants.LINK2_LENGTH * LeArmConstants.LINK3_LENGTH))
        theta2n = -theta2p

        beta = m.atan2(self.z, self.x)
        phi = m.acos((square(super().get_x_z_length()) + square(LeArmConstants.LINK2_LENGTH) - square(LeArmConstants.LINK3_LENGTH)) /
                     (2 * LeArmConstants.LINK2_LENGTH * super().get_x_z_length()))

        theta1p = beta + phi
        theta1n = beta - phi

        return [theta1p, theta2p, theta1n, theta2n]
