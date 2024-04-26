import numpy as np
import math as m
from Constants import LeArmConstants, PINS, square, LinkType
from adafruit_servokit import ServoKit

np.set_printoptions(precision=5, suppress=True, )


# Arm class - generates base to end-effector transform, handles kinematic operations, handles servo periodic??? update
class Arm:
    def __init__(self):
        self.base_to_wrist_frame_transformation = None
        self.link_list = LinkList()
        self.kit = ServoKit(channels=16)
        self.kinematics = ArmKinematics()

        self.kit.servo[PINS.GRIPPER.value].set_pulse_width_range(500, 2500)

        self.update_base_to_wrist_frame_transformation()

    def update_base_to_wrist_frame_transformation(self):
        self.base_to_wrist_frame_transformation = get_forward_kinematics(self.link_list)

    def get_kit(self):
        return self.kit

    # "raw" suggests that the user should be careful when utilizing this method as it bypasses the inverse kinematics
    def update_servos_setpoints_raw(self, outputs: list):
        i = 0
        for output in outputs:
            if output is not None:
                self.set_setpoint_to_servo_raw(output, PINS(i).value)
            i += 1

    def set_setpoint_to_servo_raw(self, value, pin: int):
        self.kit.servo[pin].angle = value

    def __str__(self):
        return self.base_to_wrist_frame_transformation


# Link class - generates homogeneous transforms
class Link:
    def __init__(self, link_parameters, link_type: LinkType):
        # a, alpha, d, theta, link_type
        self.link_parameters = LinkParameters(link_parameters[0], link_parameters[1], link_parameters[2],
                                              link_parameters[3])
        self.link_type = link_type

        self.homogeneous_transform = None
        self.update_homogeneous_transform()

    def update_joint_variable(self, value):
        if self.link_type.value == LinkType.REVOLUTE_LINK.value:
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


def get_forward_kinematics(link_list: LinkList):
    base_to_wrist_frame_transformation = np.identity(4)

    for link in link_list.get_list_reversed():
        base_to_wrist_frame_transformation = np.matmul(link.get_homogeneous_transform(),
                                                       base_to_wrist_frame_transformation)
    return base_to_wrist_frame_transformation


class ArmSetpoint:
    def __init__(self, x, y, z, pitch, roll):
        self.x = x
        self.y = y
        self.z = z
        self.pitch = pitch
        self.roll = roll

    def update_setpoints(self, x, y, z, roll, pitch):
        self.x = x
        self.y = y
        self.z = z
        self.pitch = pitch
        self.roll = roll

    def get_x_z_length(self):
        return m.sqrt(square(self.x) + square(self.z))


class ArmKinematics:
    def __init__(self):
        # Should be set to default position, i.e. vertical
        self.setpoint_list = ArmSetpoint(0, 0, 300, 0, 0)

    def update_setpoint(self, setpoint_list: ArmSetpoint):
        self.setpoint_list = setpoint_list

    def get_x_z_length(self):
        return m.sqrt(square(self.x) + square(self.z))

    def check_x_z_coordinate(self):
        x = self.setpoint_list.x
        z = self.setpoint_list.z

        if m.sqrt(square(x) + square(z)) > LeArmConstants.LINK2_LENGTH + LeArmConstants.LINK3_LENGTH:
            print("Desired (x,z) cannot be achieved \nScaled to max extension")
            scaler = (LeArmConstants.LINK2_LENGTH + LeArmConstants.LINK3_LENGTH) / self.get_x_z_length()
            self.setpoint_list.x = scaler * x
            self.setpoint_list.z = scaler * z

    def solve(self):
        pass


class planar_3_axis_inverse_kinematics(ArmKinematics):
    # Should return an array of joint angles starting from Shoulder going to Wrist
    # In this case y and roll will be 0 always
    def get_inverse_kinematics(self):
        # First remove the gripper vector from the arm position vector
        gripper_v_x = m.cos(self.setpoint_list.pitch) * LeArmConstants.GRIPPER_EVEN_BAR_LINK_LENGTH
        gripper_v_z = m.sin(self.setpoint_list.pitch) * LeArmConstants.GRIPPER_EVEN_BAR_LINK_LENGTH
        self.setpoint_list.x = self.setpoint_list.x - gripper_v_x
        self.setpoint_list.z = self.setpoint_list.z - gripper_v_z

        x = self.setpoint_list.x
        z = self.setpoint_list.z

        super().check_x_z_coordinate()
        theta2p = m.acos((square(super().get_x_z_length()) - square(LeArmConstants.LINK2_LENGTH) - square(
            LeArmConstants.LINK3_LENGTH)) /
                         (2 * LeArmConstants.LINK2_LENGTH * LeArmConstants.LINK3_LENGTH))
        theta2n = -theta2p

        beta = m.atan2(z, x)
        phi = m.acos((square(super().get_x_z_length()) + square(LeArmConstants.LINK2_LENGTH) - square(
            LeArmConstants.LINK3_LENGTH)) /
                     (2 * LeArmConstants.LINK2_LENGTH * super().get_x_z_length()))

        theta1p = beta + phi
        theta1n = beta - phi

        return [theta1p, theta2p, theta1n, theta2n]
