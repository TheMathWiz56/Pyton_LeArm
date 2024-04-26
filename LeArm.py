import numpy as np
import math as m
from Constants import LeArmConstants, square
from adafruit_servokit import ServoKit

np.set_printoptions(precision=5, suppress=True, )


# Arm class - generates base to end-effector transform, handles kinematic operations, handles servo periodic??? update
class Arm:
    def __init__(self):
        self.base_to_wrist_frame_transformation = None
        self.link_list = LinkList()
        self.kit = ServoKit(channels=16)
        self.current_setpoint = ArmSetpoint()
        self.kinematics = ArmKinematics(self.current_setpoint)

        self.kit.servo[LeArmConstants.PINS.WRIST.value].set_pulse_width_range(500, 2550)

        self.update_base_to_wrist_frame_transformation()

    def update_base_to_wrist_frame_transformation(self):
        self.base_to_wrist_frame_transformation = get_forward_kinematics(self.link_list)

    # Allows servos to be controlled outside the Arm object
    def get_kit(self):
        return self.kit

    # "raw" suggests that the user should be careful when utilizing this method as it bypasses the inverse kinematics
    def update_servos_setpoints_raw(self, outputs: list):
        i = 0
        for output in outputs:
            if output is not None:
                if output > 180 or output < 0:
                    self.set_setpoint_to_servo_raw(90, LeArmConstants.PINS(i).value)
                    print("Input Exceeded Allowed Range.")
                else:
                    self.set_setpoint_to_servo_raw(output, LeArmConstants.PINS(i).value)
            i += 1

    def set_setpoint_to_servo_raw(self, value, pin: int):
        self.kit.servo[pin].angle = value

    def go_to(self, x=None, y=None, z=None, pitch=None, roll=None):
        (servo_outputs, theta_list) = self.kinematics.solve(x, y, z, pitch, roll)
        self.update_servos_setpoints_raw(servo_outputs)
        self.link_list.update_joint_revolute_variables(theta_list)
        self.update_base_to_wrist_frame_transformation()

    def __str__(self):
        return self.base_to_wrist_frame_transformation


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


def get_forward_kinematics(link_list: LinkList):
    base_to_wrist_frame_transformation = np.identity(4)

    for link in link_list.get_list_reversed():
        base_to_wrist_frame_transformation = np.matmul(link.get_homogeneous_transform(),
                                                       base_to_wrist_frame_transformation)
    return base_to_wrist_frame_transformation


class ArmSetpoint:
    def __init__(self):
        self.x = 0
        self.y = 0
        self.z = 0
        self.pitch = 0
        self.roll = 0

        self.theta1 = LeArmConstants.SHOULDER_VERTICAL
        self.theta2 = LeArmConstants.ELBOW1_VERTICAL
        self.theta3 = LeArmConstants.ELBOW2_VERTICAL
        self.theta4 = LeArmConstants.ELBOW3_VERTICAL
        self.theta5 = LeArmConstants.WRIST_VERTICAL
        self.theta6 = LeArmConstants.GripperState.MIDDLE.value

    def update_setpoints(self, setpoint_as_list):
        self.x = setpoint_as_list[0]
        self.y = setpoint_as_list[1]
        self.z = setpoint_as_list[2]
        self.pitch = setpoint_as_list[3]
        self.roll = setpoint_as_list[4]

        self.theta1 = setpoint_as_list[5]
        self.theta2 = setpoint_as_list[6]
        self.theta3 = setpoint_as_list[7]
        self.theta4 = setpoint_as_list[8]
        self.theta5 = setpoint_as_list[9]
        self.theta6 = setpoint_as_list[10]

    def get_x_z_length(self):
        return m.sqrt(square(self.x) + square(self.z))

    def get_setpoint_as_list(self):
        return [self.x, self.y, self.z, self.pitch, self.roll, self.theta1, self.theta2, self.theta3, self.theta4,
                self.theta5, self.theta6]

    def get_theta_list(self):
        return [self.theta1, self.theta2, self.theta3, self.theta4, self.theta5, self.theta6]

    def __str__(self):
        return self.get_setpoint_as_list().__str__()


class ArmKinematics:
    def __init__(self, current_setpoint: ArmSetpoint):
        # Should be set to default position, i.e. vertical
        self.current_setpoint = current_setpoint
        self.past_setpoint = ArmSetpoint()

    # Careful to only use after the gripper vector has been removed from the arm setpoint
    def get_x_z_length(self):
        return m.sqrt(square(self.current_setpoint.x) + square(self.current_setpoint.z))

    def check_x_z_coordinate(self):
        x = self.current_setpoint.x
        z = self.current_setpoint.z

        print(f"Desired x {x}\nDesired z {z}")

        if m.sqrt(square(x) + square(z)) > LeArmConstants.LINK2_LENGTH + LeArmConstants.LINK3_LENGTH:
            print("Desired (x,z) cannot be achieved \nScaled to max extension")
            scaler = (LeArmConstants.LINK2_LENGTH + LeArmConstants.LINK3_LENGTH) / self.get_x_z_length()
            self.current_setpoint.x = scaler * x
            self.current_setpoint.z = scaler * z

    def move_current_to_past_setpoint(self):
        self.past_setpoint.update_setpoints(self.current_setpoint.get_setpoint_as_list())

    def compare(self, set1, set2):
        past_angles = self.past_setpoint.get_theta_list()
        travel1 = 0
        travel2 = 0
        for i in range(len(past_angles)):
            if set1[i] is not None:
                travel1 = travel1 + m.fabs(set1[i] - past_angles[i])
            if set2[i] is not None:
                travel2 = travel2 + m.fabs(set2[i] - past_angles[i])

        if travel1 > travel2:
            return set2
        return set1

    def solve(self, x, y, z, pitch, roll):
        self.move_current_to_past_setpoint()
        new_point = False
        if x is not None:
            self.current_setpoint.x = x
            new_point = True
        if y is not None:
            self.current_setpoint.y = y
            new_point = True
        if z is not None:
            self.current_setpoint.z = z
            new_point = True
        if pitch is not None:
            self.current_setpoint.pitch = pitch
            new_point = True
        if roll is not None:
            self.current_setpoint.roll = roll
            new_point = True

        if new_point:
            self.current_setpoint.theta6 = LeArmConstants.GripperState.MIDDLE.value
            planar_3_axis_solution = self.solve_3_axis_planar()
            return (
                [LeArmConstants.SHOULDER_VERTICAL, (90 + LeArmConstants.ELBOW1_VERTICAL) - planar_3_axis_solution[1],
                 LeArmConstants.ELBOW2_VERTICAL + planar_3_axis_solution[2], (LeArmConstants.ELBOW3_VERTICAL - 90)
                 + planar_3_axis_solution[3], LeArmConstants.GripperState.MIDDLE.value],
                [LeArmConstants.SHOULDER_VERTICAL, planar_3_axis_solution[1], planar_3_axis_solution[2],
                 planar_3_axis_solution[3], LeArmConstants.WRIST_VERTICAL])

    def solve_3_axis_planar(self):
        # First remove the gripper vector from the arm position vector
        print("Inputted Coordinates:" + self.current_setpoint.__str__())
        print("Theta6: " + str(self.current_setpoint.theta6))
        gripper_length = (m.sin(self.current_setpoint.theta6) * LeArmConstants.GRIPPER_EVEN_BAR_LINK_LENGTH +
                          LeArmConstants.WRIST_TO_GRIPPER_DISTANCE)

        gripper_v_x = m.cos(self.current_setpoint.pitch) * gripper_length
        gripper_v_z = m.sin(self.current_setpoint.pitch) * gripper_length
        self.current_setpoint.x = self.current_setpoint.x - gripper_v_x
        self.current_setpoint.z = self.current_setpoint.z - gripper_v_z

        print("Gripper Removed Coordinates:" + self.current_setpoint.__str__())

        x = self.current_setpoint.x
        z = self.current_setpoint.z

        self.check_x_z_coordinate()
        print((square(self.get_x_z_length()) - square(LeArmConstants.LINK2_LENGTH) - square(
            LeArmConstants.LINK3_LENGTH)))
        theta3p = -(m.acos((square(self.get_x_z_length()) - square(LeArmConstants.LINK2_LENGTH) - square(
            LeArmConstants.LINK3_LENGTH)) /
                           (2 * LeArmConstants.LINK2_LENGTH * LeArmConstants.LINK3_LENGTH)))
        theta3n = -theta3p

        beta = m.atan2(z, x)
        psi = m.acos((square(self.get_x_z_length()) + square(LeArmConstants.LINK2_LENGTH) - square(
            LeArmConstants.LINK3_LENGTH)) /
                     (2 * LeArmConstants.LINK2_LENGTH * self.get_x_z_length()))

        theta2p = beta + psi
        theta2n = beta - psi

        theta4p = self.current_setpoint.pitch - theta2p - theta3p
        theta4n = self.current_setpoint.pitch - theta2n - theta3n

        return self.compare([None, theta2p, theta3p, theta4p, LeArmConstants.GripperState.MIDDLE.value, None],
                            [None, theta2n, theta3n, theta4n, LeArmConstants.GripperState.MIDDLE.value, None])
