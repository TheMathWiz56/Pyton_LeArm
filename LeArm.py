import numpy as np
import math as m
from Constants import LeArmConstants, square, get_2D_vector_length
from adafruit_servokit import ServoKit

np.set_printoptions(precision=5, suppress=True, )


# Arm class - generates base to end-effector transform, handles kinematic operations, handles servo periodic??? update
class Arm:
    def __init__(self):
        self.base_to_wrist_frame_transformation = None
        self.link_list = LinkList()
        self.kit = ServoKit(channels=16)
        self.current_setpoint = ArmSetpoint()
        self.past_setpoint = ArmSetpoint()
        self.kinematics = ArmKinematics(self.current_setpoint, self.past_setpoint)

        for servo_settings in LeArmConstants.SERVO_SETTINGS_LIST:
            self.apply_servo_settings(servo_settings)

        self.update_base_to_wrist_frame_transformation()

    def update_base_to_wrist_frame_transformation(self):
        self.base_to_wrist_frame_transformation = get_forward_kinematics(self.link_list)

    # Allows servos to be controlled outside the Arm object
    def get_kit(self):
        return self.kit

    def apply_servo_settings(self, settings: list):
        self.kit.servo[settings[0]].set_pulse_width_range(settings[1], settings[2])

    def update_servos_setpoints_raw(self, outputs: list):
        """
        "raw" suggests that the user should be careful when utilizing this method as it bypasses the inverse kinematics
        :param outputs:
        :return:
        """
        i = 0
        for output in outputs:
            if output is not None:
                if output > 180:
                    self.set_setpoint_to_servo_raw(180, LeArmConstants.PINS(i).value)
                    print("Input Exceeded Allowed Range.")
                elif output < 0:
                    self.set_setpoint_to_servo_raw(0, LeArmConstants.PINS(i).value)
                    print("Input Exceeded Allowed Range.")
                else:
                    self.set_setpoint_to_servo_raw(output, LeArmConstants.PINS(i).value)
            i += 1

    def set_setpoint_to_servo_raw(self, value, pin: int):
        """
        "raw" suggests that the user should be careful when utilizing this method as it bypasses the inverse kinematics
        :param value:
        :param pin:
        :return:
        """
        self.kit.servo[pin].angle = value

    def go_to(self, gripper_setpoint, x=None, y=None, z=None, pitch=None, roll=None):
        """
        :param gripper_setpoint:
        :param x: mm
        :param y: mm
        :param z: mm
        :param pitch: RADIANS
        :param roll: RADIANS
        :return:
        """
        self.kinematics.solve(x, y, z, pitch, roll, gripper_setpoint)

        servo_outputs = self.current_setpoint.get_servo_setpoint_list()
        theta_list = self.current_setpoint.get_raw_theta_list_radians()
        print("Inverse Kinematics solved for: ")
        print(self.current_setpoint.get_raw_theta_list_radians())
        print(servo_outputs)
        print(theta_list)

        # Doesn't include gripper updates
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

        self.theta1 = m.radians(LeArmConstants.SHOULDER_VERTICAL)
        self.theta2 = m.radians(LeArmConstants.ELBOW1_VERTICAL)
        self.theta3 = m.radians(LeArmConstants.ELBOW2_VERTICAL)
        self.theta4 = m.radians(LeArmConstants.ELBOW3_VERTICAL)
        self.theta5 = m.radians(LeArmConstants.WRIST_VERTICAL)
        self.theta6 = m.radians(LeArmConstants.GripperState.MIDDLE.value)

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

    def get_raw_theta_list_radians(self):
        """
        raw means that angles have not been modified to represent real world servo setpoints
        """
        return [self.theta1, self.theta2, self.theta3, self.theta4, self.theta5, self.theta6]

    def get_servo_setpoint_list(self):
        """
        Returns servo angles from raw radians list

        (
                [LeArmConstants.SHOULDER_VERTICAL, (90 + LeArmConstants.ELBOW1_VERTICAL) -
                 m.degrees(planar_3_axis_solution[1]), LeArmConstants.ELBOW2_VERTICAL +
                 m.degrees(planar_3_axis_solution[2]), LeArmConstants.ELBOW3_VERTICAL
                 + m.degrees(planar_3_axis_solution[3]), LeArmConstants.GripperState.MIDDLE.value],
                [m.radians(LeArmConstants.SHOULDER_VERTICAL), planar_3_axis_solution[1], planar_3_axis_solution[2],
                 planar_3_axis_solution[3], m.radians(LeArmConstants.WRIST_VERTICAL)])
        """
        return [m.degrees(self.theta1),
                m.degrees(self.theta2), LeArmConstants.ELBOW2_VERTICAL -
                m.degrees(self.theta3), LeArmConstants.ELBOW3_VERTICAL
                - m.degrees(self.theta4), m.degrees(self.theta5), m.degrees(self.theta6)]

    def __str__(self):
        return self.get_setpoint_as_list().__str__()


class ArmKinematics:
    def __init__(self, current_setpoint: ArmSetpoint, past_setpoint: ArmSetpoint):
        # Should be set to default position, i.e. vertical
        self.current_setpoint = current_setpoint
        self.past_setpoint = past_setpoint
        self.temp_X = 0

    # Careful to only use after the gripper vector has been removed from the arm setpoint
    def get_x_z_length(self):
        return m.sqrt(square(self.current_setpoint.x) + square(self.current_setpoint.z))

    def check_x_z_coordinate(self):
        print(f"Desired x {self.current_setpoint.x}\nDesired z {self.current_setpoint.z}")

        if self.get_x_z_length() > LeArmConstants.LINK2_LENGTH + LeArmConstants.LINK3_LENGTH:
            print("Desired (x,z) cannot be achieved \nScaled to MAX extension")
            self.scale_x_z_coordinate_wrt(LeArmConstants.LINK2_LENGTH + LeArmConstants.LINK3_LENGTH)
        if self.get_x_z_length() < LeArmConstants.LINK2_LENGTH - LeArmConstants.LINK3_LENGTH:
            print("Desired (x,z) cannot be achieved \nScaled to MIN extension")
            self.scale_x_z_coordinate_wrt(LeArmConstants.LINK2_LENGTH - LeArmConstants.LINK3_LENGTH)

    def scale_x_z_coordinate_wrt(self, scale_against):
        scaler = scale_against / self.get_x_z_length()
        self.current_setpoint.x = scaler * self.current_setpoint.x
        self.current_setpoint.z = scaler * self.current_setpoint.z

    def move_current_to_past_setpoint(self):
        self.past_setpoint.update_setpoints(self.current_setpoint.get_setpoint_as_list())

    def compare(self, set1, set2):
        # First check if angles are achievable
        past_angles = self.past_setpoint.get_raw_theta_list_radians()
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

    def solve(self, x, y, z, pitch, roll, gripper_setpoint):
        """
        Takes in the angles in radians and returns the angles in degrees for the servos and raw angles in radians for
        storage

        :param x:
        :param y:
        :param z:
        :param pitch:
        :param roll:
        :param gripper_setpoint:
        :return:
        """
        self.move_current_to_past_setpoint()
        self.current_setpoint.theta6 = m.radians(gripper_setpoint)

        # All values will either be updated or kept the same
        # If they are kept the same, they may be the default 0 or the past value that wasn't updated
        new_point = False
        # Make this prettier
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
            # Base Rotation
            self.temp_X = self.current_setpoint.x
            self.current_setpoint.theta1 = self.solve_for_base()
            print(f"TEST FOR BASE ANGLE Angle: {self.current_setpoint.theta1}")

            planar_3_axis_solution = self.solve_3_axis_planar()
            # Make this prettier
            if planar_3_axis_solution[0] is not None:
                self.current_setpoint.theta1 = planar_3_axis_solution[0]

            # 3-axis solution
            if planar_3_axis_solution[1] is not None:
                self.current_setpoint.theta2 = planar_3_axis_solution[1]
            if planar_3_axis_solution[2] is not None:
                self.current_setpoint.theta3 = planar_3_axis_solution[2]
            if planar_3_axis_solution[3] is not None:
                self.current_setpoint.theta4 = planar_3_axis_solution[3]

            if planar_3_axis_solution[4] is not None:
                self.current_setpoint.theta5 = planar_3_axis_solution[4]
            if planar_3_axis_solution[5] is not None:
                self.current_setpoint.theta6 = planar_3_axis_solution[5]

    def solve_3_axis_planar(self):
        # First remove the gripper vector from the arm position vector
        # It's important that theta6 is updated to its new desired value before it can be removed from the total vector
        #   in order to ensure the proper amount has been removed
        # Need to add shift for elbow1 being off center
        print("Inputted Coordinates:" + self.current_setpoint.__str__())
        print("Theta6: " + str(m.radians(self.current_setpoint.theta6)))
        gripper_length = (m.sin(self.current_setpoint.theta6) * LeArmConstants.GRIPPER_EVEN_BAR_LINK_LENGTH +
                          LeArmConstants.WRIST_TO_GRIPPER_DISTANCE)
        print("Gripper Vector Length:" + gripper_length.__str__())

        gripper_v_x = m.cos(self.current_setpoint.pitch) * gripper_length
        gripper_v_z = m.sin(self.current_setpoint.pitch) * gripper_length
        self.current_setpoint.x = self.current_setpoint.x - gripper_v_x
        self.current_setpoint.z = self.current_setpoint.z - gripper_v_z

        print("Gripper Removed Coordinates:" + self.current_setpoint.__str__())

        # Shift vector to account for elbow1 displacement from center
        # Need to think about when to solve for base angle (before or after shift)
        self.current_setpoint.x = self.current_setpoint.x + LeArmConstants.X_SHIFT

        self.check_x_z_coordinate()
        print((square(self.get_x_z_length()) - square(LeArmConstants.LINK2_LENGTH) - square(
            LeArmConstants.LINK3_LENGTH)))

        theta_3 = -(m.acos((square(self.get_x_z_length()) - square(LeArmConstants.LINK2_LENGTH) - square(
            LeArmConstants.LINK3_LENGTH)) /
                           (2 * LeArmConstants.LINK2_LENGTH * LeArmConstants.LINK3_LENGTH)))
        theta_3_N = -theta_3

        beta = m.atan2(self.current_setpoint.z, self.current_setpoint.x)
        psi = m.acos((square(self.get_x_z_length()) + square(LeArmConstants.LINK2_LENGTH) - square(
            LeArmConstants.LINK3_LENGTH)) /
                     (2 * LeArmConstants.LINK2_LENGTH * self.get_x_z_length()))

        theta_2 = beta + psi
        theta_2_N = beta - psi

        theta_4 = self.current_setpoint.pitch - theta_2 - theta_3
        theta_4_N = self.current_setpoint.pitch - theta_2_N - theta_3_N

        print(f"psi: {psi}\n"
              f"beta: {beta}\n"
              f"length xz: {self.get_x_z_length()}\n"
              f"square L2: {square(LeArmConstants.LINK2_LENGTH)}\n"
              f"square L3: {square(LeArmConstants.LINK3_LENGTH)}\n"
              f"Denominator: {2 * LeArmConstants.LINK2_LENGTH * self.get_x_z_length()}")

        print(f"theta2: {theta_2}\n "
              f"theta3: {theta_3}\n"
              f"theta4: {theta_4}\n"
              f"theta2n: {theta_2_N}\n"
              f"theta3n: {theta_3_N}\n"
              f"theta4n: {theta_4_N}")

        return self.compare([None, theta_2, theta_3, theta_4, None, None],
                            [None, theta_2_N, theta_3_N, theta_4_N, None, None])

    def solve_for_base(self):
        temp = m.acos(self.current_setpoint.x / get_2D_vector_length(self.current_setpoint.x, self.current_setpoint.y))
        if temp > 90:
            temp -= 180
        elif temp < -90:
            temp += 180
        return temp
