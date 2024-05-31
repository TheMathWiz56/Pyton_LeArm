import numpy as np
import math as m
from Constants import *
from adafruit_servokit import ServoKit
from ArmSetpoint import ArmSetpoint
from Link import LinkList

np.set_printoptions(precision=5, suppress=True, )


# ____________________________________________________________________________________________________________________
# Functions
def check_angle_achievable(angle):
    if angle < -m.pi / 2:
        return False
    elif angle > m.pi / 2:
        return False
    return True


def is_valid_x_z_coordinate(x, z):
    print(f"Desired x {x}\nDesired z {z}")
    xz_length = get_2D_vector_length(x, z)

    if xz_length > LeArmConstants.MAX_EXTENSION or xz_length < LeArmConstants.MIN_EXTENSION:
        return False
    return True


def check_angle_achievable_elbow1(angle):
    if angle < 0:
        return False
    elif angle > m.pi:
        return False
    return True


def check_servo_setpoint_list_achievable(setpoint_list):
    achievable = True
    for i in range(len(setpoint_list)):
        if achievable:
            if i == 0:
                achievable = check_angle_achievable_elbow1(setpoint_list[i])
            else:
                achievable = check_angle_achievable(setpoint_list[i])
        else:
            return achievable
    return achievable


def get_travel(solution, past_setpoint):
    """
    Computes the travel for 3-axis planar angles due to the servos' limited range of motion
    @return:
    """
    travel = 0
    travel += m.fabs(solution[0] - past_setpoint[0])
    travel += m.fabs(solution[1] - past_setpoint[1])
    travel += m.fabs(solution[2] - past_setpoint[2])
    return travel


def compare(sol1, sol2, past_setpoint):
    """
    Check if solutions are achievable
    if both are, compares and returns the shorter path

    Should also do something if both not achievable
    @param sol1:
    @param sol2:
    @param past_setpoint
    @return:
    """
    sol1_achievable = check_servo_setpoint_list_achievable(sol1)
    sol2_achievable = check_servo_setpoint_list_achievable(sol2)

    # print(f"""sol1: {sol1}
    # sol1_achievable: {sol1_achievable}
    # sol2: {sol2}
    # sol2_achievable: {sol2_achievable}""")

    if sol1_achievable and sol2_achievable:
        travel1 = get_travel(sol1, past_setpoint)
        travel2 = get_travel(sol2, past_setpoint)

        if travel1 > travel2:
            return sol2
        return sol1
    elif sol1_achievable:
        return sol1
    elif sol2_achievable:
        return sol2

    # Catch all for now
    return [None, None, None]


def solve_for_base(x, y):
    angle = 0
    if x == 0 and y != 0:
        angle = m.pi / 2
    elif x != 0 and y == 0:
        angle = 0
    elif x != 0 and y != 0:
        angle = m.atan(y / x)

    return angle + m.pi / 2


def clamp_wrist_angle(roll):
    return clamp(roll, 0, m.pi)


def solve_3_axis_planar(x, z, pitch, past_setpoint):
    """First remove the gripper vector from the arm position vector
It's important that theta6 is updated to its new desired value before it can be removed from the total vector
   in order to ensure the proper amount has been removed
Need to add shift for elbow1 being off center
"""
    xz_length = get_2D_vector_length(x, z)

    theta_3 = -(m.acos((square(xz_length) - square(LeArmConstants.LINK2_LENGTH) - square(
        LeArmConstants.LINK3_LENGTH)) /
                       (2 * LeArmConstants.LINK2_LENGTH * LeArmConstants.LINK3_LENGTH)))
    theta_3_N = -theta_3

    beta = m.atan2(z, x)
    psi = m.acos((square(xz_length) + square(LeArmConstants.LINK2_LENGTH) - square(
        LeArmConstants.LINK3_LENGTH)) /
                 (2 * LeArmConstants.LINK2_LENGTH * xz_length))

    theta_2 = beta + psi
    theta_2_N = beta - psi

    theta_4 = pitch - theta_2 - theta_3
    theta_4_N = pitch - theta_2_N - theta_3_N

    """print(f"psi: {psi}\n"
              f"beta: {beta}\n"
              f"length xz: {self.get_tempx_z_length()}\n"
              f"square L2: {square(LeArmConstants.LINK2_LENGTH)}\n"
              f"square L3: {square(LeArmConstants.LINK3_LENGTH)}\n"
              f"Denominator: {2 * LeArmConstants.LINK2_LENGTH * self.get_tempx_z_length()}")

    print(f"theta2: {theta_2}\n"
        f"theta3: {theta_3}\n"
              f"theta4: {theta_4}\n"
              f"theta2n: {theta_2_N}\n"
              f"theta3n: {theta_3_N}\n"
              f"theta4n: {theta_4_N}")"""

    return compare([theta_2, theta_3, theta_4], [theta_2_N, theta_3_N, theta_4_N], past_setpoint)


def get_forward_kinematics(link_list: LinkList):
    base_to_wrist_frame_transformation = np.identity(4)

    for link in link_list.get_list_reversed():
        base_to_wrist_frame_transformation = np.matmul(link.get_homogeneous_transform(),
                                                       base_to_wrist_frame_transformation)
    return base_to_wrist_frame_transformation


def get_unit_vector(v):
    return [v[0] / get_2D_vector_length(v[0], v[1]), v[1] / get_2D_vector_length(v[0], v[1])]


# ____________________________________________________________________________________________________________________

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

    def go_to(self, gripper_setpoint, command_type=0, x=0, y=0, z=0, pitch=m.pi/2, roll=m.pi/2):
        """
        :param gripper_setpoint:
        :param command_type
        :param x: mm
        :param y: mm
        :param z: mm
        :param pitch: RADIANS
        :param roll: RADIANS
        :return:
        """

        self.kinematics.solve(x, y, z, pitch, roll, gripper_setpoint, command_type)

        servo_outputs = self.current_setpoint.get_servo_setpoint_list()
        theta_list = self.current_setpoint.get_raw_theta_list_radians()
        # print("Inverse Kinematics solved for: ")
        # print(self.current_setpoint.get_raw_theta_list_radians())
        # print(servo_outputs)

        # Doesn't include gripper updates
        self.update_servos_setpoints_raw(servo_outputs)
        self.link_list.update_joint_revolute_variables(theta_list)
        self.update_base_to_wrist_frame_transformation()

    def __str__(self):
        return self.base_to_wrist_frame_transformation


class ArmKinematics:
    def __init__(self, current_setpoint: ArmSetpoint, past_setpoint: ArmSetpoint):
        # Should be set to default position, i.e. vertical
        self.current_setpoint = current_setpoint
        self.past_setpoint = past_setpoint
        self.temp_X = 0

    # Careful to only use after the gripper vector has been removed from the arm setpoint
    def get_tempx_z_length(self):
        return get_2D_vector_length(self.temp_X, self.current_setpoint.z)

    def clamp_tempx_z_vector(self):
        if self.get_tempx_z_length() > LeArmConstants.MAX_EXTENSION:
            scaler = LeArmConstants.MAX_EXTENSION / self.get_tempx_z_length()
        elif self.get_tempx_z_length() < LeArmConstants.MIN_EXTENSION:
            scaler = LeArmConstants.MIN_EXTENSION / self.get_tempx_z_length()
        else:
            scaler = 1

        print(f"scaler : {scaler}, length : {self.get_tempx_z_length()}")
        self.temp_X = scaler * self.temp_X
        self.current_setpoint.z = scaler * self.current_setpoint.z

    def move_current_to_past_setpoint(self):
        self.past_setpoint.update_setpoints(self.current_setpoint.get_setpoint_as_list())

    def move_past_to_current_setpoint(self):
        self.current_setpoint.update_setpoints(self.past_setpoint.get_setpoint_as_list())

    def solve(self, x, y, z, pitch, roll, gripper_setpoint, command_type):
        """
        Takes in the angles in radians and returns the angles in degrees for the servos and raw angles in radians for
        storage

        :param x:
        :param y:
        :param z:
        :param pitch:
        :param roll:
        :param gripper_setpoint:
        :param command_type:
        :return:
        """
        self.move_current_to_past_setpoint()

        # All values will either be updated or kept the same
        # If they are kept the same, they may be the default 0 or the past value that wasn't updated
        new_point = self.check_new_point(x, y, z, pitch, roll)

        if new_point:
            # Base Rotation
            self.update_tempX()

            self.current_setpoint.theta1 = solve_for_base(x, y)
            self.current_setpoint.theta5 = clamp_wrist_angle(self.current_setpoint.roll)
            self.current_setpoint.theta6 = m.radians(gripper_setpoint)

            x3, z3 = self.get_coordinates_for_3_axis()

            print(f"""
            Tempx : {x3}
            z : {z3}
            pitch : {self.current_setpoint.pitch}
            command type: {command_type}""")

            if command_type == LeArmConstants.CommandType.FIXED.value:
                if not is_valid_x_z_coordinate(x3, z3):
                    print("OUTSIDE REACHABLE RANGE")
                    self.move_past_to_current_setpoint()
                else:
                    self.check_update_current_setpoint_angles(solve_3_axis_planar(x3, z3,
                                                                                  self.current_setpoint.pitch,
                                                                                  self.past_setpoint.get_3_axis_list()))
            elif command_type == LeArmConstants.CommandType.ADJUSTABLE_PITCH.value:
                pass
            elif command_type == LeArmConstants.CommandType.ADJUSTABLE_POINT.value:
                solution = solve_3_axis_planar(x3, z3, self.current_setpoint.pitch,
                                               self.past_setpoint.get_3_axis_list())
                if solution[0] is None:
                    if self.get_tempx_z_length() > LeArmConstants.MAX_EXTENSION:
                        self.solve_adjustable_point_vector(LeArmConstants.MAX_EXTENSION)
                    elif self.get_tempx_z_length() < LeArmConstants.MIN_EXTENSION:
                        self.solve_adjustable_point_vector(LeArmConstants.MIN_EXTENSION)
                    else:
                        print("NO SOLUTION")
                        self.move_past_to_current_setpoint()
                    self.check_update_current_setpoint_angles(solve_3_axis_planar(x3, z3,
                                                                                  self.current_setpoint.pitch,
                                                                                  self.past_setpoint.get_3_axis_list()))

    def update_tempX(self):
        self.temp_X = -get_2D_vector_length(self.current_setpoint.x, self.current_setpoint.y) + LeArmConstants.X_SHIFT
        if self.current_setpoint.x < 0:
            self.temp_X = -self.temp_X
        elif self.current_setpoint.x == 0 and self.current_setpoint.y < 0:
            self.temp_X = -self.temp_X

    def check_new_point(self, x=None, y=None, z=None, pitch=None, roll=None):
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
        return new_point

    def check_update_current_setpoint_angles(self, planar_3_axis_solution):
        if planar_3_axis_solution[0] is not None:
            self.current_setpoint.theta2 = planar_3_axis_solution[0]
        else:
            self.move_past_to_current_setpoint()
            print("POINT NOT REACHABLE")
        if planar_3_axis_solution[1] is not None:
            self.current_setpoint.theta3 = planar_3_axis_solution[1]
        else:
            self.move_past_to_current_setpoint()
            print("POINT NOT REACHABLE")
        if planar_3_axis_solution[2] is not None:
            self.current_setpoint.theta4 = planar_3_axis_solution[2]
        else:
            self.move_past_to_current_setpoint()
            print("POINT NOT REACHABLE")

    def get_coordinates_for_3_axis(self):
        gripper_length = (m.sin(self.current_setpoint.theta6) * LeArmConstants.GRIPPER_EVEN_BAR_LINK_LENGTH +
                          LeArmConstants.WRIST_TO_GRIPPER_DISTANCE)

        gripper_v_x = m.cos(self.current_setpoint.pitch) * gripper_length
        gripper_v_z = m.sin(self.current_setpoint.pitch) * gripper_length
        # print("Gripper Vector Length:" + gripper_length.__str__())
        return [self.temp_X - gripper_v_x, self.current_setpoint.z - gripper_v_z]

    def get_removed_gripper_coordinates(self, x, z):
        gripper_length = (m.sin(self.current_setpoint.theta6) * LeArmConstants.GRIPPER_EVEN_BAR_LINK_LENGTH +
                          LeArmConstants.WRIST_TO_GRIPPER_DISTANCE)
        gripper_v_x = m.cos(self.current_setpoint.pitch) * gripper_length
        gripper_v_z = m.sin(self.current_setpoint.pitch) * gripper_length
        return [x - gripper_v_x, z - gripper_v_z]

    def get_added_gripper_coordinates(self, x, z):
        gripper_length = (m.sin(self.current_setpoint.theta6) * LeArmConstants.GRIPPER_EVEN_BAR_LINK_LENGTH +
                          LeArmConstants.WRIST_TO_GRIPPER_DISTANCE)
        gripper_v_x = m.cos(self.current_setpoint.pitch) * gripper_length
        gripper_v_z = m.sin(self.current_setpoint.pitch) * gripper_length
        return [x + gripper_v_x, z + gripper_v_z]

    def solve_adjustable_point_vector(self, scaler):
        """
        Removes the gripper component from the <temp_X, z> vector
        Scales to scaler extension
        Re-adds gripper vector
        Updates temp_X and z to new values
        @param scaler
        """
        [x, z] = get_unit_vector(self.get_removed_gripper_coordinates(self.temp_X, self.current_setpoint.z))
        [self.temp_X, self.current_setpoint.z] = self.get_added_gripper_coordinates(x * scaler, z * scaler)
        self.update_xy_from_temp_X()

    def update_xy_from_temp_X(self):
        theta = m.atan2(self.current_setpoint.y, self.current_setpoint.x)
        self.current_setpoint.y = (self.temp_X - LeArmConstants.X_SHIFT) * m.sin(theta)
        self.current_setpoint.x = (self.temp_X - LeArmConstants.X_SHIFT) * m.cos(theta)
