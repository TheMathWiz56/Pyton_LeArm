import math as m
from Constants import *


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
        return get_2D_vector_length(self.x, self.z)

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
        """
        return [m.degrees(self.theta1),
                m.degrees(self.theta2), LeArmConstants.ELBOW2_VERTICAL -
                m.degrees(self.theta3), LeArmConstants.ELBOW3_VERTICAL
                - m.degrees(self.theta4), m.degrees(self.theta5), m.degrees(self.theta6)]

    def __str__(self):
        return self.get_setpoint_as_list().__str__()
