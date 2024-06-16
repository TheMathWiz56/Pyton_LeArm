"""
Author - Joseph Amador
All Constants for LeArm robotic arm project
"""
import math as m
from enum import Enum


def square(num):
    """
    Returns the square of the input
    :param num:
    :return:
    """
    return num * num


def get_2D_vector_length(x, y):
    """
    :param x
    :param y
    :return: the length of the 2D vector with components <x,y>
    """
    return m.sqrt(x * x + y * y)


def get_3D_vector_length(v):
    """
    :param v
    :return: the length of the 2D vector with components <x,y>
    """
    return m.sqrt(square(v[0]) + square(v[1]) + square(v[2]))


def clamp(n, minn, maxn):
    if n > maxn:
        return maxn
    if n < minn:
        return minn
    return n


def scale_to_range_from_0(n, maxn):
    return n % maxn


def get_gripper_state_from_angle_rad(angle):
    floating_point_error = .1
    angle = m.degrees(angle)
    index = 0
    for gripper_position in LeArmConstants.gripper_positions:
        if gripper_position - floating_point_error < angle < gripper_position + floating_point_error:
            return index
        index = index + 1

    return -1


class LeArmConstants:
    """
    All Constants for LeArm robotic arm project
    """

    gripper_positions = [0, 45, 105]

    # Enumerator for keeping track of Link Types
    class LinkType(Enum):
        """
        Enumerator for keeping track of link types
        """
        REVOLUTE_LINK = 0
        PRISMATIC_LINK = 1

    class CommandType(Enum):
        """
        Enumerator for specifying command type
        """
        FIXED = 0
        ADJUSTABLE_PITCH = 1
        ADJUSTABLE_POINT = 2
        STEPPED = 3
        TRAVEL_AT_HEIGHT = 4

    class PINS(Enum):
        """"
        Enumerator for keeping track of servo pins
        """
        SHOULDER = 0  # white
        ELBOW1 = 1  # green
        ELBOW2 = 2  # yellow
        ELBOW3 = 3  # orange
        WRIST = 4  # purple
        GRIPPER = 5  # grey

    LINK2_LENGTH = 104
    LINK3_LENGTH = 89

    LINK_PARAMETERS = [
        [0, 0, 0, 0, LinkType.REVOLUTE_LINK],
        [-m.pi / 2, 10, 0, 0, LinkType.REVOLUTE_LINK],
        [0, LINK2_LENGTH, 0, 0, LinkType.REVOLUTE_LINK],
        [0, LINK3_LENGTH, 0, 0, LinkType.REVOLUTE_LINK],
        [-m.pi / 2, 0, 0, 0, LinkType.REVOLUTE_LINK]
        # Really has another prismatic link but that is handled separately
    ]

    X_SHIFT = LINK_PARAMETERS[1][1]

    SERVO_SETTINGS_LIST = [[PINS.SHOULDER.value, 450, 2700],
                           [PINS.ELBOW1.value, 625, 2750],
                           [PINS.ELBOW2.value, 550, 2675],
                           [PINS.ELBOW3.value, 725, 2800],
                           [PINS.WRIST.value, 500, 2550]]

    """
    orientations (found in vertical position)
    Base = negative is CW from top down
    Elbow1 = negative is in -x direction 7 - 
    Elbow2 = negative is in +x direction
    Elbow3 = positive is in +x direction
    Wrist = negative is CW from top down
    Gripper = positive is closing, negative is opening
    """

    # Vertical positions
    SHOULDER_VERTICAL = 90
    ELBOW1_VERTICAL = 90
    ELBOW2_VERTICAL = 90
    ELBOW3_VERTICAL = 90
    WRIST_VERTICAL = 90
    VERTICAL_POSITIONS_LIST = [SHOULDER_VERTICAL, ELBOW1_VERTICAL, ELBOW2_VERTICAL, ELBOW3_VERTICAL, WRIST_VERTICAL,
                               gripper_positions[1]]

    # Stow positions
    SHOULDER_STOW = 30
    ELBOW1_STOW = 30
    ELBOW2_STOW = 20
    ELBOW3_STOW = 30
    WRIST_STOW = 110
    STOW_POSITIONS_LIST = [SHOULDER_STOW, ELBOW1_STOW, ELBOW2_STOW, ELBOW3_STOW, WRIST_STOW,
                           gripper_positions[0]]

    WRIST_TO_GRIPPER_DISTANCE = 95 + 20.075 + 27.53
    GRIPPER_EVEN_BAR_LINK_LENGTH = 30

    # Extension (with gripper vector removed)
    MAX_EXTENSION = LINK2_LENGTH + LINK3_LENGTH
    MIN_EXTENSION = get_2D_vector_length(LINK2_LENGTH, LINK3_LENGTH)

    CRUISING_HEIGHT = 310
