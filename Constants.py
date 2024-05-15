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
    return m.sqrt(x * x + y * y)


class LeArmConstants:
    """
    All Constants for LeArm robotic arm project
    """

    class GripperState(Enum):
        """"
        Enumerator for keeping track of gripper states
        """
        OPEN = 0
        MIDDLE = 45
        CLOSED = 80

    # Enumerator for keeping track of Link Types
    class LinkType(Enum):
        """
        Enumerator for keeping track of link types
        """
        REVOLUTE_LINK = 0
        PRISMATIC_LINK = 1

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
                               GripperState.MIDDLE.value]

    # Stow positions
    SHOULDER_STOW = 30
    ELBOW1_STOW = 30
    ELBOW2_STOW = 20
    ELBOW3_STOW = 30
    WRIST_STOW = 110
    STOW_POSITIONS_LIST = [SHOULDER_STOW, ELBOW1_STOW, ELBOW2_STOW, ELBOW3_STOW, WRIST_STOW,
                           GripperState.OPEN.value]

    WRIST_TO_GRIPPER_DISTANCE = 95 + 20.075 + 27.53
    GRIPPER_EVEN_BAR_LINK_LENGTH = 30

    # Extension (with gripper vector removed)
    MAX_EXTENSION = LINK2_LENGTH + LINK3_LENGTH
    MIN_EXTENSION = LINK2_LENGTH  # Just LINK2_LENGTH because of servo limitations
