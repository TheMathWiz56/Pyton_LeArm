import numpy as np
import math as m
from enum import Enum

"""
orientations (found in vertical position)
Base = negative is CW from top down
Elbow1 = negative is in -x direction
Elbow2 = negative is in +x direction
Elbow3 = negative is in +x direction
Wrist = negative is CW from top down
Gripper = positive is closing, negative is opening
"""


def square(num):
    return num * num


class LeArmConstants:
    class GripperState(Enum):
        OPEN = 0
        MIDDLE = 45
        CLOSED = 80

    # Enumerator for keeping track of Link Types
    class LinkType(Enum):
        REVOLUTE_LINK = 0
        PRISMATIC_LINK = 1

    class PINS(Enum):
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
        [m.pi / 2, 10, 0, 0, LinkType.REVOLUTE_LINK],
        [0, LINK2_LENGTH, 0, 0, LinkType.REVOLUTE_LINK],
        [0, LINK3_LENGTH, 0, 0, LinkType.REVOLUTE_LINK],
        [-m.pi / 2, 0, 0, 0, LinkType.REVOLUTE_LINK]
    ]

    # Vertical positions
    SHOULDER_VERTICAL = 124
    ELBOW1_VERTICAL = 115
    ELBOW2_VERTICAL = 105
    ELBOW3_VERTICAL = 125
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


