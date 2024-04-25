import numpy as np
import math as m
from enum import Enum

"""
orientations (found in vertical position)
Base - CW from top down
Elbow1
Elbow2
Elbow3
Wrist
Gripper
"""


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


def square(num):
    return num * num


class GripperState(Enum):
    OPEN = 0
    MIDDLE = 90
    CLOSED = 110


class LeArmConstants:
    LINK2_LENGTH = 104
    LINK3_LENGTH = 89

    LINK_PARAMETERS = np.array([
        [0, 0, 0, 0, LinkType.REVOLUTE_LINK],
        [m.pi / 2, 10, 0, m.pi / 2, LinkType.REVOLUTE_LINK],
        [0, LINK2_LENGTH, 0, 0, LinkType.REVOLUTE_LINK],
        [0, LINK3_LENGTH, 0, 0, LinkType.REVOLUTE_LINK],
        [-m.pi / 2, 0, 0, 0, LinkType.REVOLUTE_LINK]
    ])

    # Vertical positions
    SHOULDER_OFFSET = 124
    ELBOW1_OFFSET = 110
    ELBOW2_OFFSET = 110
    ELBOW3_OFFSET = 125
    WRIST_OFFSET = 110
