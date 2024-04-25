import numpy as np
import math as m
from enum import Enum


# Enumerator for keeping track of Link Types
class LinkType(Enum):
    REVOLUTE_LINK = 0
    PRISMATIC_LINK = 1


class PINS(Enum):
    SHOULDER = 0
    ELBOW1 = 1
    ELBOW2 = 5
    ELBOW3 = 2
    WRIST = 4
    GRIPPER = 3


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
