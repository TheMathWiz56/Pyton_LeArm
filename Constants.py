import numpy as np
import math as m
from enum import Enum


# Enumerator for keeping track of Link Types
class LinkType(Enum):
    REVOLUTE_LINK = 0
    PRISMATIC_LINK = 1


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

    # Servo Pins
    PINS = [0, 1, 5, 2, 4, 3]
    # Shoulder, Elbow1, Elbow2, Elbow3, Wrist, Gripper

    # Vertical positions
    SHOULDER_OFFSET = 124
    ELBOW1_OFFSET = 110
    ELBOW2_OFFSET = 110
    ELBOW3_OFFSET = 125
    WRIST_OFFSET = 110
