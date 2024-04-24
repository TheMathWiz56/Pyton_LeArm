import numpy as np
import math as m
from enum import Enum


# Enumerator for keeping track of Link Types
class LinkType(Enum):
    REVOLUTE_LINK = 0
    PRISMATIC_LINK = 1


class LeArmConstants:
    LINK2_LENGTH = 104
    LINK3_LENGTH = 89

    LINK_PARAMETERS = np.array([
        [0, 0, 0, 0, LinkType.REVOLUTE_LINK],
        [m.pi / 2, 10, 0, m.pi/2, LinkType.REVOLUTE_LINK],
        [0, LINK2_LENGTH, 0, 0, LinkType.REVOLUTE_LINK],
        [0, LINK3_LENGTH, 0, 0, LinkType.REVOLUTE_LINK],
        [-m.pi / 2, 0, 0, 0, LinkType.REVOLUTE_LINK]
    ])

    SHOULDER_OFFSET = 0
    ELBOW1_OFFSET = 0
    ELBOW2_OFFSET = 0
    ELBOW3_OFFSET = 0
    WRIST_OFFSET = 0
