import numpy as np
import math as m
from enum import Enum


# Enumerator for keeping track of Link Types
class LinkType(Enum):
    REVOLUTE_LINK = 0
    PRISMATIC_LINK = 1


class LeArmConstants:
    LINK_PARAMETERS = np.array([
        [0, 0, 0, 0, LinkType.REVOLUTE_LINK],
        [m.pi / 2, 10, 0, 0, LinkType.REVOLUTE_LINK],
        [0, 104, 0, 0, LinkType.REVOLUTE_LINK],
        [0, 89, 0, 0, LinkType.REVOLUTE_LINK],
        [-m.pi / 2, 0, 0, 0, LinkType.REVOLUTE_LINK]
    ])

    SHOULDER_OFFSET = 0
    ELBOW1_OFFSET = 0
    ELBOW2_OFFSET = 0
    ELBOW3_OFFSET = 0
    WRIST_OFFSET = 0
