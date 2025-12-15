"""
Module for all the definitions

@author: ldevillez
"""

# Torque constants
TORQUE_CONST_OLD = 0.1133  # Nm/A
TORQUE_CONST_NEW = 0.1450  # Nm/A

from enum import Enum

class TYPE_JOINT(Enum):
    """
    Enumeration of joint types
    """
    ANKLE = 0
    KNEE = 1

    def __str__(self):
        return self.name.lower()

class TYPE_CONTROLLER(Enum):
    """
    Enumeration of controller types
    """
    POSITION = 0
    IMPEDANCE = 1

    def __str__(self):
        return self.name.lower()
