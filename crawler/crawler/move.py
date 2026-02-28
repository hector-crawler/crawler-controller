from enum import Enum


class Move(Enum):
    ARM_UP = 0
    ARM_DOWN = 1
    HAND_UP = 2
    HAND_DOWN = 3


class MoveMode(Enum):
    USER_WAIT = 0
    USER_ARM_UP = 1
    USER_ARM_DOWN = 2
    USER_HAND_UP = 3
    USER_HAND_DOWN = 4
    USER_STEP = 5
    USER_STEP_EXPLORATION = 6
    USER_STEP_EXPLOITATION = 7
    AUTOMATIC = 8


MOVES_COUNT = len(Move)
