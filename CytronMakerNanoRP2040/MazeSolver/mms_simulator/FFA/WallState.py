from enum import Enum

class WallState(Enum):
    EXIT = 0,     # a wall that has been seen and confirmed absent
    WALL = 1,     # a wall that has been seen and confirmed present
    UNKNOWN = 2,  # a wall that has not yet been seen
    VIRTUAL = 3,  # a wall that has not yet been seen