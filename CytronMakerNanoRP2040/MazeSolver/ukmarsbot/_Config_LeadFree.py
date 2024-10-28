import math

class Config:
    def __init__(self,board):
        self.board = board
    
    MOTOR_R_DIRECTION = 0
    MOTOR_L_DIRECTION = 0
    BASE_SPEED = 8000        
    # POLOLU Motors
    COUNTS_PER_ROTATION = 12
    #GEAR_RATIO = 51.45
    GEAR_RATIO = 50
    WHEEL_DIAMETER = 33.0
    MM_PER_COUNT = (math.pi * WHEEL_DIAMETER) / (2 * COUNTS_PER_ROTATION * GEAR_RATIO);        
    # CHINESE Motors
    MM_PER_COUNT = 0.08        
    #Specific to every ukmarsbot
    WHEEL_SEPARATION = 93

    DEG_PER_COUNT = (360.0 * MM_PER_COUNT) / (math.pi * WHEEL_SEPARATION);

    ENCODER_LEFT_POLARITY = 1
    ENCODER_RIGHT_POLARITY = 1

    MAZE_WIDTH = 0
    MAZE_HEIGHT = 3
    MAZE_GOAL = None
    MAZE_START = None
    MAZE_CELL_COUNT = 0
    MAZE_MAX_COST = 0

    WALL_SENSOR_LEFT_THRESHOLD = 10
    WALL_SENSOR_RIGHT_THRESHOLD = 10
    WALL_SENSOR_FRONT_THRESHOLD = 20
    