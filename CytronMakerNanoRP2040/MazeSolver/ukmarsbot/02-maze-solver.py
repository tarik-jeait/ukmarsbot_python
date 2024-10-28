from machine import Pin, ADC, PWM
import neopixel
import time
import math

from _Sensors import Sensors
from sys import exit

from _CytronMakerNanoRP2040 import CytronMakerNanoRP2040
from _Config import Config
from _Motors import Motors
from _Encoders import Encoders

from Cell import Cell
from WallState import WallState

from Heading import Heading
from FloodFill import FloodFill
from Maze import Maze
from Logger import Logger
from Direction import Direction
from Logger import Logger
from StartCmd import StartCmd
from Tone import Tone
#############################################################################
# Global variable definitions
#############################################################################

global cytron_maker_rp2040_board, ukmarsbot_motors, ukmarsbot_encoders, ukmarsbot_sensors,ukmarsbot_logger

# Instance of the board
cytron_maker_rp2040_board = cytron_board = CytronMakerNanoRP2040()
# Instance of sensors
ukmarsbot_sensors = Sensors(cytron_board)
# Instance of encoders
ukmarsbot_encoders = Encoders(cytron_board)
# Instance of motors
ukmarsbot_motors = Motors(cytron_board,ukmarsbot_encoders,ukmarsbot_sensors)

ukmarsbot_logger = Logger()

ukmarsbot_logger.init()

ukmarsbot_startcmd = StartCmd(cytron_board,ukmarsbot_sensors)

global maze,flood_fill,maze_goal,maze_start

Config.MAZE_WIDTH = 3
Config.MAZE_HEIGHT = 3
#Config.MAZE_GOAL = Cell(3,2)
Config.MAZE_GOAL = Cell(1,1)
Config.MAZE_START = Cell(0,0)
Config.MAZE_CELL_COUNT = Config.MAZE_WIDTH * Config.MAZE_HEIGHT
Config.MAZE_MAX_COST = Config.MAZE_CELL_COUNT - 1

maze_goal = Config.MAZE_GOAL
maze_start = Config.MAZE_START

maze = Maze(maze_start,maze_goal)
flood_fill = FloodFill(maze,maze_start,maze_goal)



#############################################################################


  #  print (l1count, progno, bit0, bit1, bit2,)
def checkspeed():
    global leftspeed, rightspeed
    if leftspeed > 50000:
        leftspeed = 50000
    if rightspeed > 50000:
        rightspeed = 50000
    if leftspeed < 10:
        leftspeed = 10
    if rightspeed < 10:
        rightspeed = 10
    return

def play_tone():
    # Play tone
    PIEZO_PWM = machine.PWM(machine.Pin(22))
    PIEZO_PWM.freq(440)
    PIEZO_PWM.duty_u16(30000) # in range 0 to 65535
    time.sleep(0.5)
    PIEZO_PWM.duty_u16(0)

def play_tone_calibrate():
    # Play tone
    PIEZO_PWM = machine.PWM(machine.Pin(22))
    # Melody
    MELODY_NOTE = [Tone.NOTE_C4, Tone.NOTE_G3, Tone.NOTE_G3, Tone.NOTE_A3, Tone.NOTE_G3, 0, Tone.NOTE_B3, Tone.NOTE_C4]
    MELODY_DURATION = [0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.2]
    for i in range(7):
        print(i)
        if(MELODY_NOTE[i] != 0):
            PIEZO_PWM.freq(MELODY_NOTE[i])
            PIEZO_PWM.duty_u16(30000) # in range 0 to 65535
        time.sleep(MELODY_DURATION[i])
        PIEZO_PWM.duty_u16(0)

def play_tone_start():
    PIEZO_PWM = machine.PWM(machine.Pin(22))
    # Melody
    MELODY_NOTE = [659, 659, 0, 659, 0, 523, 659, 0, 784]
    MELODY_DURATION = [0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.2]
    for i in range(9):
        print(i)
        if(MELODY_NOTE[i] != 0):
            PIEZO_PWM.freq(MELODY_NOTE[i])
            PIEZO_PWM.duty_u16(30000) # in range 0 to 65535
        time.sleep(MELODY_DURATION[i])
        PIEZO_PWM.duty_u16(0)


def resolveWallMaze(board):
    Logger.log("Running...")
    flood_fill.fill_goal(first_pass=True)
    current_cell = maze.cells[maze_start.x][maze_start.y]
    current_heading = Heading.NORTH
    Logger.log("current cell:[%d][%d]"%(current_cell.x,current_cell.y))
    goal_reached = False
    start_filled = False
    start_reached = False
    second_pass = False
    while True:
        Logger.log("################### LOOP #########################")
        #READ WALL INFO
        sensors = ukmarsbot_sensors.readSensorsNormalized()
        sensors.printSensorsNormalized()
        Logger.log(">>current cell:[%d][%d]-visited:%d-cost:%d-WF:%d-WL:%d-WR:%d"%(current_cell.x,current_cell.y,current_cell.visited,current_cell.cost,sensors.isWallFront(),sensors.isWallLeft(),sensors.isWallRight()))
        Logger.log(">>current cell:[%d][%d] / cost:[%d]"%(current_cell.x,current_cell.y,current_cell.cost))
        Logger.log(">>current heading:%s"%current_heading)
        ukmarsbot_motors.stopMotors()
        time.sleep(0.1)
        current_cell.update_wall_states(current_heading,sensors.isWallFront(),sensors.isWallLeft(),sensors.isWallRight())
        Logger.log(">> updated current cell:[%d][%d] / cost:%d / Walls:N[%s]-S[%s]-E[%s]-W[%s]"%(current_cell.x,current_cell.y,current_cell.cost,current_cell.walls["NORTH"],current_cell.walls["SOUTH"],current_cell.walls["EAST"],current_cell.walls["WEST"]))            
        maze.print()
        
        if not goal_reached:
            Logger.log(">>##Goal not reached")
            flood_fill.fill_goal(first_pass=False)
            
            if current_cell.x == maze_goal.x and current_cell.y == maze_goal.y and second_pass:
                Logger.log(">>Goal reached")
                exit()
            elif current_cell.x == maze_goal.x and current_cell.y == maze_goal.y and not start_filled:
                Logger.log(">>Goal reached")
                goal_reached=True
                #flood_fill.fill_goal(first_pass=False)
                maze.print()
                #exit()
                #flood_fill.fill_goal(first_pass=False)
                flood_fill.fill_start()
                start_filled = True
            next_move = maze.next_move(current_heading,current_cell,sensors.isWallFront(),sensors.isWallLeft(),sensors.isWallRight())
        else:
            Logger.log(">>##Start not reached")
            if current_cell.x == maze_start.x and current_cell.y == maze_start.y:
                Logger.log(">>##Start reached --> go to the goal")
                start_reached = True
                second_pass = True
                goal_reached = False
                flood_fill.fill_goal(first_pass=False)
                next_move = maze.next_move(current_heading,current_cell,sensors.isWallFront(),sensors.isWallLeft(),sensors.isWallRight())
            else:
                flood_fill.fill_start()
                #next_move = maze.get_next_move_return_back_to_start(current_heading,current_cell,API.wallFront(),API.wallLeft(),API.wallRight())
                next_move = maze.next_move(current_heading,current_cell,sensors.isWallFront(),sensors.isWallLeft(),sensors.isWallRight())
        
        if next_move == Direction.LEFT:
            ukmarsbot_motors.turnLeftAngle(75)
            current_heading =  Heading.update(current_heading,Direction.LEFT)
            ukmarsbot_motors.MoveForwardDistance(150)
            current_cell_coordinates = current_cell.neighbour(current_heading,Direction.FORWARD)
            current_cell = maze.cells[current_cell_coordinates.x][current_cell_coordinates.y]
        else:
            if next_move == Direction.RIGHT:
                ukmarsbot_motors.turnRightAngle(75)
                current_heading =  Heading.update(current_heading,Direction.RIGHT)
                ukmarsbot_motors.MoveForwardDistance(150)
                current_cell_coordinates = current_cell.neighbour(current_heading,Direction.FORWARD)
                current_cell = maze.cells[current_cell_coordinates.x][current_cell_coordinates.y]
            else:
                if next_move == Direction.FORWARD:
                    ukmarsbot_motors.MoveForwardDistance(150)
                    current_cell_coordinates = current_cell.neighbour(current_heading,Direction.FORWARD)
                    current_cell = maze.cells[current_cell_coordinates.x][current_cell_coordinates.y]
                else:
                    if next_move == Direction.BACK:
                        ukmarsbot_motors.turnRightAngle(75)
                        current_heading =  Heading.update(current_heading,Direction.RIGHT)
                        time.sleep(0.1)
                        ukmarsbot_motors.turnRightAngle(75)
                        current_heading =  Heading.update(current_heading,Direction.RIGHT)


#ENCODERS
def encoders_enable_interrupts(board):    
    board.ENCODER_LEFT_CLK.irq(trigger=machine.Pin.IRQ_FALLING | machine.Pin.IRQ_RISING, handler=left_input_change)
    board.ENCODER_RIGHT_CLK.irq(trigger=machine.Pin.IRQ_FALLING | machine.Pin.IRQ_RISING, handler=right_input_change)
    
def left_input_change(pin):
    ukmarsbot_encoders.left_input_change(pin)

def right_input_change(pin):
    ukmarsbot_encoders.right_input_change(pin)


encoders_enable_interrupts(cytron_board)
ukmarsbot_encoders.init()


# Play tone
play_tone()

time.sleep(2)

#ukmarsbot_sensors.testReadSensorsWallFollower()
ukmarsbot_sensors.testReadSensorsNormalized()


cytron_board.LEFT_LED.value(0) # switch off sensor1 LED on line folllower board
cytron_board.RIGHT_LED.value(0) # switch off sensor2 LED on line folllower board
cytron_board.LMOTOR_PWM.duty_u16(0) # in range 0 to 65535
cytron_board.RMOTOR_PWM.duty_u16(0)


cmd = ukmarsbot_startcmd.waitForUserStart()
print("cmd:%s"%(cmd))

if cmd == "Calibrate":
    play_tone_calibrate()
    time.sleep(1)
    ukmarsbot_sensors.calibrateSensorsWallFollower()
elif cmd == "Start":
    play_tone_start()
    time.sleep(1)
    resolveWallMaze(cytron_board)
else:
    exit()


# wait for onboard button to be pressed
card_button = 1
# value for switch button
car_switch = 0


# First Loop for calibration
Logger.log("Calibration....")
button_pressed = 0
calibration_pass = 1
while (button_pressed == 0 and calibration_pass==1): # Check button 1 (GP20)
    card_button = cytron_board.btn1.value()
    if(card_button == 0):
        button_pressed = 1
        Logger.log("button_pressed:%d"%button_pressed)
        Logger.log("card_button:%d"%card_button)
        # Calibrate
        time.sleep(2)
        ukmarsbot_sensors.calibrateSensorsWallFollower()
        #ukmarsbot_sensors.testReadSensorsNormalized()

        #button_pressed = 1
    time.sleep(0.1)

# Play tone
play_tone()

#exit()
Logger.log("Maze Solving....")

# SECOND LOOP FOR MAZE SOLVER
button_pressed = 0
while (button_pressed == 0): # Check button 1 (GP20)
    card_button = cytron_board.btn1.value()
    if(card_button == 0):
        button_pressed = 1
        Logger.log("button_pressed:%d"%button_pressed)
        Logger.log("car_switch:%d"%car_switch)
        time.sleep(2)
        #ukmarsbot_motors.MoveForwardDistance(180)
        resolveWallMaze(cytron_board)
        #resolveMaze(cytron_board)
    time.sleep(0.1)

exit()

time.sleep(2)

# Play tone
play_tone()

print("Follow Sortest Path....")

# THIRD LOOP FOR MAZE SOLVER
button_pressed = 0
while (button_pressed == 0): # Check button 1 (GP20)
    card_button = cytron_board.btn1.value()
    if(card_button == 0):
        button_pressed = 1
        print("button_pressed:%d"%button_pressed)
        print("car_switch:%d"%car_switch)
        time.sleep(2)
        followShortestPath(cytron_board)
        button_pressed = 0
    time.sleep(0.1)


time.sleep(2)


exit()

#widelinefollow()
#BasicLineFollower(cytron_board)
#turnBackRight(cytron_board)

testReadSensorsNormalized(cytron_board)

#calibrateSensors(cytron_board)
#time.sleep(10)
#resolveMaze(cytron_board)
#turnLeft(cytron_board)

cytron_board.LEFT_LED.value(0) # switch off sensor1 LED on line folllower board
cytron_board.RIGHT_LED.value(0) # switch off sensor2 LED on line folllower board
cytron_board.LMOTOR_PWM.duty_u16(0) # in range 0 to 65535
cytron_board.RMOTOR_PWM.duty_u16(0)


