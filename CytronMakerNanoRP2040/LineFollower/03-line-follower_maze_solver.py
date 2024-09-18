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

# Global variable definitions
global cytron_maker_rp2040_board, ukmarsbot_motors, ukmarsbot_encoders, ukmarsbot_sensors


# Instance of the board
cytron_maker_rp2040_board = cytron_board = CytronMakerNanoRP2040()
# Instance of encoders
ukmarsbot_encoders = Encoders(cytron_board)
# Instance of motors
ukmarsbot_motors = Motors(cytron_board,ukmarsbot_encoders)

# Instance of sensors
ukmarsbot_sensors = Sensors(cytron_board)




# LINE FOLLOWER

LINE_WIDTH = 19.0;  # ADJUST THIS so that CTE is roughly equal to the error in mm
LINE_DETECT_THRESHOLD = 30000.0;  # minimum value to register the line - ADJUST TO SUIT
FRONTLEFT_MARKER_THRESHOLD = 5000.0;  # minimum value to register the turn marker
FRONTRIGHT_MARKER_THRESHOLD = 5000.0; # minimum value to register the start marker
LEFT_MARKER_THRESHOLD = 15000.0;  # minimum value to register the turn marker
RIGHT_MARKER_THRESHOLD = 15000.0; # minimum value to register the start marker

gSensorSum=0
gSensorDifference=0
gSensorCTE=0
global updateTime,updateInterval
updateTime=0
updateInterval = 2  # in milliseconds



global MAZE_REGISTER
MAZE_REGISTER = ""

global MAZE_REGISTER_LIST
MAZE_REGISTER_LIST = []

global FINAL_REGISTER_PATH
FINAL_REGISTER_PATH = []
global SHORTEST_PATH
SHORTEST_PATH = ""

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
        
def resolveMaze(board):
    global MAZE_REGISTER
    # Move Forward
    ukmarsbot_motors.MoveForward(0)    
    crossroads = False
    while crossroads == False:
        sensors = ukmarsbot_sensors.readSensorsNormalized()
        print("L:%3.2f - LF:%3.2f - RF:%3.2f - R:%3.2f"% (ukmarsbot_sensors.leftside_normalized,ukmarsbot_sensors.leftfront_normalized,ukmarsbot_sensors.rightfront_normalized,ukmarsbot_sensors.rightside_normalized))
        if(sensors.isCrossRoads() == True):
            # move few mm
            #MoveForward(board,0)    
            time.sleep(0.1)
            print("CrossRoads...")
            ukmarsbot_motors.stopMotors()
            sensors = ukmarsbot_sensors.readSensorsNormalized()
            time.sleep(0.01)
            #
            if(sensors.isRouteLeft()):
                #MoveForward(board,0)
                #time.sleep(0.4)
                #stopMotors()
                ukmarsbot_motors.MoveForwardDistance(50)
                # Check Again if it is the end
                sensors_end = ukmarsbot_sensors.readSensorsNormalized()
                if(sensors_end.isTheEnd()):
                    ukmarsbot_motors.stopMotors()
                    crossroads = True
                else:
                    #turnLeft(board)
                    MAZE_REGISTER = MAZE_REGISTER + "L"
                    MAZE_REGISTER_LIST.append("L")
                    
                    ukmarsbot_motors.turnLeftAngle(80)
                    #time.sleep(0.4)
                    time.sleep(0.01)
                    ukmarsbot_motors.MoveForward(0)
            else:
                if(sensors.isRouteRight()):
                    #MoveForward(board,0)
                    #time.sleep(0.4)
                    #stopMotors()
                    ukmarsbot_motors.MoveForwardDistance(50)
                    # Check Again if it is the end
                    sensors_end = ukmarsbot_sensors.readSensorsNormalized()
                    if(sensors_end.isTheEnd()):
                        ukmarsbot_motors.stopMotors()
                        crossroads = True
                    else:
                        if(sensors_end.isRouteStraigth()):
                            MAZE_REGISTER = MAZE_REGISTER + "S"
                            MAZE_REGISTER_LIST.append("S")
                            ukmarsbot_motors.MoveForward(0)
                        else:
                            #turnRight(board)
                            MAZE_REGISTER = MAZE_REGISTER + "R"
                            MAZE_REGISTER_LIST.append("R")
                            
                            ukmarsbot_motors.turnRightAngle(80)
                            #time.sleep(0.4)
                            time.sleep(0.01)
                            ukmarsbot_motors.MoveForward(0)
                else:
                    ukmarsbot_motors.stopMotors()
                    crossroads = True
        else:
            if(sensors.isBlackSurface()):
                MAZE_REGISTER = MAZE_REGISTER + "B"
                MAZE_REGISTER_LIST.append("B")

                ukmarsbot_motors.turnBackLeft()
                #time.sleep(0.5)
                time.sleep(0.01)
                ukmarsbot_motors.MoveForward(0)
            else:
                adjustment = 0
                # compute adjustment
                gSensorDifference = sensors.rightfront - sensors.leftfront;
                if(gSensorDifference > 2000):
                    adjustment = 5000        
                if(gSensorDifference < -2000):
                    adjustment = -5000
                ukmarsbot_motors.MoveForward(adjustment)
        time.sleep(0.01)
    save_maze_register()
    calculate_shortest_path()
    save_final_maze_register()

def save_final_maze_register():
    global FINAL_REGISTER_PATH
    try:
        with open('final_maze_register.txt', 'w') as f:
            f.write(' '.join(FINAL_REGISTER_PATH))
            f.close()
    except:
        print("Could not save data.")


def followShortestPath(board):
    global FINAL_REGISTER_PATH
    # Move Forward
    ukmarsbot_motors.MoveForward(0)    
    crossroads = False
    CURRENT_DIRECTION = ""
    while crossroads == False:
        sensors = ukmarsbot_sensors.readSensorsNormalized()
        print("L:%3.2f - LF:%3.2f - RF:%3.2f - R:%3.2f"% (sensors.leftside_normalized,sensors.leftfront_normalized,sensors.rightfront_normalized,sensors.rightside_normalized))
        if(sensors.isCrossRoads() == True):            
            # move few mm
            #MoveForward(board,0)    
            time.sleep(0.1)
            print("CrossRoads...")
            ukmarsbot_motors.stopMotors()
            sensors = ukmarsbot_sensors.readSensorsNormalized()
            time.sleep(0.01)
            CURRENT_DIRECTION = FINAL_REGISTER_PATH.pop(0)
            if CURRENT_DIRECTION is None:
                break
            else:
                if CURRENT_DIRECTION == "S":
                    ukmarsbot_motors.MoveForwardDistance(20)
                    #time.sleep(0.5)

                else:
                    if CURRENT_DIRECTION == "L":
                        ukmarsbot_motors.MoveForwardDistance(50)
                        sensors_end = ukmarsbot_sensors.readSensorsNormalized()
                        if(sensors_end.isTheEnd()):
                            ukmarsbot_motors.stopMotors()
                            crossroads = True
                        else:
                            ukmarsbot_motors.turnLeftAngle(80)
                            #time.sleep(0.4)
                            time.sleep(0.01)
                            #ukmarsbot_motors.MoveForwardDistance(20)
                            #time.sleep(0.3)

                       
                    else:
                        if CURRENT_DIRECTION == "R":
                            ukmarsbot_motors.MoveForwardDistance(50)
                            # Check Again if it is the end
                            sensors_end = ukmarsbot_sensors.readSensorsNormalized()
                            if(sensors_end.isTheEnd()):
                                ukmarsbot_motors.stopMotors()
                                crossroads = True
                            else:
                                ukmarsbot_motors.turnRightAngle(80)
                                time.sleep(0.01)
                                #ukmarsbot_motors.MoveForwardDistance(20)
                                #time.sleep(0.3)
        else:
            if(sensors.isBlackSurface()):
                ukmarsbot_motors.stopMotors()
            else:
                adjustment = 0
                # compute adjustment
                gSensorDifference = sensors.rightfront - sensors.leftfront;
                if(gSensorDifference > 2000):
                    adjustment = 5000        
                if(gSensorDifference < -2000):
                    adjustment = -5000
                ukmarsbot_motors.MoveForward(adjustment)
        time.sleep(0.01)
    save_maze_register()


def save_maze_register():
    try:
        with open('maze_register.txt', 'w') as f:
            f.write(MAZE_REGISTER)
            f.close()
    except:
        print("Could not save data.")

def calculate_shortest_path():
    global MAZE_REGISTER_LIST,FINAL_REGISTER_PATH
    End_Of_List = False
    CURRENT_DIRECTION = ""
    CURRENT_PATH = []
    NEW_PATH = ""
    FINAL_REGISTER_PATH = []
    while End_Of_List != True:
        if len(MAZE_REGISTER_LIST) > 0 :
            CURRENT_DIRECTION = MAZE_REGISTER_LIST.pop(0)
            print("CURRENT_DIRECTION:%s"%CURRENT_DIRECTION)
            CURRENT_PATH.append(CURRENT_DIRECTION)
            print("CURRENT_PATH:%s"%CURRENT_PATH)
            if(len(CURRENT_PATH) == 3):
                NEW_PATH = optimize_path(CURRENT_PATH)
                if(NEW_PATH is None):
                    FINAL_REGISTER_PATH.append(CURRENT_PATH.pop(0))
                else:
                    FINAL_REGISTER_PATH.append(NEW_PATH)
                    CURRENT_PATH = []
        else:
            End_Of_List = True        
            # ADD remaining CURENT_PATH entries
            #...
    print("FINAL_REGISTER_PATH:%s"%FINAL_REGISTER_PATH)

#LBR = B
#LBS = R
#RBL = B
#SBL = R
#SBS = B
#LBL = S
def optimize_path(path_list):
    path = path_list[0]+path_list[1]+path_list[2]
    if(path == "LBR"):
        return "B"    
    if(path == "LBS"):
        return "R"    
    if(path == "RBL"):
        return "B"    
    if(path == "SBL"):
        return "R"    
    if(path == "SBS"):
        return "B"    
    if(path == "LBL"):
        return "S"    
    return None    
    

###################################################################################
######## Line follower code #######################################################
###################################################################################



def BasicLineFollower(board): #Black Line Follow on White surface
    global leftside,leftside_dark,leftside_light, leftfront,leftfront_dark,leftfront_light, rightfront,rightfront_dark,rightfront_light, rightside,rightside_dark,rightside_light, leftspeed, rightspeed
    updateTime = time.time() + updateInterval
    basespeed = 10000
    adjustment = 0
    prevdiff = 0
    board.RMOTOR_DIR.value(1)  # set the right motor direction
    board.LMOTOR_DIR.value(1)  # set the left motor direction
    while True:
        ukmarsbot_sensors.readSensors(board)
        
        if (rightfront > FRONTRIGHT_MARKER_THRESHOLD) :
            board.RIGHT_LED.value(1) # switch off sensor2 LED on line folllower board
        else:
            board.RIGHT_LED.value(0) # switch off sensor2 LED on line folllower board
        
        if (leftfront > FRONTLEFT_MARKER_THRESHOLD) :
            board.LEFT_LED.value(1) # switch off sensor2 LED on line folllower board
        else:
            board.LEFT_LED.value(0) # switch off sensor2 LED on line folllower board
        
        gSensorSum = rightfront + leftfront;
        gSensorDifference = rightfront - leftfront;
                    
        if(time.time() > updateTime):
            updateTime = updateTime + updateInterval
            print("-----------------------------------------------")
            print("left:(%d)-left front:(%d)-right front:(%d)-right:(%d)-sum:(%d)-diff:(%d)"% (leftside,leftfront,rightfront,rightside,gSensorSum,gSensorDifference))
        
        if(gSensorDifference > 2000):
            adjustment = 5000
        
        if(gSensorDifference < -2000):
            adjustment = -5000
        
        leftspeed = int(basespeed + adjustment)
        rightspeed = int(basespeed - adjustment)
        
        board.LMOTOR_PWM.duty_u16(leftspeed)
        board.RMOTOR_PWM.duty_u16(rightspeed)         

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
#MoveForwardDistance(cytron_board,53)
#turnLeftAngle(cytron_board,80)

#testReadSensorsNormalized(cytron_board)

# configure irq callback
#cytron_board.Leftenc1.irq(lambda p:leftcount())
#cytron_board.Rightenc1.irq(lambda p:rightcount())

cytron_board.LEFT_LED.value(0) # switch off sensor1 LED on line folllower board
cytron_board.RIGHT_LED.value(0) # switch off sensor2 LED on line folllower board
cytron_board.LMOTOR_PWM.duty_u16(0) # in range 0 to 65535
cytron_board.RMOTOR_PWM.duty_u16(0)

# wait for onboard button to be pressed
card_button = 1
# value for switch button
car_switch = 0

# First Loop for calibration
print("Calibration....")
button_pressed = 0
calibration_pass = 1
while (button_pressed == 0 and calibration_pass==1): # Check button 1 (GP20)
    card_button = cytron_board.btn1.value()
    if(card_button == 0):
        button_pressed = 1
        print("button_pressed:%d"%button_pressed)
        print("card_button:%d"%card_button)
        # Calibrate
        time.sleep(2)
        ukmarsbot_sensors.calibrateSensors()
        #button_pressed = 1
    time.sleep(0.1)

# Play tone
play_tone()

print("Maze Solving....")

# SECOND LOOP FOR MAZE SOLVER
button_pressed = 0
while (button_pressed == 0): # Check button 1 (GP20)
    card_button = cytron_board.btn1.value()
    if(card_button == 0):
        button_pressed = 1
        print("button_pressed:%d"%button_pressed)
        print("car_switch:%d"%car_switch)
        time.sleep(2)
        resolveMaze(cytron_board)
    time.sleep(0.1)


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


