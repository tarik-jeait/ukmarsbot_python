from machine import Pin, ADC, PWM
from _CytronMakerNanoRP2040 import CytronMakerNanoRP2040
import time
import math 
import ujson as json

# Global variable definitions
global leftside,leftfront,rightfront,rightside
global calibrated_data

# LINE FOLLOWER
LINE_WIDTH = 19.0;  # ADJUST THIS so that CTE is roughly equal to the error in mm
LINE_DETECT_THRESHOLD = 30000.0;  # minimum value to register the line - ADJUST TO SUIT
LEFT_MARKER_THRESHOLD = 15000.0;  # minimum value to register the turn marker
RIGHT_MARKER_THRESHOLD = 15000.0; # minimum value to register the start marker

gSensorSum=0
gSensorDifference=0
gSensorCTE=0
updateTime=0
updateInterval = 2;  # in milliseconds

def sensorsLineFollowerTest(board,retries_count):
    print("sensors basic reading for Line Follower ........")
    updateTime = time.time() + updateInterval
    retries = retries_count
    loop = True
    while loop:
        global leftside,leftfront,rightfront, rightside        
        board.TRIGGER_PIN.value(0)     # switch off LEDs on sensor board
        time.sleep(1)        
        leftside_dark = board.LEFT_SENSOR.read_u16()
        leftfront_dark = board.LEFT_FRONT_SENSOR.read_u16()
        rightfront_dark = board.RIGHT_FRONT_SENSOR.read_u16()
        rightside_dark = board.RIGHT_SENSOR.read_u16()
         
        board.TRIGGER_PIN.value(1)     # switch on LEDs on sensor board
        time.sleep(1)
        leftside_light = board.LEFT_SENSOR.read_u16()
        leftfront_light = board.LEFT_FRONT_SENSOR.read_u16()
        rightfront_light = board.RIGHT_FRONT_SENSOR.read_u16()
        rightside_light = board.RIGHT_SENSOR.read_u16()

        leftside = math.fabs(leftside_light - leftside_dark)
        leftfront = math.fabs(leftfront_light - leftfront_dark)
        rightfront = math.fabs(rightfront_light - rightfront_dark)
        rightside = math.fabs(rightside_light - rightside_dark)
        
        #
        if (rightside > RIGHT_MARKER_THRESHOLD) :
            board.RIGHT_LED.value(1) # switch off sensor2 LED on line folllower board
        else:
            board.RIGHT_LED.value(0) # switch off sensor2 LED on line folllower board
        
        if (leftside > LEFT_MARKER_THRESHOLD) :
            board.LEFT_LED.value(1) # switch off sensor2 LED on line folllower board
        else:
            board.LEFT_LED.value(0) # switch off sensor2 LED on line folllower board
        
        gSensorSum = rightfront + leftfront;
        gSensorDifference = rightfront - leftfront;
        if (gSensorSum > LINE_DETECT_THRESHOLD) :
            gSensorCTE = LINE_WIDTH * (gSensorDifference / gSensorSum);
        else:
            gSensorCTE = 0;
            
        if(time.time() > updateTime):
            updateTime = updateTime + updateInterval
            print("-----------------------------------------------")
            print("left:(%d)-left front:(%d)-right front:(%d)-right:(%d)-sum:(%d)-diff:(%d)-CTE:(%f)"% (leftside,leftfront,rightfront,rightside,gSensorSum,gSensorDifference,gSensorCTE))
        time.sleep(.5) # white around 4,500, black around 55,000
        retries = retries - 1
        if(retries < 0):
            loop = False


def readSensors(board):
    global leftside,leftfront,rightfront,rightside        
    board.TRIGGER_PIN.value(0)     # switch off LEDs on sensor board
    time.sleep(0.001)        
    leftside_dark = board.LEFT_SENSOR.read_u16()
    leftfront_dark = board.LEFT_FRONT_SENSOR.read_u16()
    rightfront_dark = board.RIGHT_SENSOR.read_u16()
    rightside_dark = board.RIGHT_SENSOR.read_u16()
    
    board.TRIGGER_PIN.value(1)     # switch on LEDs on sensor board
    time.sleep(0.001)
        
    leftside_light = board.LEFT_SENSOR.read_u16()
    leftfront_light = board.LEFT_FRONT_SENSOR.read_u16()
    rightfront_light = board.RIGHT_FRONT_SENSOR.read_u16()
    rightside_light = board.RIGHT_SENSOR.read_u16()

    leftside = math.fabs(leftside_light - leftside_dark)
    leftfront = math.fabs(leftfront_light - leftfront_dark)
    rightfront = math.fabs(rightfront_light - rightfront_dark)
    rightside = math.fabs(rightside_light - rightside_dark)      

def calibrateSensors(board):
    print("sensors calibration for Line Follower ........")

    global leftside,leftfront,rightfront,rightside    
    count = 5000
    index = count
    
    leftside_min = 10000
    leftside_max = 0
    
    leftfront_min = 10000
    leftfront_max = 0
    
    rightfront_min = 10000
    rightfront_max = 0
    
    rightside_min = 10000
    rightside_max = 0
    
    while index>0 :
        readSensors(board)
        
        leftside_min = min(leftside_min,leftside)
        leftside_max = max(leftside_max,leftside)
        
        leftfront_min = min(leftfront_min,leftfront)
        leftfront_max = max(leftfront_max,leftfront)

        rightfront_min = min(rightfront_min,rightfront)
        rightfront_max = max(rightfront_max,rightfront)
        
        rightside_min = min(rightside_min,rightside)
        rightside_max = max(rightside_max,rightside)
        
        index = index - 1
        
    print("leftside: Min:(%d) - Max:(%d)"% (leftside_min,leftside_max))    
    print("leftfront: Min:(%d) - Max:(%d)"% (leftfront_min,leftfront_max))    
    print("rightfront: Min:(%d) - Max:(%d)"% (rightfront_min,rightfront_max))    
    print("rightside: Min:(%d) - Max:(%d)"% (rightside_min,rightside_max))
    
    jsonData = {"leftside": {"min":leftside_min,"max":leftside_max},"leftfront":{"min":leftfront_min,"max":leftfront_max},"rightfront":{"min":rightfront_min,"max":rightfront_max},"rightside":{"min":rightside_min,"max":rightside_max}}
    
    save_calibrated_data(jsonData)

def save_calibrated_data(jsonData):
    try:
        with open('calibrated_data.json', 'w') as f:
            json.dump(jsonData, f)
    except:
        print("Could not save data.")
       
def read_calibrated_data():
    global calibrated_data
    try:
        with open('calibrated_data.json', 'r') as f:
            calibrated_data = json.load(f)        
    except:
        print("Could not read calibrated data")

def readSensorsNormalized(board):
    print("sensors reading normalized data for Line Follower ........")
    global calibrated_data
    read_calibrated_data()
    leftside_min = calibrated_data["leftside"]["min"]
    leftside_max = calibrated_data["leftside"]["max"]
    print("leftside:Min:(%d) - Max:(%d)"% (leftside_min,leftside_max))
    
    leftfront_min = calibrated_data["leftfront"]["min"]
    leftfront_max = calibrated_data["leftfront"]["max"]
    print("leftfront:Min:(%d) - Max:(%d)"% (leftfront_min,leftfront_max))
    
    rightfront_min = calibrated_data["rightfront"]["min"]
    rightfront_max = calibrated_data["rightfront"]["max"]
    print("rightfront:Min:(%d) - Max:(%d)"% (rightfront_min,rightfront_max))
    
    rightside_min = calibrated_data["rightside"]["min"]
    rightside_max = calibrated_data["rightside"]["max"]
    print("rightside:Min:(%d) - Max:(%d)"% (rightside_min,rightside_max))
    
    readSensors(board)
    
    print("leftside:%.2f%%"% ((leftside-leftside_min)/(leftside_max-leftside_min)*100))
    print("leftfront:%.2f%%"% ((leftfront-leftfront_min)/(leftfront_max-leftfront_min)*100))
    print("rightfront:%.2f%%"% ((rightfront-rightfront_min)/(rightfront_max-rightfront_min)*100))
    print("rightside:%.2f%%"% ((rightside-rightside_min)/(rightside_max-rightside_min)*100))


cytron_board = CytronMakerNanoRP2040()

# Basic reading of sensors (5 times)
sensorsLineFollowerTest(cytron_board,5)

# calibration of sensors reading
# move the manually ukmarsbot on the black surface / white lines
# the ukmarsbot should stay on the table during the calibration
# output : calibrated data (max & min values) for every sensors stored on the board
calibrateSensors(cytron_board)
# read sensors data and use calibrated data to get normalised sensors comprised between 0 - 100
readSensorsNormalized(cytron_board)