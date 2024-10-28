from machine import Pin, ADC, PWM
from _CytronMakerNanoRP2040 import CytronMakerNanoRP2040
import time
import math 
import ujson as json

# Global variable definitions
global leftside,front,rightside
global calibrated_data


gSensorSum=0
gSensorDifference=0
gSensorCTE=0
updateTime=0.5
updateInterval = 0;  # in milliseconds

def sensorsWallFollowerTest(board,retries_count):
    print("sensors basic reading for Wall Follower ........")
    updateTime = time.time() + updateInterval
    retries = retries_count
    loop = True
    while loop:
        global leftside,front,rightside        
        board.TRIGGER_PIN.value(0)     # switch off LEDs on sensor board
        time.sleep(1)        
        leftside_dark = board.WALL_LEFT_SENSOR.read_u16()
        front_dark = board.WALL_FRONT_SENSOR.read_u16()
        rightside_dark = board.WALL_RIGHT_SENSOR.read_u16()
        
        
        board.TRIGGER_PIN.value(1)     # switch on LEDs on sensor board
        time.sleep(1)
        leftside_light = board.WALL_LEFT_SENSOR.read_u16()
        front_light = board.WALL_FRONT_SENSOR.read_u16()
        rightside_light = board.WALL_RIGHT_SENSOR.read_u16()      

        leftside = math.fabs(leftside_light - leftside_dark)
        front = math.fabs(front_light - front_dark)
        rightside = math.fabs(rightside_light - rightside_dark)
       
           
        if(time.time() > updateTime):
            updateTime = updateTime + updateInterval
            print("-----------------------------------------------")
            print("left:(%d)-front:(%d)-right:(%d)"% (leftside,front,rightside))
        time.sleep(.5) # white around 4,500, black around 55,000
        retries = retries - 1
        if(retries < 0):
            loop = False


def readSensors(board):
    global leftside,front,rightside        
    board.TRIGGER_PIN.value(0)     # switch off LEDs on sensor board
    time.sleep(0.001)        

    leftside_dark = board.WALL_LEFT_SENSOR.read_u16()
    front_dark = board.WALL_FRONT_SENSOR.read_u16()
    rightside_dark = board.WALL_RIGHT_SENSOR.read_u16()
    
    board.TRIGGER_PIN.value(1)     # switch on LEDs on sensor board
    time.sleep(0.001)
        
    leftside_light = board.WALL_LEFT_SENSOR.read_u16()
    front_light = board.WALL_FRONT_SENSOR.read_u16()
    rightside_light = board.WALL_RIGHT_SENSOR.read_u16()      

    leftside = math.fabs(leftside_light - leftside_dark)
    front = math.fabs(front_light - front_dark)
    rightside = math.fabs(rightside_light - rightside_dark)

def calibrateSensors(board):
    print("sensors calibration for Line Follower ........")

    global leftside,front,rightside   
    count = 10000
    index = count
    
    leftside_min = 10000
    leftside_max = 0
    
    front_min = 10000
    front_max = 0
    
    rightside_min = 10000
    rightside_max = 0
    
    while index>0 :
        readSensors(board)
        
        leftside_min = min(leftside_min,leftside)
        leftside_max = max(leftside_max,leftside)
        
        front_min = min(front_min,front)
        front_max = max(front_max,front)
        
        rightside_min = min(rightside_min,rightside)
        rightside_max = max(rightside_max,rightside)
        
        index = index - 1
        
    print("leftside: Min:(%d) - Max:(%d)"% (leftside_min,leftside_max))    
    print("front: Min:(%d) - Max:(%d)"% (front_min,front_max))    
    print("rightside: Min:(%d) - Max:(%d)"% (rightside_min,rightside_max))
    
    jsonData = {"leftside": {"min":leftside_min,"max":leftside_max},"front":{"min":front_min,"max":front_max},"rightside":{"min":rightside_min,"max":rightside_max}}
    
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
    
    front_min = calibrated_data["front"]["min"]
    front_max = calibrated_data["front"]["max"]
    print("front:Min:(%d) - Max:(%d)"% (front_min,front_max))
        
    rightside_min = calibrated_data["rightside"]["min"]
    rightside_max = calibrated_data["rightside"]["max"]
    print("rightside:Min:(%d) - Max:(%d)"% (rightside_min,rightside_max))
    
    readSensors(board)
    
    print("leftside:%.2f%%"% ((leftside-leftside_min)/(leftside_max-leftside_min)*100))
    print("front:%.2f%%"% ((front-front_min)/(front_max-front_min)*100))
    print("rightside:%.2f%%"% ((rightside-rightside_min)/(rightside_max-rightside_min)*100))


cytron_board = CytronMakerNanoRP2040()

# Basic reading of sensors (5 times)
sensorsWallFollowerTest(cytron_board,5)

# calibration of sensors reading
# move the manually ukmarsbot on the black surface / white lines
# the ukmarsbot should stay on the table during the calibration
# output : calibrated data (max & min values) for every sensors stored on the board
calibrateSensors(cytron_board)
# read sensors data and use calibrated data to get normalised sensors comprised between 0 - 100
readSensorsNormalized(cytron_board)