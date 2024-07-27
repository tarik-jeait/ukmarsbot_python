from machine import Pin, ADC, PWM
from CytronMakerNanoRP2040 import CytronMakerNanoRP2040
import time
import math 

# Global variable definitions
global leftside,leftside_dark,leftside_light, leftfront,leftfront_dark,leftfront_light, rightfront,rightfront_dark,rightfront_light, rightside,rightside_dark,rightside_light
global l1count, l2count, r1count, r2count, prevleft1, prevleft2, prevright1, prevright2
global bit0, bit1, bit2, bit3


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
    updateTime = time.time() + updateInterval
    retries = retries_count
    loop = True
    while loop:
        global leftside,leftside_dark,leftside_light, leftfront,leftfront_dark,leftfront_light, rightfront,rightfront_dark,rightfront_light, rightside,rightside_dark,rightside_light        
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


def motorsBasicTest(board,direction):
    global leftfront, rightfront, leftspeed, rightspeed
    adjustment=0
    basespeed = 30000
    if(direction==0):
        # GO Forward
        board.RMOTOR_DIR.value(1)  # set the right motor direction
        board.LMOTOR_DIR.value(1)  # set the left motor direction
        leftspeed = int(basespeed + adjustment)
        rightspeed = int(basespeed - adjustment)
        board.LMOTOR_PWM.duty_u16(leftspeed)
        board.RMOTOR_PWM.duty_u16(rightspeed)
        time.sleep(2)
        board.LMOTOR_PWM.duty_u16(0)
        board.RMOTOR_PWM.duty_u16(0)
    else :
        if(direction==180):
            board.RMOTOR_DIR.value(0)  # set the right motor direction
            board.LMOTOR_DIR.value(0)  # set the left motor direction
            # GO Forward
            leftspeed = int(basespeed + adjustment)
            rightspeed = int(basespeed - adjustment)
            board.LMOTOR_PWM.duty_u16(leftspeed)
            board.RMOTOR_PWM.duty_u16(rightspeed)
            time.sleep(2)
            board.LMOTOR_PWM.duty_u16(0)
            board.RMOTOR_PWM.duty_u16(0)


cytron_board = CytronMakerNanoRP2040()
sensorsLineFollowerTest(cytron_board,5)
motorsBasicTest(cytron_board,0)
motorsBasicTest(cytron_board,180)
