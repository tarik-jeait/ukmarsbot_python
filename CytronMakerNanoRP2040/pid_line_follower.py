from CytronMakerNanoRP2040 import CytronMakerNanoRP2040 
from machine import Pin, ADC, PWM
import neopixel
import time
import math

# Global variable definitions
global leftside,leftside_dark,leftside_light, leftfront,leftfront_dark,leftfront_light, rightfront,rightfront_dark,rightfront_light, rightside,rightside_dark,rightside_light
global l1count, l2count, r1count, r2count, prevleft1, prevleft2, prevright1, prevright2
global bit0, bit1, bit2, bit3

# LINE FOLLOWER

LINE_WIDTH = 19.0;  # ADJUST THIS so that CTE is roughly equal to the error in mm

LINE_DETECT_THRESHOLD = 30000.0;  # minimum value to register the line - ADJUST TO SUIT
FRONTLEFT_MARKER_THRESHOLD = 5000.0;  # minimum value to register the turn marker
FRONTRIGHT_MARKER_THRESHOLD = 5000.0; # minimum value to register the start marker
LEFT_MARKER_THRESHOLD = 15000.0;  # minimum value to register the turn marker
RIGHT_MARKER_THRESHOLD = 15000.0; # minimum value to register the start marker

gSensorSum=0
gSensorDifference=0
gSensorDifferencePrevious=0

gSensorCTE=0
updateTime=0
updateInterval = 2;  # in milliseconds

Kp = 0.15
Kd = Kp/2.0

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

###################################################################################
######## Line follower code #######################################################
###################################################################################
def readSensors(board):
        global leftside,leftside_dark,leftside_light, leftfront,leftfront_dark,leftfront_light, rightfront,rightfront_dark,rightfront_light, rightside,rightside_dark,rightside_light        
        board.TRIGGER_PIN.value(0)     # switch off LEDs on sensor board
        time.sleep(0.01)        
        leftside_dark = board.LEFT_SENSOR.read_u16()
        leftfront_dark = board.LEFT_FRONT_SENSOR.read_u16()
        rightfront_dark = board.RIGHT_SENSOR.read_u16()
        rightside_dark = board.RIGHT_SENSOR.read_u16()
        
        board.TRIGGER_PIN.value(1)     # switch on LEDs on sensor board
        time.sleep(0.01)
        
        leftside_light = board.LEFT_SENSOR.read_u16()
        leftfront_light = board.LEFT_FRONT_SENSOR.read_u16()
        rightfront_light = board.RIGHT_FRONT_SENSOR.read_u16()
        rightside_light = board.RIGHT_SENSOR.read_u16()

        leftside = math.fabs(leftside_light - leftside_dark)
        leftfront = math.fabs(leftfront_light - leftfront_dark)
        rightfront = math.fabs(rightfront_light - rightfront_dark)
        rightside = math.fabs(rightside_light - rightside_dark)      

def PdLineFollower(board): #Black Line Follow on White surface
    global leftside,leftside_dark,leftside_light, leftfront,leftfront_dark,leftfront_light, rightfront,rightfront_dark,rightfront_light, rightside,rightside_dark,rightside_light, leftspeed, rightspeed
    updateTime = time.time() + updateInterval
    basespeed = 10000
    side = 0
    adjustment = 0.0
    prevdiff = 0
    board.RMOTOR_DIR.value(0)  # set the right motor direction
    board.LMOTOR_DIR.value(0)  # set the left motor direction
    gSensorDifferencePrevious = 0
    gSensorDifference = 0
    while True:
        readSensors(board)
        
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
        
        if (gSensorSum > LINE_DETECT_THRESHOLD) :
            gSensorCTE = LINE_WIDTH * (gSensorDifference / gSensorSum);
        else:
            gSensorCTE = 0;
            
        if(time.time() > updateTime):
            updateTime = updateTime + updateInterval
            print("-----------------------------------------------")
            print("left:(%d)-left front:(%d)-right front:(%d)-right:(%d)-sum:(%d)-diff:(%d)-CTE:(%f)"% (leftside,leftfront,rightfront,rightside,gSensorSum,gSensorDifference,gSensorCTE))
        
        adjustment = (gSensorDifference * Kp + gSensorDifferencePrevious*Kd)*5.0
        
        print("adjustment:(%f)-sum:(%d)-diff:(%d)-CTE:(%f)"% (adjustment,gSensorSum,gSensorDifference,gSensorCTE))
                
        leftspeed = int(basespeed + adjustment)
        rightspeed = int(basespeed - adjustment)
        
        board.LMOTOR_PWM.duty_u16(leftspeed)
        board.RMOTOR_PWM.duty_u16(rightspeed)
        
        gSensorDifferencePrevious = gSensorDifference

def PdLineFollowerFast(board): #Black Line Follow on White surface
    global leftside,leftside_dark,leftside_light, leftfront,leftfront_dark,leftfront_light, rightfront,rightfront_dark,rightfront_light, rightside,rightside_dark,rightside_light, leftspeed, rightspeed
    updateTime = time.time() + updateInterval
    basespeed = 45000
    side = 0
    adjustment = 0.0
    prevdiff = 0
    board.RMOTOR_DIR.value(1)  # set the right motor direction
    board.LMOTOR_DIR.value(1)  # set the left motor direction
    gSensorDifferencePrevious = 0
    gSensorDifference = 0
    while True:
        readSensors(board)
        
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
        
        if (gSensorSum > LINE_DETECT_THRESHOLD) :
            gSensorCTE = LINE_WIDTH * (gSensorDifference / gSensorSum);
        else:
            gSensorCTE = 0;
            
        if(time.time() > updateTime):
            updateTime = updateTime + updateInterval
            print("-----------------------------------------------")
            print("left:(%d)-left front:(%d)-right front:(%d)-right:(%d)-sum:(%d)-diff:(%d)-CTE:(%f)"% (leftside,leftfront,rightfront,rightside,gSensorSum,gSensorDifference,gSensorCTE))
        
        adjustment = (gSensorDifference * Kp + gSensorDifferencePrevious*Kd)*7.0
        
        print("adjustment:(%f)-sum:(%d)-diff:(%d)-CTE:(%f)"% (adjustment,gSensorSum,gSensorDifference,gSensorCTE))
                
        leftspeed = int(basespeed + adjustment)
        rightspeed = int(basespeed - adjustment)
        
        board.LMOTOR_PWM.duty_u16(leftspeed)
        board.RMOTOR_PWM.duty_u16(rightspeed)
        
        gSensorDifferencePrevious = gSensorDifference


def PidLineFollower(board): #Black Line Follow on White surface
    global leftside,leftside_dark,leftside_light, leftfront,leftfront_dark,leftfront_light, rightfront,rightfront_dark,rightfront_light, rightside,rightside_dark,rightside_light, leftspeed, rightspeed
    updateTime = time.time() + updateInterval
    basespeed = 30000
    side = 0
    adjustment = 0
    prevdiff = 0
    board.RMOTOR_DIR.value(1)  # set the right motor direction
    board.LMOTOR_DIR.value(1)  # set the left motor direction
    while True:
        readSensors(board)
        
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
        
        if (gSensorSum > LINE_DETECT_THRESHOLD) :
            gSensorCTE = LINE_WIDTH * (gSensorDifference / gSensorSum);
        else:
            gSensorCTE = 0;
            
        if(time.time() > updateTime):
            updateTime = updateTime + updateInterval
            print("-----------------------------------------------")
            print("left:(%d)-left front:(%d)-right front:(%d)-right:(%d)-sum:(%d)-diff:(%d)-CTE:(%f)"% (leftside,leftfront,rightfront,rightside,gSensorSum,gSensorDifference,gSensorCTE))
        if(gSensorDifference > 2000):
            adjustment = 10000
        
        if(gSensorDifference < -2000):
            adjustment = -10000
        
        leftspeed = int(basespeed + adjustment)
        rightspeed = int(basespeed - adjustment)
        
        board.LMOTOR_PWM.duty_u16(leftspeed)
        board.RMOTOR_PWM.duty_u16(rightspeed)


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
 
    
# ------- End of definitions -----------------------------------------------------
#---------------------------------------------------------------------------------
# Melody
MELODY_NOTE = [659, 659, 0, 659, 0, 523, 659, 0, 784]
MELODY_DURATION = [0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15, 0.2]

#-------------------------------------------------

#LED_PIN.value(1)     # switch on LED
#LEFT_LED.value(1) # switch on sensor1 LED on line folllower board
#RIGHT_LED.value(1) # switch on sensor2 LED on line folllower board


#RMOTOR_DIR.value(1)  # set the right motor direction
#LMOTOR_DIR.value(1)  # set the left motor direction


cytron_board = CytronMakerNanoRP2040()

#color = 0 
#state = 0

# Play tone
PIEZO_PWM = machine.PWM(machine.Pin(22))
PIEZO_PWM.freq(440)
PIEZO_PWM.duty_u16(30000) # in range 0 to 65535
time.sleep(0.5)
PIEZO_PWM.duty_u16(0)

progno = 0
swvalue = 0


# ENCODERS TEST
l1count = l2count = r1count = r2count = 0 #reset the encoder counts
left1 = left2 = right1 = right2 = True
prevleft1 = prevleft2 = prevright1 = prevright2 = True # reset the previous encoder values

# configure irq callback
#cytron_board.Leftenc1.irq(lambda p:leftcount())
#cytron_board.Rightenc1.irq(lambda p:rightcount())

cytron_board.LEFT_LED.value(0) # switch off sensor1 LED on line folllower board
cytron_board.RIGHT_LED.value(0) # switch off sensor2 LED on line folllower board
cytron_board.LMOTOR_PWM.duty_u16(0) # in range 0 to 65535
cytron_board.RMOTOR_PWM.duty_u16(0)

# wait for onboard button to be pressed
setting = 1
while (setting == True): # Check button 1 (GP20)
    setting = cytron_board.btn1.value()
    #print ("progno", progno)
    #progdisp()
    #encountdisp()     
    time.sleep(0.1)
    slowdownby = 0

time.sleep(2)


#sensorsBasicTest()
#sensorsLineFollowerTest()
#switchTest()

# MOTORS BASIC TEST
# move forward -> direction = 0 | move backward -> direction = 180
#motorsBasicTest(0)
#encodertest()

#BasicLineFollower(cytron_board)
PdLineFollower(cytron_board)

#PdLineFollowerFast(cytron_board)

cytron_board.LEFT_LED.value(0) # switch off sensor1 LED on line folllower board
cytron_board.RIGHT_LED.value(0) # switch off sensor2 LED on line folllower board
cytron_board.LMOTOR_PWM.duty_u16(0) # in range 0 to 65535
cytron_board.RMOTOR_PWM.duty_u16(0)