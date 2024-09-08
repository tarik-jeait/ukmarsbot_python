from _CytronMakerNanoRP2040 import CytronMakerNanoRP2040 
from machine import Pin, ADC, PWM
import neopixel
import time
import math
import ujson as json
from _LineFollowerSensors import Sensors
from sys import exit


# Global variable definitions
global leftside,leftfront,rightfront,rightside
global calibrated_data

calibrated_data = None

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
gSensorCTE=0
global updateTime,updateInterval
updateTime=0
updateInterval = 2;  # in milliseconds

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
######## Motors             #######################################################
###################################################################################

def stopMotors(board):
    board.LMOTOR_PWM.duty_u16(0)
    board.RMOTOR_PWM.duty_u16(0)         

def MoveForward(board,adjustment):
    board.RMOTOR_DIR.value(0)  # set the right motor direction
    board.LMOTOR_DIR.value(0)  # set the left motor direction   
    #Move Forward until a road crossing
    basespeed = 10000
    #adjustment = 0    
    leftspeed = int(basespeed + adjustment)
    rightspeed = int(basespeed - adjustment)
    
    board.LMOTOR_PWM.duty_u16(leftspeed)
    board.RMOTOR_PWM.duty_u16(rightspeed)         

def turnLeft(board):
    basespeed = 10000
    adjustment = 0
    # Move Forward for 5 cm before turn left    
    # Turn Left
    board.RMOTOR_DIR.value(0)  # set the right motor direction
    board.LMOTOR_DIR.value(1)  # set the left motor direction
    
    leftspeed = int(basespeed+adjustment)
    rightspeed = int(basespeed - adjustment)

    board.LMOTOR_PWM.duty_u16(leftspeed)
    board.RMOTOR_PWM.duty_u16(rightspeed)         
    
    time.sleep(0.5)        
    board.LMOTOR_PWM.duty_u16(0)
    board.RMOTOR_PWM.duty_u16(0)         
    
def turnRight(board):
    basespeed = 10000
    adjustment = 0
    board.RMOTOR_DIR.value(1)  # set the right motor direction
    board.LMOTOR_DIR.value(0)  # set the left motor direction
    
    leftspeed = int(basespeed+adjustment)
    rightspeed = int(basespeed - adjustment)
        
    board.LMOTOR_PWM.duty_u16(leftspeed)
    board.RMOTOR_PWM.duty_u16(rightspeed)         
    time.sleep(0.5)        
    board.LMOTOR_PWM.duty_u16(0)
    board.RMOTOR_PWM.duty_u16(0)         

def turnBackLeft(board):
    basespeed = 10000
    adjustment = 0
    board.RMOTOR_DIR.value(0)  # set the right motor direction
    board.LMOTOR_DIR.value(1)  # set the left motor direction
    
    leftspeed = int(basespeed + adjustment)
    rightspeed = int(basespeed - adjustment)
        
    board.LMOTOR_PWM.duty_u16(leftspeed)
    board.RMOTOR_PWM.duty_u16(rightspeed)         
    time.sleep(1.0)        
    board.LMOTOR_PWM.duty_u16(0)
    board.RMOTOR_PWM.duty_u16(0)

def resolveMaze(board):
    # Move Forward
    MoveForward(board,0)    
    crossroads = False
    while crossroads == False:
        sensors = readSensorsNormalized(board)
        print("L:%3.2f - LF:%3.2f - RF:%3.2f - R:%3.2f"% (sensors.leftside_normalized,sensors.leftfront_normalized,sensors.rightfront_normalized,sensors.rightside_normalized))
        if(sensors.isCrossRoads() == True):
            # move few mm
            #MoveForward(board,0)    
            time.sleep(0.1)
            print("CrossRoads...")
            stopMotors(board)
            sensors = readSensorsNormalized(board)
            time.sleep(0.2)
            #
            if(sensors.isRouteLeft()):
                MoveForward(board,0)
                time.sleep(0.4)
                stopMotors(board)
                # Check Again if it is the end
                sensors_end = readSensorsNormalized(board)
                if(sensors_end.isTheEnd()):
                    stopMotors(board)
                    crossroads = True
                else:
                    turnLeft(board)
                    time.sleep(0.4)
                    MoveForward(board,0)
            else:
                if(sensors.isRouteRight()):
                    MoveForward(board,0)
                    time.sleep(0.4)
                    stopMotors(board)
                    # Check Again if it is the end
                    sensors_end = readSensorsNormalized(board)
                    if(sensors_end.isTheEnd()):
                        stopMotors(board)
                        crossroads = True
                    else:
                        turnRight(board)
                        time.sleep(0.4)
                        MoveForward(board,0)
                else:
                    stopMotors(board)
                    crossroads = True
        else:
            if(sensors.isBlackSurface()):
                turnBackLeft(board)
                time.sleep(0.5)
                MoveForward(board,0)
            else:
                adjustment = 0
                # compute adjustment
                gSensorDifference = sensors.rightfront - sensors.leftfront;
                if(gSensorDifference > 2000):
                    adjustment = 5000        
                if(gSensorDifference < -2000):
                    adjustment = -5000
                MoveForward(board,adjustment)
        time.sleep(0.01)
       
###################################################################################
######## Line follower code #######################################################
###################################################################################
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
    global leftside,leftfront,rightfront,rightside    
    count = 1000
    index = count
    
    leftside_min = 10000
    leftside_max = 0    
    leftfront_min = 10000
    leftfront_max = 0    
    rightfront_min = 10000
    rightfront_max = 0
    rightside_min = 10000
    rightside_max = 0

    basespeed = 10000
    adjustment = 0
    board.RMOTOR_DIR.value(1)  # set the right motor direction
    board.LMOTOR_DIR.value(0)  # set the left motor direction
    
    leftspeed = int(basespeed + adjustment)
    rightspeed = int(basespeed - adjustment)
        
    board.LMOTOR_PWM.duty_u16(leftspeed)
    board.RMOTOR_PWM.duty_u16(rightspeed)         

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
    
    #STOP MOTORS AFTER CALIBRATION
    board.LMOTOR_PWM.duty_u16(0)
    board.RMOTOR_PWM.duty_u16(0)         
    
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
    if calibrated_data is None:
        try:
            with open('calibrated_data.json', 'r') as f:
                calibrated_data = json.load(f)        
        except:
            print("Could not read calibrated data")

def readSensorsNormalized(board):
    global calibrated_data
    read_calibrated_data()
    leftside_min = calibrated_data["leftside"]["min"]
    leftside_max = calibrated_data["leftside"]["max"]
    #print("leftside:Min:(%d) - Max:(%d)"% (leftside_min,leftside_max))
    
    leftfront_min = calibrated_data["leftfront"]["min"]
    leftfront_max = calibrated_data["leftfront"]["max"]
    #print("leftfront:Min:(%d) - Max:(%d)"% (leftfront_min,leftfront_max))
    
    rightfront_min = calibrated_data["rightfront"]["min"]
    rightfront_max = calibrated_data["rightfront"]["max"]
    #print("rightfront:Min:(%d) - Max:(%d)"% (rightfront_min,rightfront_max))
    
    rightside_min = calibrated_data["rightside"]["min"]
    rightside_max = calibrated_data["rightside"]["max"]
    #print("rightside:Min:(%d) - Max:(%d)"% (rightside_min,rightside_max))
    
    readSensors(board)
    
    #print("leftside:%.2f%%"% ((leftside-leftside_min)/(leftside_max-leftside_min)*100))
    #print("leftfront:%.2f%%"% ((leftfront-leftfront_min)/(leftfront_max-leftfront_min)*100))
    #print("rightfront:%.2f%%"% ((rightfront-rightfront_min)/(rightfront_max-rightfront_min)*100))
    #print("rightside:%.2f%%"% ((rightside-rightside_min)/(rightside_max-rightside_min)*100))
    sensors = Sensors(board)
    
    sensors.leftside = leftside
    sensors.leftfront = leftfront
    sensors.rightfront = rightfront
    sensors.rightside = rightside
    
    sensors.leftside_normalized = (leftside-leftside_min)/(leftside_max-leftside_min)*100
    sensors.leftfront_normalized = (leftfront-leftfront_min)/(leftfront_max-leftfront_min)*100
    sensors.rightfront_normalized = (rightfront-rightfront_min)/(rightfront_max-rightfront_min)*100
    sensors.rightside_normalized = (rightside-rightside_min)/(rightside_max-rightside_min)*100
    
    
    return sensors    

def testReadSensorsNormalized(board):
    global updateTime
    LOOP = True
    while LOOP:
        s = readSensorsNormalized(board)
        gSensorDifference = s.rightfront - s.leftfront
        if(time.time() > updateTime):
            updateTime = updateTime + updateInterval
            print("-----------------------------------------------")
            print("left:(%d)-left front:(%d)-right front:(%d)-right:(%d)-diff:(%d)"% (s.leftside,s.leftfront,s.rightfront,s.rightside,gSensorDifference))
            print("left n:(%d)-left front n:(%d)-right front n:(%d)-right n:(%d)-diff:(%d)"% (s.leftside_normalized,s.leftfront_normalized,s.rightfront_normalized,s.rightside_normalized,gSensorDifference))
            
        time.sleep(.5) # white around 4,500, black around 55,000



def BasicLineFollower(board): #Black Line Follow on White surface
    global leftside,leftside_dark,leftside_light, leftfront,leftfront_dark,leftfront_light, rightfront,rightfront_dark,rightfront_light, rightside,rightside_dark,rightside_light, leftspeed, rightspeed
    updateTime = time.time() + updateInterval
    basespeed = 10000
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

#
button_pressed = 0

while (button_pressed == 0): # Check button 1 (GP20)
    card_button = cytron_board.btn1.value()
    if(card_button == 0):
        button_pressed = 1
        print("button_pressed:%d"%button_pressed)
        print("card_button:%d"%card_button)
        # Calibrate
        time.sleep(2)
        calibrateSensors(cytron_board)
        button_pressed = 0
    else:
        car_switch = cytron_board.SWITCH.value()
        if(car_switch == 1):
            button_pressed = 1
            print("button_pressed:%d"%button_pressed)
            print("car_switch:%d"%car_switch)
            time.sleep(2)
            resolveMaze(cytron_board)
            button_pressed = 0
    #print ("progno", progno)
    #progdisp()
    #encountdisp()
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
