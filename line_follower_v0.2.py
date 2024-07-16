# UKMARSbot running python on maker nano RP2040 board using MicroPython
# 5V cpu board socket pin removed and 5v and 3.3v lines connected on back of the sensor connector
# R7 10K resistor removed to stop battery voltage feed to CPU
# In Thonny on windows PC  - From Run, Select Interpreter then MicroPython (Raspberry Pi Pico)
# 10:1 gear motors with encoders
# uses 2 LEDs on sensor board plus the mainboard red LED to select programs no 0 to 7
# press PCB switch to advance th eprogram number from 0 through to 7
# - program number is dislayed as 3 bits of binary
# prog 0 = wide line follow, 1 = narrow line follow, 2 = drag race, 3 = phototest, 4 = encodertest
#
# to use the press button set all the 4 switches to on. When button pressed switchvalue goes from 0 to 1
# if leftmost switch is set to off, switch value will always read 1

from machine import Pin, ADC, PWM
import neopixel
import time
import math

# Global variable definitions
global leftside,leftside_dark,leftside_light, leftfront,leftfront_dark,leftfront_light, rightfront,rightfront_dark,rightfront_light, rightside,rightside_dark,rightside_light
global l1count, l2count, r1count, r2count, prevleft1, prevleft2, prevright1, prevright2
global bit0, bit1, bit2, bit3

# pin definitions
# Set pins for digital outputs on Maker Pi RP2040
LED_PIN = Pin(18, Pin.OUT) # ext LED on GP18

###################################################################################
######## SENSORS ##################################################################
# Arduino D11 : LEFT LED - LED5
LEFT_LED = Pin(19, Pin.OUT) # 1st Sensor LED on line follow sensor board
# Arduino D6 : RIGHT LED - LED4
RIGHT_LED = Pin(6, Pin.OUT) # 2nd Sensor LED on line follow sensor board
#Arduino D12 : Sensor EMITTER
TRIGGER_PIN = Pin(16, Pin.OUT) # Trigger for LEDs on line follow sensor board
# Define analogue inputs used on sensor board
# T1 / RIGHT SENSOR
Lsidesense = ADC(Pin(29))   # A3
# T2 / LEFT FRONT ANALOG SENSOR
Lfrontsense = ADC(Pin(28))  # A2
# T3 / RIGHT FRONT ANALOG SENSOR
Rfrontsense = ADC(Pin(27))  # A1
# T4 / RIGHT ANALOG SENSOR
Rsidesense = ADC(Pin(26))   # A0

###################################################################################
######## BUTTONS ##################################################################

# Define RP2040 on-board button with pull up so it goes false when pressed
# USER BUTTON marked as in the cytron maker card : GP20
btn1 = machine.Pin(20, Pin.IN, Pin.PULL_UP)
# define press button switch on UKMARS board next to 4 way switch which should be set to all on
# Arduino A6 : FUNCTION_SELECT SWITCH
Switch = Pin(14, Pin.IN) # button / switch pin

###################################################################################
######## SOUND   ##################################################################

PIEZO_PIN = Pin(22, Pin.OUT) # Pin connected to piezo buzzer 

###################################################################################
######## MOTORS  ##################################################################

RMOTOR_DIR = Pin(8, Pin.OUT)
LMOTOR_DIR = Pin(7, Pin.OUT) 

#define PWM motor speed pins
#LMOTOR = machine.Pin(9) # left motor is pin 9
#MOTOR = machine.Pin(17) # right motor is pin 17

LMOTOR_PWM = machine.PWM(machine.Pin(9))
RMOTOR_PWM = machine.PWM(machine.Pin(17))
LMOTOR_PWM.freq(2000)
RMOTOR_PWM.freq(2000)

LMOTOR_PWM.duty_u16(0) # left motor speed - range is 0 to 65535
RMOTOR_PWM.duty_u16(0) # right motor speed - range is 0 to 65535

Leftenc1 = Pin(2, Pin.IN) # encoder pins
Leftenc2 = Pin(4, Pin.IN)
Rightenc1 = Pin(3, Pin.IN)
Rightenc2 = Pin(5, Pin.IN)

# Initialize Neopixel RGB LEDs
#pixels = neopixel.NeoPixel(11,Pin.OUT, 2)
#pixels.fill(0)


###################################################################################
######## TEST SENSORS  ############################################################

# BASIC TEST

def sensorsBasicTest():
    while True:
        global leftside,leftside_dark,leftside_light, leftfront,leftfront_dark,leftfront_light, rightfront,rightfront_dark,rightfront_light, rightside,rightside_dark,rightside_light
        TRIGGER_PIN.value(0)     # switch off LEDs on sensor board
        time.sleep(1)        
        leftside_dark = Lsidesense.read_u16()
        leftfront_dark = Lfrontsense.read_u16()
        rightfront_dark = Rfrontsense.read_u16()
        rightside_dark = Rsidesense.read_u16()
        
        TRIGGER_PIN.value(1)     # switch on LEDs on sensor board
        time.sleep(1)
        leftside_light = Lsidesense.read_u16()
        leftfront_light = Lfrontsense.read_u16()
        rightfront_light = Rfrontsense.read_u16()
        rightside_light = Rsidesense.read_u16()

        leftside = leftside_light - leftside_dark
        leftfront = leftfront_light - leftfront_dark
        rightfront = rightfront_light - rightfront_dark
        rightside = rightside_light - rightside_dark
        print("-----------------------------------------------")
        print("left sensor-------->(light:%d) - (dark:%d) = %d"% (leftside_light,leftside_dark,leftside))
        print("left front sensor-->(light:%d) - (dark:%d) = %d"% (leftfront_light,leftfront_dark,leftfront))
        print("right front sensor->(light:%d) - (dark:%d) = %d"% (rightfront_light,rightfront_dark,rightfront))
        print("right sensor------->(light:%d) - (dark:%d) = %d"% (rightside_light,rightside_dark,rightside))
        time.sleep(.5) # white around 4,500, black around 55,000

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
updateTime=0
updateInterval = 2;  # in milliseconds

def sensorsLineFollowerTest():
    updateTime = time.time() + updateInterval
    while True:
        global leftside,leftside_dark,leftside_light, leftfront,leftfront_dark,leftfront_light, rightfront,rightfront_dark,rightfront_light, rightside,rightside_dark,rightside_light        
        TRIGGER_PIN.value(0)     # switch off LEDs on sensor board
        time.sleep(1)        
        leftside_dark = Lsidesense.read_u16()
        leftfront_dark = Lfrontsense.read_u16()
        rightfront_dark = Rfrontsense.read_u16()
        rightside_dark = Rsidesense.read_u16()
        
        TRIGGER_PIN.value(1)     # switch on LEDs on sensor board
        time.sleep(1)
        
        leftside_light = Lsidesense.read_u16()
        leftfront_light = Lfrontsense.read_u16()
        rightfront_light = Rfrontsense.read_u16()
        rightside_light = Rsidesense.read_u16()

        leftside = math.fabs(leftside_light - leftside_dark)
        leftfront = math.fabs(leftfront_light - leftfront_dark)
        rightfront = math.fabs(rightfront_light - rightfront_dark)
        rightside = math.fabs(rightside_light - rightside_dark)
        
        #
        if (rightside > RIGHT_MARKER_THRESHOLD) :
            RIGHT_LED.value(1) # switch off sensor2 LED on line folllower board
        else:
            RIGHT_LED.value(0) # switch off sensor2 LED on line folllower board
        
        if (leftside > LEFT_MARKER_THRESHOLD) :
            LEFT_LED.value(1) # switch off sensor2 LED on line folllower board
        else:
            LEFT_LED.value(0) # switch off sensor2 LED on line folllower board
        
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
    

###################################################################################
######## TEST BUTTONS  ############################################################

switch_voltage = 0
def switchTest():
    while True:
        switch_voltage = switch.read_u16()
        print("switch_voltage:%d"% switch_voltage)
        time.sleep(1)

###################################################################################
######## TEST MOTORS   ############################################################
###################################################################################
def encoderread():
    global lefte1, lefte2, righte1, righte2
    lefte1 = Leftenc1.value
    lefte2 = Leftenc2.value
    righte1 = Rightenc1.value
    righte2 = Rightenc2.value

def encodertest():
    global lefte1, lefte2, righte1, righte2, l1count, l2count, r1count, r2count
    while True:
        encoderread()
        print((l1count), (l2count), (r1count), (r2count),)
        time.sleep(.5)
    return    

def leftcount():
    global lefte1, lefte2, righte1, righte2, l1count, l2count, r1count, r2count
    l1count = l1count+1
    l2count = l2count+1
    return

def rightcount():
    global lefte1, lefte2, righte1, righte2, l1count, l2count, r1count, r2count
    r1count = r1count+1
    r2count = r2count+1
    return

def encountdisp(): 
    global l1count, l2count, r1count, r2count
    global bit0, bit1, bit2, progno
    p1count = int(l1count / 8)
    progno = p1count - (int(p1count / 8) * 8) # this gets it in range 0 to 7
    bit2 = p1count - (int(p1count / 2) * 2) # this extracts bit 3
    p1count = int(p1count / 2)
    bit1 = p1count - (int(p1count / 2) * 2) # this extracts bit 2
    p1count = int(p1count / 2)
    bit0 = p1count - (int(p1count / 2) * 2) # this extracts bit 1
    p1count = int(p1count / 2)
    if (bit0 == 1):
        LEFT_LED.value(True)
    else:
        LEFT_LED.value(False)
    if (bit1 == 1):
        RIGHT_LED.value(True)
    else:
        RIGHT_LED.value(False)
    if (bit2 == 1):
        LED_PIN.value(True)
    else:
        LED_PIN.value(False) 

def encodertest():
    global lefte1, lefte2, righte1, righte2, l1count, l2count, r1count, r2count
    while True:
        encoderread()
        print((l1count), (l2count), (r1count), (r2count),)
        time.sleep(.5)
    return

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

def motorsBasicTest(direction):
    global leftfront, rightfront, leftspeed, rightspeed
    adjustment=0
    basespeed = 16000
    if(direction==0):
        # GO Forward
        RMOTOR_DIR.value(1)  # set the right motor direction
        LMOTOR_DIR.value(1)  # set the left motor direction
        leftspeed = int(basespeed + adjustment)
        rightspeed = int(basespeed - adjustment)
        LMOTOR_PWM.duty_u16(leftspeed)
        RMOTOR_PWM.duty_u16(rightspeed)
        time.sleep(2)
        LMOTOR_PWM.duty_u16(0)
        RMOTOR_PWM.duty_u16(0)
    else :
        if(direction==180):
            RMOTOR_DIR.value(0)  # set the right motor direction
            LMOTOR_DIR.value(0)  # set the left motor direction
            # GO Forward
            leftspeed = int(basespeed + adjustment)
            rightspeed = int(basespeed - adjustment)
            LMOTOR_PWM.duty_u16(leftspeed)
            RMOTOR_PWM.duty_u16(rightspeed)
            time.sleep(2)
            LMOTOR_PWM.duty_u16(0)
            RMOTOR_PWM.duty_u16(0)

###################################################################################
######## Line follower code #######################################################
###################################################################################
def photoread():
    global leftside, leftfront, rightfront, rightside
    TRIGGER_PIN.value(1)     # switch on LEDs on sensor board
    time.sleep(0.02)
    leftside = Lsidesense.read_u16()
    leftfront = Lfrontsense.read_u16()
    rightfront = Rfrontsense.read_u16()
    rightside = Rsidesense.read_u16()
    TRIGGER_PIN.value(0)
    return

# Line follower code
def widelinefollow():
    global leftfront, rightfront, leftspeed, rightspeed
    basespeed = 8000 - slowdownby
    side = 0
    adjustment = 0
    prevdiff = 0
    RMOTOR_DIR.value(1)  # set the right motor direction
    LMOTOR_DIR.value(1)  # set the left motor direction
    while True:
        photoread()
        if (leftfront < 10000):            # put LED indicators on if white line seen
            LEFT_LED.value(True)
        else :
            LEFT_LED.value(False)
            
        if (rightfront < 10000):
            RIGHT_LED.value(True)
        else :
            RIGHT_LED.value(False)       
        
        sensordiff = (leftfront - rightfront) / 6
        dterm = (sensordiff - prevdiff) * 3
        
        if (sensordiff > 40000):
            sensordiff = 40000
        
        if (sensordiff < -40000):
            sensordiff = -40000
            
        if ((leftfront > 20000) and (rightfront > 20000)):  # gone off the line
            if (side == 1):
                adjustment = 5000
            if (side == 2):
                adjustment = -5000
        else:                             # proportional control
            adjustment = sensordiff
            if (leftfront > rightfront):
                side = 1
            if (rightfront > leftfront):
                side = 2
            adjustment = adjustment + dterm                     
        
        leftspeed = int(basespeed + adjustment)
        rightspeed = int(basespeed - adjustment)
        checkspeed()
        
        LMOTOR_PWM.duty_u16(leftspeed)
        RMOTOR_PWM.duty_u16(rightspeed)
        
        prevdiff = sensordiff
        print(sensordiff, adjustment, leftspeed, rightspeed,)    

def readSensors():
        global leftside,leftside_dark,leftside_light, leftfront,leftfront_dark,leftfront_light, rightfront,rightfront_dark,rightfront_light, rightside,rightside_dark,rightside_light        
        TRIGGER_PIN.value(0)     # switch off LEDs on sensor board
        time.sleep(0.01)        
        leftside_dark = Lsidesense.read_u16()
        leftfront_dark = Lfrontsense.read_u16()
        rightfront_dark = Rfrontsense.read_u16()
        rightside_dark = Rsidesense.read_u16()
        
        TRIGGER_PIN.value(1)     # switch on LEDs on sensor board
        time.sleep(0.01)
        
        leftside_light = Lsidesense.read_u16()
        leftfront_light = Lfrontsense.read_u16()
        rightfront_light = Rfrontsense.read_u16()
        rightside_light = Rsidesense.read_u16()

        leftside = math.fabs(leftside_light - leftside_dark)
        leftfront = math.fabs(leftfront_light - leftfront_dark)
        rightfront = math.fabs(rightfront_light - rightfront_dark)
        rightside = math.fabs(rightside_light - rightside_dark)      

def lineFollow(): #Black Line Follow on White surface
    global leftside,leftside_dark,leftside_light, leftfront,leftfront_dark,leftfront_light, rightfront,rightfront_dark,rightfront_light, rightside,rightside_dark,rightside_light, leftspeed, rightspeed
    updateTime = time.time() + updateInterval
    basespeed = 8000
    side = 0
    adjustment = 0
    prevdiff = 0
    RMOTOR_DIR.value(1)  # set the right motor direction
    LMOTOR_DIR.value(1)  # set the left motor direction
    while True:
        readSensors()
        
        if (rightfront < FRONTRIGHT_MARKER_THRESHOLD) :
            RIGHT_LED.value(1) # switch off sensor2 LED on line folllower board
        else:
            RIGHT_LED.value(0) # switch off sensor2 LED on line folllower board
        
        if (leftfront < FRONTLEFT_MARKER_THRESHOLD) :
            LEFT_LED.value(1) # switch off sensor2 LED on line folllower board
        else:
            LEFT_LED.value(0) # switch off sensor2 LED on line folllower board
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
            adjustment = -1000
        
        if(gSensorDifference < -2000):
            adjustment = 1000
        
        leftspeed = int(basespeed + adjustment)
        rightspeed = int(basespeed - adjustment)
        
        LMOTOR_PWM.duty_u16(leftspeed)
        RMOTOR_PWM.duty_u16(rightspeed)
        

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
Leftenc1.irq(lambda p:leftcount())
Rightenc1.irq(lambda p:rightcount())

LEFT_LED.value(0) # switch off sensor1 LED on line folllower board
RIGHT_LED.value(0) # switch off sensor2 LED on line folllower board
LMOTOR_PWM.duty_u16(0) # in range 0 to 65535
RMOTOR_PWM.duty_u16(0)

# wait for onboard button to be pressed
setting = 1
while (setting == True): # Check button 1 (GP20)
    setting = btn1.value()
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

#widelinefollow()
lineFollow()

LEFT_LED.value(0) # switch off sensor1 LED on line folllower board
RIGHT_LED.value(0) # switch off sensor2 LED on line folllower board
LMOTOR_PWM.duty_u16(0) # in range 0 to 65535
RMOTOR_PWM.duty_u16(0)
